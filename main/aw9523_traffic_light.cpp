// main/aw9523_traffic_light.cpp

#include "aw9523_traffic_light.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "AW9523_TL";

// Gebruik dezelfde I2C-config als POST_I2C_* in post_diagnostics.cpp
#define AW9523_I2C_PORT       I2C_NUM_1
#define AW9523_I2C_SDA_GPIO   GPIO_NUM_3
#define AW9523_I2C_SCL_GPIO   GPIO_NUM_43
#define AW9523_I2C_FREQ_HZ    100000

#define AW9523_I2C_ADDR       0x58

// Registers
#define AW9523_REG_GCR          0x11
#define AW9523_REG_LED_MODE_P0  0x12
#define AW9523_REG_LED_MODE_P1  0x13

// Stoplicht / “relais” pinmapping
static constexpr uint8_t PIN_GREEN_LED   = 1;
static constexpr uint8_t PIN_GREEN_REL   = 2;   // “relay” groen
static constexpr uint8_t PIN_YELLOW_LED  = 6;
static constexpr uint8_t PIN_YELLOW_REL  = 7;   // “relay” geel
static constexpr uint8_t PIN_RED_LED     = 15;

// ------------------------------------------------------------
// I2C state
// ------------------------------------------------------------
static i2c_master_bus_handle_t s_i2c_bus = nullptr;
static i2c_master_dev_handle_t s_aw_dev  = nullptr;

// ------------------------------------------------------------
// Lage-level helpers
// ------------------------------------------------------------

static esp_err_t aw9523_write_reg(uint8_t reg, uint8_t value)
{
    if (!s_aw_dev) {
        ESP_LOGE(TAG, "aw9523_write_reg: device handle is null");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[2] = { reg, value };
    esp_err_t err = i2c_master_transmit(s_aw_dev, buf, sizeof(buf), 50 /* ms */);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t aw9523_read_reg(uint8_t reg, uint8_t* value)
{
    if (!s_aw_dev || !value) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = i2c_master_transmit_receive(
        s_aw_dev,
        &reg, 1,
        value, 1,
        50
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
    return err;
}

// pin 0..15 → DIM-register (volgens datasheet/Adafruit)
static uint8_t aw9523_dim_reg_for_pin(uint8_t pin)
{
    // DIM0..15: 0x20..0x2F
    // DIM0  (0x20) = P1_0
    // DIM1  (0x21) = P1_1
    // DIM2  (0x22) = P1_2
    // DIM3  (0x23) = P1_3
    // DIM4  (0x24) = P0_0
    // ...
    if (pin <= 7) {
        // P0_0..P0_7 → DIM4..DIM11
        return 0x24 + pin;          // 0x24 = DIM4
    } else if (pin <= 11) {
        // P1_0..P1_3 → DIM0..DIM3
        return 0x20 + (pin - 8);    // 0x20 = DIM0
    } else {
        // P1_4..P1_7 → DIM12..DIM15
        return 0x2C + (pin - 12);   // 0x2C = DIM12
    }
}

// Bij onze eerdere werkende versie: 0 = LED-mode, 1 = GPIO-mode
static esp_err_t aw9523_set_led_mode_for_pin(uint8_t pin)
{
    uint8_t reg_addr;
    uint8_t bit;

    if (pin <= 7) {
        reg_addr = AW9523_REG_LED_MODE_P0;
        bit      = pin;
    } else {
        reg_addr = AW9523_REG_LED_MODE_P1;
        bit      = pin - 8;
    }

    uint8_t reg_val = 0;
    ESP_RETURN_ON_ERROR(aw9523_read_reg(reg_addr, &reg_val), TAG, "read LED_MODE failed");

    reg_val &= ~(1u << bit);   // 0 = LED-mode
    return aw9523_write_reg(reg_addr, reg_val);
}

static esp_err_t aw9523_set_led_level(uint8_t pin, uint8_t level)
{
    uint8_t dim_reg = aw9523_dim_reg_for_pin(pin);
    return aw9523_write_reg(dim_reg, level);
}

// ------------------------------------------------------------
// Fade-helper: één kleur LED faden + “relais”-LEDs op max
// ------------------------------------------------------------
static void aw9523_fade_single(uint8_t active_pin, TickType_t step_delay)
{
    // RELAIS: we gebruiken hiervoor gewoon de LED-dimmer op pin 2 en 7
    bool green_phase  = (active_pin == PIN_GREEN_LED);
    bool yellow_phase = (active_pin == PIN_YELLOW_LED);

    // Aan het begin van de fase: bijpassende “relais”-LED op vol (0xFF)
    aw9523_set_led_level(PIN_GREEN_REL,  green_phase  ? 0xFF : 0x00);
    aw9523_set_led_level(PIN_YELLOW_REL, yellow_phase ? 0xFF : 0x00);

    // Fade-in: 0 -> 255
    for (int level = 0; level <= 255; level += 4) {
        uint8_t l = static_cast<uint8_t>(level);

        aw9523_set_led_level(PIN_GREEN_LED,
                             (active_pin == PIN_GREEN_LED)  ? l : 0);
        aw9523_set_led_level(PIN_YELLOW_LED,
                             (active_pin == PIN_YELLOW_LED) ? l : 0);
        aw9523_set_led_level(PIN_RED_LED,
                             (active_pin == PIN_RED_LED)    ? l : 0);

        vTaskDelay(step_delay);
    }

    // Fade-out: 255 -> 0
    for (int level = 255; level >= 0; level -= 4) {
        uint8_t l = static_cast<uint8_t>(level);

        aw9523_set_led_level(PIN_GREEN_LED,
                             (active_pin == PIN_GREEN_LED)  ? l : 0);
        aw9523_set_led_level(PIN_YELLOW_LED,
                             (active_pin == PIN_YELLOW_LED) ? l : 0);
        aw9523_set_led_level(PIN_RED_LED,
                             (active_pin == PIN_RED_LED)    ? l : 0);

        vTaskDelay(step_delay);
    }

    // Aan het eind: alle echte stoplicht-LEDs uit
    aw9523_set_led_level(PIN_GREEN_LED,  0);
    aw9523_set_led_level(PIN_YELLOW_LED, 0);
    aw9523_set_led_level(PIN_RED_LED,    0);

    // “Relais”-LEDs ook uit; volgende fase zet de juiste weer aan
    aw9523_set_led_level(PIN_GREEN_REL,  0x00);
    aw9523_set_led_level(PIN_YELLOW_REL, 0x00);
}

// ------------------------------------------------------------
// Publieke API voor main.cpp
// ------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t aw9523_traffic_light_init(void)
{
    esp_err_t err;

    // 1) I2C-bus init (nieuwe i2c_master API)
    if (!s_i2c_bus) {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port          = AW9523_I2C_PORT;
        bus_cfg.sda_io_num        = AW9523_I2C_SDA_GPIO;
        bus_cfg.scl_io_num        = AW9523_I2C_SCL_GPIO;
        bus_cfg.clk_source        = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.intr_priority     = 0;
        bus_cfg.trans_queue_depth = 0;
        bus_cfg.flags.enable_internal_pullup = true;
        bus_cfg.flags.allow_pd               = false;

        err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "I2C master bus created on SDA=%d, SCL=%d",
                 (int)AW9523_I2C_SDA_GPIO, (int)AW9523_I2C_SCL_GPIO);
    }

    // 2) Device-handle
    if (!s_aw_dev) {
        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address  = AW9523_I2C_ADDR;
        dev_cfg.scl_speed_hz    = AW9523_I2C_FREQ_HZ;

        err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_aw_dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_master_bus_add_device (0x%02X) failed: %s",
                     AW9523_I2C_ADDR, esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "AW9523 device added at 0x%02X", AW9523_I2C_ADDR);
    }

    // 3) (optioneel) GCR loggen
    uint8_t gcr = 0;
    if (aw9523_read_reg(AW9523_REG_GCR, &gcr) == ESP_OK) {
        ESP_LOGI(TAG, "AW9523 GCR = 0x%02X", gcr);
    }

    // 4) Alle relevante pins in LED-mode:
    //    stoplicht: 1, 6, 15
    //    “relais”:  2, 7
    ESP_RETURN_ON_ERROR(aw9523_set_led_mode_for_pin(PIN_GREEN_LED),  TAG, "LED mode pin 1 failed");
    ESP_RETURN_ON_ERROR(aw9523_set_led_mode_for_pin(PIN_GREEN_REL),  TAG, "LED mode pin 2 failed");
    ESP_RETURN_ON_ERROR(aw9523_set_led_mode_for_pin(PIN_YELLOW_LED), TAG, "LED mode pin 6 failed");
    ESP_RETURN_ON_ERROR(aw9523_set_led_mode_for_pin(PIN_YELLOW_REL), TAG, "LED mode pin 7 failed");
    ESP_RETURN_ON_ERROR(aw9523_set_led_mode_for_pin(PIN_RED_LED),    TAG, "LED mode pin 15 failed");

    // 5) Alles uit
    ESP_RETURN_ON_ERROR(aw9523_set_led_level(PIN_GREEN_LED,  0x00), TAG, "init level pin1 failed");
    ESP_RETURN_ON_ERROR(aw9523_set_led_level(PIN_GREEN_REL,  0x00), TAG, "init level pin2 failed");
    ESP_RETURN_ON_ERROR(aw9523_set_led_level(PIN_YELLOW_LED, 0x00), TAG, "init level pin6 failed");
    ESP_RETURN_ON_ERROR(aw9523_set_led_level(PIN_YELLOW_REL, 0x00), TAG, "init level pin7 failed");
    ESP_RETURN_ON_ERROR(aw9523_set_led_level(PIN_RED_LED,    0x00), TAG, "init level pin15 failed");

    ESP_LOGI(TAG, "AW9523 traffic light init done");
    return ESP_OK;
}

void aw9523_traffic_light_run(void)
{
    if (aw9523_traffic_light_init() != ESP_OK) {
        ESP_LOGE(TAG, "Init failed, aborting traffic light demo");
        return;
    }

    const TickType_t step_delay = pdMS_TO_TICKS(20);   // ~20 ms per stap

    while (true) {
        ESP_LOGI(TAG, "Traffic light: fade GREEN");
        aw9523_fade_single(PIN_GREEN_LED, step_delay);

        ESP_LOGI(TAG, "Traffic light: fade YELLOW");
        aw9523_fade_single(PIN_YELLOW_LED, step_delay);

        ESP_LOGI(TAG, "Traffic light: fade RED");
        aw9523_fade_single(PIN_RED_LED, step_delay);
    }
}

#ifdef __cplusplus
}
#endif
