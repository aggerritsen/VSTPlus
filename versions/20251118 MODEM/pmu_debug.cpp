#include "pmu_debug.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PMU_DUMP";

// AXP2101 I2C address on T-SIM7080G-S3
static const uint8_t AXP2101_ADDR = 0x34;

// PMU I2C pins on T-SIM7080G-S3
static const i2c_port_t PMU_I2C_PORT = I2C_NUM_0;
static const gpio_num_t PMU_SDA_GPIO = GPIO_NUM_15;
static const gpio_num_t PMU_SCL_GPIO = GPIO_NUM_7;

// GPIOs we care about for modem / board state
static const gpio_num_t kPinsToDump[] = {
    GPIO_NUM_3,   // RI / I2C1 SDA for sensors
    GPIO_NUM_4,   // MODEM_RXD (ESP RX)
    GPIO_NUM_5,   // MODEM_TXD (ESP TX)
    GPIO_NUM_6,   // PMU INT
    GPIO_NUM_7,   // PMU SCL
    GPIO_NUM_15,  // PMU SDA
    GPIO_NUM_38,  // SD CLK
    GPIO_NUM_39,  // SD CMD
    GPIO_NUM_40,  // SD D0
    GPIO_NUM_41,  // MODEM_PWRKEY
    GPIO_NUM_42   // MODEM_DTR
};

static esp_err_t pmu_i2c_init_for_dump(void)
{
    // We don't want to fight with the existing legacy driver.
    // If it's already configured/installed, we just accept that quietly.

    static bool s_inited = false;
    if (s_inited) {
        return ESP_OK;
    }

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PMU_SDA_GPIO;
    conf.scl_io_num = PMU_SCL_GPIO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;

    esp_err_t err = i2c_param_config(PMU_I2C_PORT, &conf);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "i2c_param_config failed (but continuing): %s", esp_err_to_name(err));
    }

    err = i2c_driver_install(PMU_I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // Most likely: driver already installed by other code and old driver complains.
        ESP_LOGW(TAG, "i2c_driver_install failed (but continuing): %s", esp_err_to_name(err));
    }

    s_inited = true;
    return ESP_OK;
}

static bool axp2101_read_reg_dump(uint8_t reg, uint8_t *val)
{
    if (!val) {
        return false;
    }

    if (pmu_i2c_init_for_dump() != ESP_OK) {
        return false;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return false;
    }

    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP2101_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    // Repeated start + read
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP2101_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(PMU_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "read reg 0x%02X failed: %s", reg, esp_err_to_name(err));
        return false;
    }

    return true;
}

static void pmu_dump_axp2101_registers(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== AXP2101 register dump (0x00-0xFF) ===");

    if (pmu_i2c_init_for_dump() != ESP_OK) {
        ESP_LOGE(TAG, "AXP2101: I2C init failed, cannot dump");
        return;
    }

    // Simple presence check
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP2101_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(PMU_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AXP2101 not responding at 0x%02X (err=%s)",
                 AXP2101_ADDR, esp_err_to_name(err));
        return;
    }

    for (uint16_t base = 0x00; base <= 0xF0; base += 0x10) {
        char line[128];
        int len = snprintf(line, sizeof(line), "0x%02X:", base);

        for (uint8_t off = 0; off < 0x10; ++off) {
            uint8_t reg = base + off;
            uint8_t val = 0;
            bool ok = axp2101_read_reg_dump(reg, &val);
            if (ok) {
                len += snprintf(line + len, sizeof(line) - len, " %02X", val);
            } else {
                len += snprintf(line + len, sizeof(line) - len, " ??");
            }
            if (len >= (int)sizeof(line)) {
                break;
            }
        }

        ESP_LOGI(TAG, "%s", line);
    }

    ESP_LOGI(TAG, "=== End of AXP2101 dump ===");
    ESP_LOGI(TAG, "");
}

static void pmu_dump_gpio_levels(void)
{
    ESP_LOGI(TAG, "=== GPIO level snapshot ===");

    for (size_t i = 0; i < sizeof(kPinsToDump) / sizeof(kPinsToDump[0]); ++i) {
        gpio_num_t pin = kPinsToDump[i];

        gpio_config_t cfg = {};
        cfg.pin_bit_mask = (1ULL << pin);
        cfg.mode = GPIO_MODE_INPUT;
        cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&cfg);

        int level = gpio_get_level(pin);
        ESP_LOGI(TAG, "GPIO%02d: level=%d", (int)pin, level);
    }

    ESP_LOGI(TAG, "=== End of GPIO snapshot ===");
    ESP_LOGI(TAG, "");
}

extern "C" void pmu_debug_dump_state(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, " PMU / GPIO debug dump (T-SIM7080G-S3)     ");
    ESP_LOGI(TAG, "===========================================");

    pmu_dump_axp2101_registers();
    pmu_dump_gpio_levels();

    ESP_LOGI(TAG, "PMU / GPIO debug dump completed");
}
