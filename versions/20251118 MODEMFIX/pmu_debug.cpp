/*
 * main/pmu_debug.cpp
 *
 * PMU / GPIO debug dump for T-SIM7080G-S3 (AXP2101)
 *
 * - Uses the *new* I2C master driver (driver/i2c_master.h)
 * - Provides shared AXP2101 read/write functions for other modules
 * - Dumps AXP2101 registers 0x00-0xFF
 * - Decodes DCDC3 and BLDO2 voltages
 * - Prints a GPIO snapshot for modem/PMU-related pins
 */

#include <cstdio>
#include <cstdint>

#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "pmu_debug.h"

static const char *TAG_PMU = "PMU_DUMP";

// -----------------------------------------------------------------------------
// Board / PMU configuration (match T-SIM7080G-S3)
// -----------------------------------------------------------------------------

// AXP2101 I2C bus: I2C_NUM_0, SDA=15, SCL=7
#define PMU_I2C_PORT       I2C_NUM_0
#define PMU_SDA_GPIO       GPIO_NUM_15
#define PMU_SCL_GPIO       GPIO_NUM_7
#define PMU_I2C_FREQ_HZ    400000

// AXP2101 7-bit address
#define AXP2101_I2C_ADDR   0x34

// Some important registers
#define AXP2101_REG_DCDC_ONOFF   0x80
#define AXP2101_REG_DCDC3_VOLT   0x84
#define AXP2101_REG_LDO_ONOFF0   0x90
#define AXP2101_REG_BLDO2_VOLT   0x97

// Modem / SD pins to snapshot
#define SNAP_GPIO_MODEM_RI    GPIO_NUM_3
#define SNAP_GPIO_MODEM_RX    GPIO_NUM_4
#define SNAP_GPIO_MODEM_TX    GPIO_NUM_5
#define SNAP_GPIO_PMU_INT     GPIO_NUM_6
#define SNAP_GPIO_PMU_SCL     GPIO_NUM_7
#define SNAP_GPIO_PMU_SDA     GPIO_NUM_15
#define SNAP_GPIO_SD_CLK      GPIO_NUM_38
#define SNAP_GPIO_SD_CMD      GPIO_NUM_39
#define SNAP_GPIO_SD_D0       GPIO_NUM_40
#define SNAP_GPIO_MODEM_PWR   GPIO_NUM_41
#define SNAP_GPIO_MODEM_DTR   GPIO_NUM_42

// -----------------------------------------------------------------------------
// New I2C master state
// -----------------------------------------------------------------------------

static i2c_master_bus_handle_t  s_pmu_bus = nullptr;
static i2c_master_dev_handle_t  s_pmu_dev = nullptr;

/**
 * @brief Initialise I2C bus and AXP2101 device handle (new driver).
 */
esp_err_t pmu_axp2101_init(void)
{
    if (s_pmu_dev) {
        return ESP_OK;
    }

    esp_err_t err;

    if (!s_pmu_bus) {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port          = PMU_I2C_PORT;
        bus_cfg.sda_io_num        = PMU_SDA_GPIO;
        bus_cfg.scl_io_num        = PMU_SCL_GPIO;
        bus_cfg.clk_source        = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.intr_priority     = 0;
        bus_cfg.trans_queue_depth = 0;
        bus_cfg.flags.enable_internal_pullup = true;   // ✅ hier wél

        err = i2c_new_master_bus(&bus_cfg, &s_pmu_bus);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_PMU, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = AXP2101_I2C_ADDR;
    dev_cfg.scl_speed_hz    = PMU_I2C_FREQ_HZ;

    err = i2c_master_bus_add_device(s_pmu_bus, &dev_cfg, &s_pmu_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_PMU, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        s_pmu_dev = nullptr;
        return err;
    }

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Shared AXP2101 helpers (C-linkage via pmu_debug.h)
// -----------------------------------------------------------------------------

esp_err_t axp2101_read_reg(uint8_t reg, uint8_t *value)
{
    esp_err_t err = pmu_axp2101_init();
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_transmit_receive(
        s_pmu_dev,
        &reg,
        1,
        value,
        1,
        50 // ms
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG_PMU, "AXP2101: read reg 0x%02X failed: %s",
                 reg, esp_err_to_name(err));
    }
    return err;
}

esp_err_t axp2101_write_reg(uint8_t reg, uint8_t value)
{
    esp_err_t err = pmu_axp2101_init();
    if (err != ESP_OK) {
        return err;
    }

    uint8_t buf[2] = { reg, value };
    err = i2c_master_transmit(
        s_pmu_dev,
        buf,
        sizeof(buf),
        50 // ms
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG_PMU, "AXP2101: write reg 0x%02X failed: %s",
                 reg, esp_err_to_name(err));
    }
    return err;
}

// -----------------------------------------------------------------------------
// Voltage decoding helpers
// -----------------------------------------------------------------------------

static float decode_dcdc3_voltage(uint8_t reg_val)
{
    uint8_t code = reg_val & 0x7F;

    // 0.5–1.2 V, 10 mV steps
    if (code <= 0x46) {
        return 0.50f + 0.01f * code;
    }
    // 1.22–1.54 V, 20 mV steps
    else if (code <= 0x57) {
        return 1.22f + 0.02f * (code - 0x47);
    }
    // 1.60–3.40 V, 100 mV steps
    else if (code <= 0x6B) {
        return 1.60f + 0.10f * (code - 0x58);
    }

    // Reserved / invalid
    return -1.0f;
}

static float decode_bldo2_voltage(uint8_t reg_val)
{
    uint8_t code = reg_val & 0x1F;
    return 0.5f + 0.1f * code;   // 0.5–3.5 V in 0.1 V steps
}

// -----------------------------------------------------------------------------
// GPIO snapshot
// -----------------------------------------------------------------------------

static void pmu_debug_dump_gpio_snapshot(void)
{
    ESP_LOGI(TAG_PMU, "=== GPIO level snapshot ===");

    const gpio_num_t pins[] = {
        SNAP_GPIO_MODEM_RI,
        SNAP_GPIO_MODEM_RX,
        SNAP_GPIO_MODEM_TX,
        SNAP_GPIO_PMU_INT,
        SNAP_GPIO_PMU_SCL,
        SNAP_GPIO_PMU_SDA,
        SNAP_GPIO_SD_CLK,
        SNAP_GPIO_SD_CMD,
        SNAP_GPIO_SD_D0,
        SNAP_GPIO_MODEM_PWR,
        SNAP_GPIO_MODEM_DTR
    };

    for (size_t i = 0; i < sizeof(pins)/sizeof(pins[0]); ++i) {
        gpio_num_t pin = pins[i];
        gpio_config_t io = {};
        io.pin_bit_mask = (1ULL << pin);
        io.mode         = GPIO_MODE_INPUT;
        io.pull_up_en   = GPIO_PULLUP_DISABLE;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io.intr_type    = GPIO_INTR_DISABLE;
        gpio_config(&io);

        int level = gpio_get_level(pin);
        ESP_LOGI(TAG_PMU, "GPIO%02d: level=%d", (int)pin, level);
    }

    ESP_LOGI(TAG_PMU, "=== End of GPIO snapshot ===");
}

// -----------------------------------------------------------------------------
// Public debug dump
// -----------------------------------------------------------------------------

void pmu_debug_dump(void)
{
    ESP_LOGI(TAG_PMU, "===========================================");
    ESP_LOGI(TAG_PMU, " PMU / GPIO debug dump (T-SIM7080G-S3)");
    ESP_LOGI(TAG_PMU, "===========================================\n");

    // --- Raw register dump ---
    ESP_LOGI(TAG_PMU, "=== AXP2101 register dump (0x00-0xFF) ===");

    esp_err_t err;
    uint8_t buf[16];

    // NOTE: base is now 'int' to avoid uint8_t wrap-around after 0xF0.
    for (int base = 0x00; base <= 0xF0; base += 0x10) {
        for (int i = 0; i < 16; ++i) {
            uint8_t reg = static_cast<uint8_t>(base + i);
            err = axp2101_read_reg(reg, &buf[i]);
            if (err != ESP_OK) {
                ESP_LOGW(TAG_PMU,
                         "AXP2101: read reg 0x%02X failed: %s",
                         reg, esp_err_to_name(err));
                buf[i] = 0x00;
            }
        }

        char line[3 + 2 + 2 + 1 + 3 * 16 + 1]; // "0xXX: " + "XX " * 16 + '\0'
        int n = std::snprintf(line, sizeof(line),
                              "0x%02X: "
                              "%02X %02X %02X %02X %02X %02X %02X %02X "
                              "%02X %02X %02X %02X %02X %02X %02X %02X",
                              base,
                              buf[0],  buf[1],  buf[2],  buf[3],
                              buf[4],  buf[5],  buf[6],  buf[7],
                              buf[8],  buf[9],  buf[10], buf[11],
                              buf[12], buf[13], buf[14], buf[15]);

        if (n > 0) {
            ESP_LOGI(TAG_PMU, "%s", line);
        }
    }

    ESP_LOGI(TAG_PMU, "=== End of AXP2101 dump ===");

    // --- Human-readable summary (DCDC3 + BLDO2) ---
    uint8_t dcdc_onoff = 0;
    uint8_t dcdc3_val  = 0;
    uint8_t ldo_onoff0 = 0;
    uint8_t bldo2_val  = 0;

    if (axp2101_read_reg(AXP2101_REG_DCDC_ONOFF, &dcdc_onoff) == ESP_OK &&
        axp2101_read_reg(AXP2101_REG_DCDC3_VOLT, &dcdc3_val) == ESP_OK &&
        axp2101_read_reg(AXP2101_REG_LDO_ONOFF0, &ldo_onoff0) == ESP_OK &&
        axp2101_read_reg(AXP2101_REG_BLDO2_VOLT, &bldo2_val) == ESP_OK) {

        bool dcdc3_enabled = (dcdc_onoff & (1 << 2)) != 0; // bit2 = DCDC3
        bool bldo2_enabled = (ldo_onoff0 & (1 << 5)) != 0; // bit5 = BLDO2

        float dcdc3_v = decode_dcdc3_voltage(dcdc3_val);
        float bldo2_v = decode_bldo2_voltage(bldo2_val);

        ESP_LOGI(TAG_PMU, "");
        ESP_LOGI(TAG_PMU, "--- Decoded PMU summary ---");
        ESP_LOGI(TAG_PMU, "DCDC_ONOFF (0x80) = 0x%02X", dcdc_onoff);
        ESP_LOGI(TAG_PMU, "LDO_ONOFF0 (0x90) = 0x%02X", ldo_onoff0);
        ESP_LOGI(TAG_PMU, "DCDC3_VOLT (0x84) = 0x%02X -> %s, %.2f V",
                 dcdc3_val,
                 dcdc3_enabled ? "ENABLED" : "DISABLED",
                 dcdc3_v);
        ESP_LOGI(TAG_PMU, "BLDO2_VOLT (0x97) = 0x%02X -> %s, %.2f V",
                 bldo2_val,
                 bldo2_enabled ? "ENABLED" : "DISABLED",
                 bldo2_v);
        ESP_LOGI(TAG_PMU, "---------------------------");
    } else {
        ESP_LOGW(TAG_PMU, "Could not read DCDC/BLDO summary registers");
    }

    ESP_LOGI(TAG_PMU, "");
    pmu_debug_dump_gpio_snapshot();

    ESP_LOGI(TAG_PMU, "");
    ESP_LOGI(TAG_PMU, "PMU / GPIO debug dump completed");
}
