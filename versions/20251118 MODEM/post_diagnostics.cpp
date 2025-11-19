/*
 * main/post_diagnostics.cpp
 *
 * Power-On Self Test (POST) module for VSTPlus on T-SIM7080G-S3
 *
 * - Logs chip / flash / PSRAM / heap info to the serial console
 * - Scans I2C bus for devices
 * - Brings up the SIM7080 modem power (via PMU stub) and checks AT response
 * - Attempts to mount SD card on SDMMC host slot 1 (1-bit bus, GPIO 38/39/40)
 * - Writes diagnostics to /sdcard/boot_diagnostics.txt on success
 */

#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <ctime>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_check.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pmu_debug.h"


static const char *TAG = "POST";
#define TAG_POST TAG

// -----------------------------------------------------------------------------
// Board / wiring configuration
// -----------------------------------------------------------------------------

// ----- PMU (AXP2101) on T-SIM7080G-S3 -----
// DCDC control and DCDC3 voltage (op basis van AXP2101 datasheet + dumps)
#define AXP2101_REG_DCDC_ONOFF    0x80   // DCDCS ON/OFF and DVM control
#define AXP2101_REG_DCDC3_VOLT    0x84   // DCDC3 voltage setting

// PMU I2C is on I2C_NUM_0, SDA=15, SCL=7, INT=6
#define POST_PMU_I2C_PORT   I2C_NUM_0
#define POST_PMU_SDA_GPIO   GPIO_NUM_15   // PMU SDA
#define POST_PMU_SCL_GPIO   GPIO_NUM_7    // PMU SCL
#define POST_PMU_I2C_FREQ   400000

// AXP2101 TWSI 7-bit address: 0b0110100 = 0x34
#define AXP2101_I2C_ADDR    0x34

// Registers we actually use (from AXP2101 register map)
#define AXP2101_REG_LDO_ONOFF0   0x90   // LDO ON/OFF control 0, bit5 = BLDO2 enable
#define AXP2101_REG_BLDO2_VOLT   0x97   // BLDO2 voltage setting
//  bits4:0 = BLDO2 voltage code: 0.5V + 0.1V*code (0..30)
//  3.3V => code = (3.3 - 0.5) / 0.1 = 28 = 0b11100

// SD card configuration: SDMMC host slot 1, 1-bit bus on 38/39/40
#define POST_MOUNT_POINT   "/sdcard"
#define POST_DIAG_FILE     POST_MOUNT_POINT "/boot_diagnostics.txt"

#define POST_SD_CLK_GPIO   GPIO_NUM_38
#define POST_SD_CMD_GPIO   GPIO_NUM_39
#define POST_SD_D0_GPIO    GPIO_NUM_40

// I2C probe configuration (camera-safe bus for external sensors)
#define POST_I2C_PORT      I2C_NUM_1
#define POST_I2C_SDA_GPIO  GPIO_NUM_3
#define POST_I2C_SCL_GPIO  GPIO_NUM_43
#define POST_I2C_FREQ_HZ   100000

// PMU (AXP2101) INT pin (not used yet, but defined for completeness)
#define PMU_INT_GPIO       GPIO_NUM_6

// ----- Modem pins (from your notes) -----
#define BOARD_MODEM_PWR_PIN  GPIO_NUM_41   // PWRKEY
#define BOARD_MODEM_DTR_PIN  GPIO_NUM_42   // DTR
#define BOARD_MODEM_RI_PIN   GPIO_NUM_3    // RI (we just read it as input)
#define BOARD_MODEM_RXD_PIN  GPIO_NUM_4    // ESP RX  <- modem TX
#define BOARD_MODEM_TXD_PIN  GPIO_NUM_5    // ESP TX  -> modem RX

// Use UART1 on those pins
#define POST_MODEM_UART_NUM   UART_NUM_1
#define POST_MODEM_TX_GPIO    BOARD_MODEM_TXD_PIN
#define POST_MODEM_RX_GPIO    BOARD_MODEM_RXD_PIN
#define POST_MODEM_BAUD       115200

// How long a single AT probe waits for a response (in microseconds)
#define POST_MODEM_TIMEOUT_US 1000000      // 1 s per AT attempt
// How many AT attempts per phase
#define POST_MODEM_MAX_RETRY  7           // retries before/after PWRKEY pulse

// Diagnostics buffer configuration
#define POST_MAX_DIAG_LEN  (8 * 1024)

// -----------------------------------------------------------------------------
// Internal state
// -----------------------------------------------------------------------------

static char   s_diag_buf[POST_MAX_DIAG_LEN];
static size_t s_diag_len = 0;

// Forward declaration so we can use post_log earlier in the file
static void post_log(const char *fmt, ...);

// -----------------------------------------------------------------------------
// Small helper for GPIO tests (currently mostly a placeholder)
// -----------------------------------------------------------------------------

typedef struct {
    gpio_num_t   pin;
    const char  *name;
    int          expected_level;   // -1 if "no expectation"
} post_gpio_test_t;

// Fill this with pins that matter on your board if you want static checks.
static const post_gpio_test_t s_gpio_tests[] = {
    // { BOARD_MODEM_PWR_PIN, "MODEM_PWRKEY", -1 },
    // { BOARD_MODEM_DTR_PIN, "MODEM_DTR",    -1 },
};

// -----------------------------------------------------------------------------
// PMU helpers (AXP2101 over I2C0)
// -----------------------------------------------------------------------------

static bool s_pmu_i2c_inited = false;

static esp_err_t axp2101_i2c_init(void)
{
    if (s_pmu_i2c_inited) {
        return ESP_OK;
    }

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = POST_PMU_SDA_GPIO;
    conf.scl_io_num = POST_PMU_SCL_GPIO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = POST_PMU_I2C_FREQ;

    esp_err_t err = i2c_param_config(POST_PMU_I2C_PORT, &conf);
    if (err != ESP_OK) {
        post_log("PMU: i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(POST_PMU_I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        post_log("PMU: i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    s_pmu_i2c_inited = true;
    return ESP_OK;
}

static esp_err_t axp2101_write_reg(uint8_t reg, uint8_t value)
{
    esp_err_t err = axp2101_i2c_init();
    if (err != ESP_OK) return err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP2101_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(POST_PMU_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        post_log("PMU: write reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t axp2101_read_reg(uint8_t reg, uint8_t *value)
{
    esp_err_t err = axp2101_i2c_init();
    if (err != ESP_OK) return err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP2101_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    // Read 1 byte
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP2101_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(POST_PMU_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        post_log("PMU: read reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t post_pmu_enable_modem_rails(void)
{
    ESP_LOGI(TAG_POST, "PMU: configuring AXP2101 on SDA=%d, SCL=%d for modem power",
             POST_PMU_SDA_GPIO, POST_PMU_SCL_GPIO);

    esp_err_t err = ESP_OK;

    // 1) BLDO2 op ~3.3 V zetten (zoals in je dumps: REG 0x97 = 0x1C)
    const uint8_t bldo2_code_3v3 = 0x1C; // 3.3V => code 28
    err = axp2101_write_reg(AXP2101_REG_BLDO2_VOLT, bldo2_code_3v3);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_POST, "PMU: write BLDO2 voltage (0x%02X) failed", AXP2101_REG_BLDO2_VOLT);
        return err;
    }

    // LDO_ONOFF0 (REG 0x90): bit5 = BLDO2 enable, rest laten we ongemoeid.
    uint8_t ldo_onoff0 = 0;
    err = axp2101_read_reg(AXP2101_REG_LDO_ONOFF0, &ldo_onoff0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_POST, "PMU: read LDO_ONOFF0 (0x%02X) failed", AXP2101_REG_LDO_ONOFF0);
        return err;
    }

    ldo_onoff0 |= (1 << 5);  // BLDO2 aan
    err = axp2101_write_reg(AXP2101_REG_LDO_ONOFF0, ldo_onoff0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_POST, "PMU: write LDO_ONOFF0 (0x%02X) failed", AXP2101_REG_LDO_ONOFF0);
        return err;
    }

    // 2) DCDC3 aanzetten en op ~3.0 V zetten, zoals in de "goede" dump:
    //
    //  - REG 0x80: DCDCS ON/OFF
    //      * in slechte state: 0x19 (DCDC1,4,5 aan)
    //      * in goede  state: 0x1D (DCDC1,3,4,5 aan)
    //    -> we zetten bit2 (DCDC3 enable) erbij.
    //
    //  - REG 0x84: DCDC3 voltage
    //      * in goede state: 0x66  ≈ 3.0 V
    uint8_t dcdc_onoff = 0;
    err = axp2101_read_reg(AXP2101_REG_DCDC_ONOFF, &dcdc_onoff);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_POST, "PMU: read DCDC_ONOFF (0x%02X) failed", AXP2101_REG_DCDC_ONOFF);
        return err;
    }

    // Zet bit2 (DCDC3 enable) hoog, laat de rest zoals hij was.
    dcdc_onoff |= (1 << 2);

    err = axp2101_write_reg(AXP2101_REG_DCDC_ONOFF, dcdc_onoff);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_POST, "PMU: write DCDC_ONOFF (0x%02X) failed", AXP2101_REG_DCDC_ONOFF);
        return err;
    }

    // DCDC3 voltage naar 0x66 (≈ 3.0V), exact zoals in je werkende dump.
    err = axp2101_write_reg(AXP2101_REG_DCDC3_VOLT, 0x66);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_POST, "PMU: write DCDC3 voltage (0x%02X) failed", AXP2101_REG_DCDC3_VOLT);
        return err;
    }

    ESP_LOGI(TAG_POST,
             "PMU: BLDO2 3.3V enabled, DCDC3 enabled at ~3.0V (DCDC_ONOFF=0x%02X)",
             dcdc_onoff);

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Logging helpers
// -----------------------------------------------------------------------------

/**
 * @brief Append formatted diagnostics text to the in-memory buffer
 *        and log it to ESP logging.
 */
static void post_log(const char *fmt, ...)
{
    va_list ap;

    // Format into a local line buffer first
    char line[256];
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    if (n < 0) {
        return;
    }

    // Ensure null-termination
    line[sizeof(line) - 1] = '\0';

    // Log to normal ESP log (info-level)
    ESP_LOGI(TAG, "%s", line);

    // Append to diagnostics buffer (with newline)
    if (s_diag_len < POST_MAX_DIAG_LEN - 2) {
        size_t copy_len = strnlen(line, sizeof(line));
        if (copy_len > POST_MAX_DIAG_LEN - 2 - s_diag_len) {
            copy_len = POST_MAX_DIAG_LEN - 2 - s_diag_len;
        }

        memcpy(&s_diag_buf[s_diag_len], line, copy_len);
        s_diag_len += copy_len;

        s_diag_buf[s_diag_len++] = '\n';
        s_diag_buf[s_diag_len]   = '\0';
    }
}

/**
 * @brief Reset the diagnostics buffer.
 */
static void post_reset_buffer(void)
{
    s_diag_len       = 0;
    s_diag_buf[0]    = '\0';
}

// -----------------------------------------------------------------------------
// System info dump
// -----------------------------------------------------------------------------

static void post_dump_system_info(void)
{
    post_log("=== Power-On Self Test (POST) ===");

    // Approximate boot time
    std::time_t now = std::time(nullptr);
    std::tm tm_info{};
    if (localtime_r(&now, &tm_info) != nullptr) {
        char time_buf[64];
        std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &tm_info);
        post_log("Boot time (approx): %s", time_buf);
    } else {
        post_log("Boot time (approx): <unknown>");
    }

    // Chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    const char *chip_model_str = "unknown";
    switch (chip_info.model) {
        case CHIP_ESP32:   chip_model_str = "esp32";   break;
        case CHIP_ESP32S2: chip_model_str = "esp32s2"; break;
        case CHIP_ESP32S3: chip_model_str = "esp32s3"; break;
        case CHIP_ESP32C2: chip_model_str = "esp32c2"; break;
        case CHIP_ESP32C3: chip_model_str = "esp32c3"; break;
        case CHIP_ESP32C6: chip_model_str = "esp32c6"; break;
        case CHIP_ESP32H2: chip_model_str = "esp32h2"; break;
        default: break;
    }

    post_log("Chip model: %s, %d core(s), revision %d",
             chip_model_str, chip_info.cores, chip_info.revision);

    // Features
    post_log("Features: %s%s%s%s",
             (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "",
             (chip_info.features & CHIP_FEATURE_BLE)       ? "/ BLE " : "",
             (chip_info.features & CHIP_FEATURE_BT)        ? "/ BT "  : "",
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "/ Embedded flash" : "");

    // Flash size
    uint32_t flash_size = 0;
    if (esp_flash_get_size(nullptr, &flash_size) == ESP_OK) {
        post_log("Flash size: %u bytes (%.2f MB)",
                 (unsigned)flash_size,
                 (double)flash_size / (1024.0 * 1024.0));
    } else {
        post_log("Flash size: <unknown>");
    }

    // PSRAM
    size_t psram_size = esp_psram_get_size();
    if (psram_size > 0) {
        post_log("PSRAM: present, %u bytes (%.2f MB)",
                 (unsigned)psram_size,
                 (double)psram_size / (1024.0 * 1024.0));
    } else {
        post_log("PSRAM: not present");
    }

    // Heap stats
    multi_heap_info_t heap_info{};
    heap_caps_get_info(&heap_info, MALLOC_CAP_INTERNAL | MALLOC_CAP_DEFAULT);
    post_log("Internal heap: free=%u, largest_block=%u",
             (unsigned)heap_info.total_free_bytes,
             (unsigned)heap_info.largest_free_block);

    if (psram_size > 0) {
        heap_caps_get_info(&heap_info, MALLOC_CAP_SPIRAM);
        post_log("PSRAM heap: free=%u, largest_block=%u",
                 (unsigned)heap_info.total_free_bytes,
                 (unsigned)heap_info.largest_free_block);
    }
}

// -----------------------------------------------------------------------------
// I2C probe
// -----------------------------------------------------------------------------

static void post_test_i2c(void)
{
    post_log("I2C: scanning bus on SDA=%d, SCL=%d",
             (int)POST_I2C_SDA_GPIO, (int)POST_I2C_SCL_GPIO);

    i2c_config_t cfg = {};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = POST_I2C_SDA_GPIO;
    cfg.scl_io_num = POST_I2C_SCL_GPIO;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = POST_I2C_FREQ_HZ;

    esp_err_t err = i2c_param_config(POST_I2C_PORT, &cfg);
    if (err != ESP_OK) {
        post_log("I2C: i2c_param_config failed: %s", esp_err_to_name(err));
        return;
    }

    err = i2c_driver_install(POST_I2C_PORT, cfg.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        post_log("I2C: i2c_driver_install failed: %s", esp_err_to_name(err));
        return;
    }

    // Scan 7-bit address space
    for (uint8_t addr = 1; addr < 0x7F; ++addr) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t res = i2c_master_cmd_begin(
            POST_I2C_PORT,
            cmd,
            pdMS_TO_TICKS(20)
        );

        i2c_cmd_link_delete(cmd);

        if (res == ESP_OK) {
            post_log("I2C: found device at 0x%02X", addr);
        }
    }

    i2c_driver_delete(POST_I2C_PORT);
}

// -----------------------------------------------------------------------------
// GPIO checks (optional, currently just reports configured pins)
// -----------------------------------------------------------------------------

static void post_test_gpio(void)
{
    if (sizeof(s_gpio_tests) == 0) {
        return;
    }

    for (size_t i = 0; i < sizeof(s_gpio_tests)/sizeof(s_gpio_tests[0]); ++i) {
        const post_gpio_test_t &test = s_gpio_tests[i];

        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << test.pin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);

        int level = gpio_get_level(test.pin);
        if (test.expected_level >= 0 && level != test.expected_level) {
            post_log("GPIO: %s (GPIO%d) level=%d, expected=%d",
                     test.name, (int)test.pin, level, test.expected_level);
        } else {
            post_log("GPIO: %s (GPIO%d) level=%d",
                     test.name, (int)test.pin, level);
        }
    }
}

// -----------------------------------------------------------------------------
// Modem test helpers (manual AT over UART1)
// -----------------------------------------------------------------------------

static bool post_modem_send_at_and_wait_ok(const char *tag_prefix, int attempt)
{
    const char *cmd = "AT\r\n";

    uart_flush_input(POST_MODEM_UART_NUM);
    uart_write_bytes(POST_MODEM_UART_NUM, cmd, strlen(cmd));

    uint8_t buf[128];
    int total = 0;
    bool ok = false;

    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < POST_MODEM_TIMEOUT_US &&
           total < (int)sizeof(buf) - 1)
    {
        int n = uart_read_bytes(
            POST_MODEM_UART_NUM,
            buf + total,
            sizeof(buf) - 1 - total,
            pdMS_TO_TICKS(50)
        );
        if (n > 0) {
            total += n;
            buf[total] = 0;
            if (strstr((char *)buf, "OK")) {
                ok = true;
                break;
            }
        }
    }

    if (!ok) {
        post_log("%s: no 'OK' yet, retry=%d", tag_prefix, attempt);
    } else {
        post_log("%s: got 'OK' from modem", tag_prefix);
    }

    return ok;
}

static void post_modem_pulse_pwrkey(void)
{
    post_log("MODEM: pulsing PWRKEY (GPIO%d) to start modem", BOARD_MODEM_PWR_PIN);

    // SIM7080G PWRKEY pulse: pull low for ~500–800 ms, then high.
    gpio_set_level(BOARD_MODEM_PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(800));
    gpio_set_level(BOARD_MODEM_PWR_PIN, 1);

    // Give modem time to boot before next AT attempt
    vTaskDelay(pdMS_TO_TICKS(1500));
}

static void post_test_modem(void)
{
    post_log("MODEM: checking AT response on UART%d (TX=GPIO%d, RX=GPIO%d)",
             POST_MODEM_UART_NUM,
             POST_MODEM_TX_GPIO,
             POST_MODEM_RX_GPIO);

    // 1) Ensure modem power rail is up via PMU
    esp_err_t err = post_pmu_enable_modem_rails();
    if (err != ESP_OK) {
        post_log("MODEM: PMU configuration failed, modem may stay off");
    }

    // 2) Configure PWR / DTR / RI pins
    gpio_config_t io = {};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pin_bit_mask = (1ULL << BOARD_MODEM_PWR_PIN) | (1ULL << BOARD_MODEM_DTR_PIN);
    gpio_config(&io);

    // PWRKEY idle high, DTR high (keep modem awake)
    gpio_set_level(BOARD_MODEM_PWR_PIN, 1);
    gpio_set_level(BOARD_MODEM_DTR_PIN, 1);

    gpio_config_t ri_cfg = {};
    ri_cfg.intr_type = GPIO_INTR_DISABLE;
    ri_cfg.mode = GPIO_MODE_INPUT;
    ri_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    ri_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ri_cfg.pin_bit_mask = (1ULL << BOARD_MODEM_RI_PIN);
    gpio_config(&ri_cfg);

    // 3) Configure UART1 on TX=5, RX=4
    uart_config_t cfg = {};
    cfg.baud_rate = POST_MODEM_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    uart_param_config(POST_MODEM_UART_NUM, &cfg);
    uart_set_pin(POST_MODEM_UART_NUM,
                 POST_MODEM_TX_GPIO,
                 POST_MODEM_RX_GPIO,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    uart_driver_install(POST_MODEM_UART_NUM, 2048, 0, 0, NULL, 0);

    // 4) Try a few plain ATs first (in case modem is already up)
    bool ok = false;
    for (int i = 0; i < POST_MODEM_MAX_RETRY && !ok; ++i) {
        ok = post_modem_send_at_and_wait_ok("MODEM", i);
        if (!ok) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // 5) If still no OK, pulse PWRKEY and try again
    if (!ok) {
        post_modem_pulse_pwrkey();

        for (int i = 0; i < POST_MODEM_MAX_RETRY && !ok; ++i) {
            ok = post_modem_send_at_and_wait_ok("MODEM", i);
            if (!ok) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }

    if (!ok) {
        post_log("MODEM: no response after PWRKEY pulse");
    }

    uart_driver_delete(POST_MODEM_UART_NUM);
}

// -----------------------------------------------------------------------------
// SD card: write diagnostics buffer to /sdcard/boot_diagnostics.txt
// -----------------------------------------------------------------------------

static void post_write_to_sd(void)
{
    esp_err_t    ret;
    sdmmc_card_t *card = nullptr;

    esp_vfs_fat_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = true;   // auto-format if needed
    mount_config.max_files              = 10;
    mount_config.allocation_unit_size   = 16 * 1024;

    // SDMMC host configuration
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    // Slot configuration: 1-bit bus, GPIO matrix
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk   = POST_SD_CLK_GPIO;
    slot_config.cmd   = POST_SD_CMD_GPIO;
    slot_config.d0    = POST_SD_D0_GPIO;

    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
#ifdef SDMMC_SLOT_FLAG_USE_GPIO_MATRIX
    slot_config.flags |= SDMMC_SLOT_FLAG_USE_GPIO_MATRIX;
#endif

    post_log("SD: trying to mount at '%s'...", POST_MOUNT_POINT);

    ret = esp_vfs_fat_sdmmc_mount(
        POST_MOUNT_POINT,
        &host,
        &slot_config,
        &mount_config,
        &card
    );
    if (ret != ESP_OK) {
        post_log("SD: mount failed: %s", esp_err_to_name(ret));
        return;
    }

    post_log("SD: filesystem mounted, card name: %s", card->cid.name);

    // Write diagnostics buffer
    FILE *f = std::fopen(POST_DIAG_FILE, "w");
    if (!f) {
        post_log("SD: failed to open '%s' for write", POST_DIAG_FILE);
    } else {
        size_t written = std::fwrite(s_diag_buf, 1, s_diag_len, f);
        std::fclose(f);
        post_log("SD: wrote %u bytes to '%s'",
                 (unsigned)written, POST_DIAG_FILE);
    }

    esp_vfs_fat_sdcard_unmount(POST_MOUNT_POINT, card);
    post_log("SD: unmounted");
}

// -----------------------------------------------------------------------------
// Public entry point
// -----------------------------------------------------------------------------

extern "C" esp_err_t post_run(void)
{
    post_reset_buffer();
    post_dump_system_info();

    // Board-specific tests
    post_test_i2c();
    post_test_gpio();
    post_test_modem();

    // Extra: dump AXP2101 + GPIO state to console
    pmu_debug_dump_state();

    // Mirror diagnostics to SD card
    post_write_to_sd();

    post_log("=== POST completed ===");
    return ESP_OK;
}
