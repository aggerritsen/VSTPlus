/*
 * main/post_diagnostics.cpp
 *
 * Power-On Self Test (POST) module for VSTPlus on T-SIM7080G-S3
 *
 * - Logs chip / flash / PSRAM / heap info to the serial console
 * - Scans I2C bus for devices
 * - Brings up the SIM7080 modem power (via AXP2101 PMU) and checks AT response
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
#include "driver/i2c_master.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pmu_debug.h"           // shared AXP2101 access + debug dump

static const char *TAG = "POST";
#define TAG_POST TAG

// -----------------------------------------------------------------------------
// Board / wiring configuration
// -----------------------------------------------------------------------------

// ----- PMU (AXP2101) on T-SIM7080G-S3 -----
// PMU I2C is on I2C_NUM_0, SDA=15, SCL=7, INT=6

#define POST_PMU_I2C_PORT   I2C_NUM_0
#define POST_PMU_SDA_GPIO   GPIO_NUM_15   // PMU SDA
#define POST_PMU_SCL_GPIO   GPIO_NUM_7    // PMU SCL
#define POST_PMU_I2C_FREQ   400000

// AXP2101 TWSI 7-bit address: 0b0110100 = 0x34
#define AXP2101_I2C_ADDR    0x34

// Registers we use
#define AXP2101_REG_LDO_ONOFF0   0x90   // LDO ON/OFF control 0, bit5 = BLDO2 enable
#define AXP2101_REG_BLDO2_VOLT   0x97   // BLDO2 voltage setting

// SD card configuration: SDMMC host slot 1, 1-bit bus on 38/39/40
#define POST_MOUNT_POINT   "/sdcard"
#define POST_DIAG_FILE     POST_MOUNT_POINT "/boot_diagnostics.txt"

#define POST_SD_CLK_GPIO   GPIO_NUM_38
#define POST_SD_CMD_GPIO   GPIO_NUM_39
#define POST_SD_D0_GPIO    GPIO_NUM_40

// I2C probe configuration (camera-safe bus for external sensors, new driver)
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

// -----------------------------------------------------------------------------
// Small helper for GPIO tests (optional)
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
// PMU helpers (AXP2101 via new driver, shared with pmu_debug)
// -----------------------------------------------------------------------------

/**
 * Enable BLDO2 at 3.3V to power the SIM7080G modem.
 * Uses axp2101_*() from pmu_debug.cpp (new i2c_master driver).
 */
static esp_err_t post_pmu_enable_modem_rails(void)
{
    ESP_LOGI(TAG_POST, "PMU: configuring AXP2101 on SDA=%d, SCL=%d for modem power",
             POST_PMU_SDA_GPIO, POST_PMU_SCL_GPIO);

    esp_err_t err;

    // 1) Set BLDO2 voltage to ~3.3V
    const uint8_t bldo2_code_3v3 = 0x1C; // 3.3V => code 28 => 0b11100
    err = axp2101_write_reg(AXP2101_REG_BLDO2_VOLT, bldo2_code_3v3);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_POST, "PMU: write BLDO2 voltage (0x%02X) failed", AXP2101_REG_BLDO2_VOLT);
        return err;
    }

    // 2) Enable BLDO2 in LDO ON/OFF control 0 (bit5 = 1)
    uint8_t ldo_onoff0 = 0;
    err = axp2101_read_reg(AXP2101_REG_LDO_ONOFF0, &ldo_onoff0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_POST, "PMU: read LDO_ONOFF0 (0x%02X) failed", AXP2101_REG_LDO_ONOFF0);
        return err;
    }

    ldo_onoff0 |= (1 << 5);  // BLDO2 enable
    err = axp2101_write_reg(AXP2101_REG_LDO_ONOFF0, ldo_onoff0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_POST, "PMU: write LDO_ONOFF0 (0x%02X) failed", AXP2101_REG_LDO_ONOFF0);
        return err;
    }

    ESP_LOGI(TAG_POST, "PMU: BLDO2 3.3V enabled to power modem (DCDC3 left as configured by bootloader)");
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Logging helpers
// -----------------------------------------------------------------------------

static void post_log(const char *fmt, ...)
{
    va_list ap;

    char line[256];
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    if (n < 0) {
        return;
    }

    line[sizeof(line) - 1] = '\0';

    ESP_LOGI(TAG, "%s", line);

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

static void post_reset_buffer(void)
{
    s_diag_len    = 0;
    s_diag_buf[0] = '\0';
}

// -----------------------------------------------------------------------------
// System info dump
// -----------------------------------------------------------------------------

static void post_dump_system_info(void)
{
    post_log("=== Power-On Self Test (POST) ===");

    std::time_t now = std::time(nullptr);
    std::tm tm_info{};
    if (localtime_r(&now, &tm_info) != nullptr) {
        char time_buf[64];
        std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &tm_info);
        post_log("Boot time (approx): %s", time_buf);
    } else {
        post_log("Boot time (approx): <unknown>");
    }

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

    post_log("Features: %s%s%s%s",
             (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "",
             (chip_info.features & CHIP_FEATURE_BLE)       ? "/ BLE " : "",
             (chip_info.features & CHIP_FEATURE_BT)        ? "/ BT "  : "",
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "/ Embedded flash" : "");

    uint32_t flash_size = 0;
    if (esp_flash_get_size(nullptr, &flash_size) == ESP_OK) {
        post_log("Flash size: %u bytes (%.2f MB)",
                 (unsigned)flash_size,
                 (double)flash_size / (1024.0 * 1024.0));
    } else {
        post_log("Flash size: <unknown>");
    }

    size_t psram_size = esp_psram_get_size();
    if (psram_size > 0) {
        post_log("PSRAM: present, %u bytes (%.2f MB)",
                 (unsigned)psram_size,
                 (double)psram_size / (1024.0 * 1024.0));
    } else {
        post_log("PSRAM: not present");
    }

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

static void post_test_i2c(void)
{
    post_log("I2C: scanning bus on SDA=%d, SCL=%d (new driver, full scan)",
             (int)POST_I2C_SDA_GPIO, (int)POST_I2C_SCL_GPIO);

    i2c_master_bus_handle_t bus = nullptr;

    // Volledig init met {} om alle velden, incl. flags/allow_pd, op 0 te zetten
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port          = POST_I2C_PORT;
    bus_cfg.sda_io_num        = POST_I2C_SDA_GPIO;
    bus_cfg.scl_io_num        = POST_I2C_SCL_GPIO;
    bus_cfg.clk_source        = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.intr_priority     = 0;
    bus_cfg.trans_queue_depth = 0;
    // flags is een sub-struct; we zetten alleen wat we nodig hebben
    bus_cfg.flags.enable_internal_pullup = true;
    bus_cfg.flags.allow_pd               = false;  // expliciet, voor de vorm

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        post_log("I2C: i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return;
    }

    bool any_found = false;

    // Volledige 7-bit adresruimte scannen
    for (uint8_t addr = 1; addr < 0x7F; ++addr) {
        err = i2c_master_probe(bus, addr, 20 /* timeout ms */);
        if (err == ESP_OK) {
            post_log("I2C: found device at 0x%02X", addr);
            any_found = true;
        }
        // Andere fouten = geen device → negeren
    }

    if (!any_found) {
        post_log("I2C: no devices found");
    }

    i2c_del_master_bus(bus);
}


// -----------------------------------------------------------------------------
// GPIO checks (optional)
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
// Modem test helpers
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
        ESP_LOGI(TAG_POST, "%s: no 'OK' yet, retry=%d", tag_prefix, attempt);
    } else {
        ESP_LOGI(TAG_POST, "%s: got 'OK' from modem", tag_prefix);
    }

    return ok;
}

static void post_modem_pulse_pwrkey(void)
{
    ESP_LOGI(TAG_POST, "MODEM: pulsing PWRKEY (GPIO%d) to start modem", BOARD_MODEM_PWR_PIN);

    gpio_set_level(BOARD_MODEM_PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(800));
    gpio_set_level(BOARD_MODEM_PWR_PIN, 1);

    vTaskDelay(pdMS_TO_TICKS(1500));
}

static void post_test_modem(void)
{
    ESP_LOGI(TAG_POST,
             "MODEM: checking AT response on UART%d (TX=GPIO%d, RX=GPIO%d)",
             POST_MODEM_UART_NUM,
             POST_MODEM_TX_GPIO,
             POST_MODEM_RX_GPIO);

    esp_err_t err = post_pmu_enable_modem_rails();
    if (err != ESP_OK) {
        ESP_LOGW(TAG_POST, "MODEM: PMU configuration failed, modem may stay off");
    }

    gpio_config_t io = {};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pin_bit_mask = (1ULL << BOARD_MODEM_PWR_PIN) | (1ULL << BOARD_MODEM_DTR_PIN);
    gpio_config(&io);

    gpio_set_level(BOARD_MODEM_PWR_PIN, 1);
    gpio_set_level(BOARD_MODEM_DTR_PIN, 1);

    gpio_config_t ri_cfg = {};
    ri_cfg.intr_type = GPIO_INTR_DISABLE;
    ri_cfg.mode = GPIO_MODE_INPUT;
    ri_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    ri_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ri_cfg.pin_bit_mask = (1ULL << BOARD_MODEM_RI_PIN);
    gpio_config(&ri_cfg);

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

    bool ok = false;
    for (int i = 0; i < POST_MODEM_MAX_RETRY && !ok; ++i) {
        ok = post_modem_send_at_and_wait_ok("MODEM", i);
        if (!ok) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

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
        ESP_LOGW(TAG_POST, "MODEM: no response after PWRKEY pulse");
    }

    uart_driver_delete(POST_MODEM_UART_NUM);
}

// -----------------------------------------------------------------------------
// SD card
// -----------------------------------------------------------------------------

static void post_write_to_sd(void)
{
    esp_err_t    ret;
    sdmmc_card_t *card = nullptr;

    esp_vfs_fat_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = true;
    mount_config.max_files              = 10;
    mount_config.allocation_unit_size   = 16 * 1024;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

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

    post_test_i2c();
    post_test_gpio();
    post_test_modem();

    // Optional: PMU / GPIO register dump after modem bring-up
    pmu_debug_dump();

    post_write_to_sd();

    post_log("=== POST completed ===");
    return ESP_OK;
}
