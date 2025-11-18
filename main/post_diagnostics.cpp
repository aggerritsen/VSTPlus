/* main/post_diagnostics.c
* 
* Simple Power-On Self Test (POST) module
 *
 * - Logs chip / flash / PSRAM / heap info to the serial console
 * - Attempts to mount SD card on SDMMC host slot 1 (1-bit bus, GPIO 38/39/40)
 * - Writes diagnostics to /sdcard/boot_diagnostics.txt on success
 */

#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <ctime>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"

#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_timer.h"


#include "post_diagnostics.h"

static void post_log(const char* fmt, ...);

static const char* TAG = "POST";

#define POST_MOUNT_POINT   "/sdcard"
#define POST_DIAG_FILE     POST_MOUNT_POINT "/boot_diagnostics.txt"

// Adjust if you ever change wiring
#define POST_SD_CLK_GPIO   GPIO_NUM_38
#define POST_SD_CMD_GPIO   GPIO_NUM_39
#define POST_SD_D0_GPIO    GPIO_NUM_40

// ---- I2C probe config (based on your header) ----
//#define POST_I2C_PORT      I2C_NUM_0
//#define POST_I2C_SDA_GPIO  GPIO_NUM_8    // P1.6 GPIO08
//#define POST_I2C_SCL_GPIO  GPIO_NUM_9    // P1.9 GPIO09
//#define POST_I2C_FREQ_HZ   100000

// ---- I2C probe config (NEW, camera-safe) ----
#define POST_I2C_PORT      I2C_NUM_1
#define POST_I2C_SDA_GPIO  GPIO_NUM_3    // P1.7
#define POST_I2C_SCL_GPIO  GPIO_NUM_43   // P2.5
#define POST_I2C_FREQ_HZ   100000

// ---- Modem UART config (from your pin table) ----
#define POST_MODEM_UART_NUM   UART_NUM_1
#define POST_MODEM_TX_GPIO    GPIO_NUM_17   // P1.4 U1TXD → to modem RX
#define POST_MODEM_RX_GPIO    GPIO_NUM_18   // P1.5 U1RXD → from modem TX
#define POST_MODEM_BAUD       115200
#define POST_MODEM_TIMEOUT_US 1000000


// Internal buffer to mirror what we print to serial
static char   s_diag_buf[2048];
static size_t s_diag_len = 0;

typedef struct {
    gpio_num_t pin;
    const char* name;
    int expected_level;   // -1 if "no expectation"
} post_gpio_test_t;

// Fill this with pins that matter on *your* board
static const post_gpio_test_t s_gpio_tests[] = {
    // { GPIO_NUM_4,  "MODEM_PWRKEY",  -1 },
    // { GPIO_NUM_5,  "MODEM_STATUS",  -1 },
    // Add / adjust as you like:
};

static void post_test_modem(void)
{
    post_log("MODEM: checking AT response on UART%d", POST_MODEM_UART_NUM);

    uart_config_t cfg = {};
    cfg.baud_rate = POST_MODEM_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_APB;

    esp_err_t err;
    err = uart_param_config(POST_MODEM_UART_NUM, &cfg);
    if (err != ESP_OK) {
        post_log("MODEM: uart_param_config failed: %s", esp_err_to_name(err));
        return;
    }

    err = uart_set_pin(POST_MODEM_UART_NUM,
                       POST_MODEM_TX_GPIO,
                       POST_MODEM_RX_GPIO,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        post_log("MODEM: uart_set_pin failed: %s", esp_err_to_name(err));
        return;
    }

    err = uart_driver_install(POST_MODEM_UART_NUM, 1024, 0, 0, nullptr, 0);
    if (err != ESP_OK) {
        post_log("MODEM: uart_driver_install failed: %s", esp_err_to_name(err));
        return;
    }

    const char* cmd = "AT\r\n";
    uart_flush_input(POST_MODEM_UART_NUM);
    uart_write_bytes(POST_MODEM_UART_NUM, cmd, strlen(cmd));

    uint8_t buf[128];
    int total = 0;
    bool ok = false;

    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < POST_MODEM_TIMEOUT_US && total < (int)sizeof(buf) - 1) {
        int n = uart_read_bytes(POST_MODEM_UART_NUM,
                                buf + total,
                                sizeof(buf) - 1 - total,
                                20 / portTICK_PERIOD_MS);
        if (n > 0) {
            total += n;
            buf[total] = '\0';

            if (strstr((char*)buf, "OK")) {
                ok = true;
                break;
            }
        }
    }

    if (ok) {
        post_log("MODEM: AT -> OK (modem alive)");
    } else {
        post_log("MODEM: no 'OK' received (check power / UART pins / baud)");
    }

    uart_driver_delete(POST_MODEM_UART_NUM);
}

static void post_test_i2c(void)
{
    post_log("I2C: scanning bus on SDA=%d, SCL=%d",
             POST_I2C_SDA_GPIO, POST_I2C_SCL_GPIO);

    i2c_master_bus_handle_t bus = nullptr;

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.i2c_port = I2C_NUM_0;                 // controller 0
    bus_cfg.sda_io_num = POST_I2C_SDA_GPIO;
    bus_cfg.scl_io_num = POST_I2C_SCL_GPIO;
    bus_cfg.glitch_ignore_cnt = 7;                // small default, not too critical
    bus_cfg.flags.enable_internal_pullup = true;  // use internal pull-ups

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        post_log("I2C: i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return;
    }

    bool any = false;

    // Standard 7-bit I2C address space 0x08..0x77
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        err = i2c_master_probe(bus, addr, 50 /* timeout ms */);
        if (err == ESP_OK) {
            post_log("I2C: found device at 0x%02X", addr);
            any = true;
        }
        // We deliberately ignore ESP_ERR_TIMEOUT / ESP_FAIL details here
    }

    if (!any) {
        post_log("I2C: no devices found");
    }

    // Clean up the bus
    err = i2c_del_master_bus(bus);
    if (err != ESP_OK) {
        post_log("I2C: i2c_del_master_bus failed: %s", esp_err_to_name(err));
    }
}


static void post_test_gpio(void)
{
    if (sizeof(s_gpio_tests) == 0) {
        // Nothing configured
        return;
    }

    post_log("GPIO: basic input state check");

    for (size_t i = 0; i < (sizeof(s_gpio_tests) / sizeof(s_gpio_tests[0])); ++i) {
        const auto& t = s_gpio_tests[i];

        if (t.pin < 0) {
            continue;
        }

        gpio_reset_pin(t.pin);
        gpio_set_direction(t.pin, GPIO_MODE_INPUT);

        int level = gpio_get_level(t.pin);
        if (t.expected_level == 0 || t.expected_level == 1) {
            const char* ok = (level == t.expected_level) ? "OK" : "MISMATCH";
            post_log("GPIO %-14s (%2d): level=%d (expected %d) -> %s",
                     t.name, (int)t.pin, level, t.expected_level, ok);
        } else {
            post_log("GPIO %-14s (%2d): level=%d",
                     t.name, (int)t.pin, level);
        }
    }
}

static void post_log(const char* fmt, ...)
{
    char line[256];

    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    // Log to serial
    ESP_LOGI(TAG, "%s", line);

    // Append to diagnostics buffer (with newline)
    size_t len = strnlen(line, sizeof(line));
    if (s_diag_len + len + 2 < sizeof(s_diag_buf)) {
        memcpy(s_diag_buf + s_diag_len, line, len);
        s_diag_len += len;
        s_diag_buf[s_diag_len++] = '\n';
        s_diag_buf[s_diag_len] = '\0';
    }
}

static void post_reset_buffer(void)
{
    s_diag_len = 0;
    s_diag_buf[0] = '\0';
}

/**
 * @brief Try to mount SD card and write diagnostics to POST_DIAG_FILE.
 *
 * Errors are logged but do not propagate as fatal.
 */
static void post_write_to_sd(void)
{
    
    esp_err_t ret;
    sdmmc_card_t* card = nullptr;

    esp_vfs_fat_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed   = true;          // Set to true to auto-format SD if mount fails
    mount_config.max_files                = 10;            // Safe for camera + AI + logs
    mount_config.allocation_unit_size     = 16 * 1024;     // FAT allocation unit (16 KB recommended for SD wear & performance)
    mount_config.disk_status_check_enable = false;         // Disable periodic card polling (saves power)
    mount_config.use_one_fat              = false;         // Use both FATs (standard FAT filesystem behavior)

    // Host configuration: SDMMC slot 1, modest speed
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_1;
    host.max_freq_khz = 20000;    // 20 MHz = default, conservative & stable

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

    ret = esp_vfs_fat_sdmmc_mount(POST_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        post_log("SD: mount failed (%s), skipping SD diagnostics", esp_err_to_name(ret));
        return;
    }

    post_log("SD: filesystem mounted, card name: %s", card->cid.name);

    // Write diagnostics file
    FILE* f = std::fopen(POST_DIAG_FILE, "w");
    if (!f) {
        post_log("SD: failed to open '%s' for writing", POST_DIAG_FILE);
        esp_vfs_fat_sdcard_unmount(POST_MOUNT_POINT, card);
        post_log("SD: unmounted");
        return;
    }

    size_t written = std::fwrite(s_diag_buf, 1, s_diag_len, f);
    std::fclose(f);

    post_log("SD: wrote %u bytes to '%s'", static_cast<unsigned>(written), POST_DIAG_FILE);

    // Unmount again (for a real app you might keep it mounted globally instead)
    esp_vfs_fat_sdcard_unmount(POST_MOUNT_POINT, card);
    post_log("SD: unmounted");
}

/**
 * @brief Run the complete POST sequence.
 */
esp_err_t post_run(void)
{
    post_reset_buffer();

    post_log("=== Power-On Self Test (POST) ===");

    // Timestamp
    std::time_t now = std::time(nullptr);
    char time_str[64] = {};
    std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    post_log("Boot time (approx): %s", time_str);

    // Chip info
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    post_log("Chip model: %s, %d core(s), revision %d",
             CONFIG_IDF_TARGET, chip_info.cores, chip_info.revision);

    post_log("Features: WiFi%s%s%s",
             (chip_info.features & CHIP_FEATURE_BT)      ? " / BT"      : "",
             (chip_info.features & CHIP_FEATURE_BLE)     ? " / BLE"     : "",
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? " / EMB_FLASH" : "");

    post_log("IDF version: %s", esp_get_idf_version());

    // Flash size
    uint32_t flash_size = 0;
    if (esp_flash_get_size(nullptr, &flash_size) == ESP_OK) {
        post_log("Flash size: %u bytes (%.2f MB)", flash_size, flash_size / (1024.0f * 1024.0f));
    } else {
        post_log("Flash size: unknown (esp_flash_get_size failed)");
    }

    // PSRAM / SPIRAM info
    size_t psram_size = esp_psram_get_size();
    if (psram_size > 0) {
        size_t psram_free    = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);

        post_log("PSRAM: present, %u bytes (%.2f MB)",
                 static_cast<unsigned>(psram_size),
                 psram_size / (1024.0f * 1024.0f));
        post_log("PSRAM heap: free=%u, largest_block=%u",
                 static_cast<unsigned>(psram_free),
                 static_cast<unsigned>(psram_largest));
    } else {
        post_log("PSRAM: not detected or not enabled");
    }

    // Internal heap info
    size_t heap_free    = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    size_t heap_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    post_log("Internal heap: free=%u, largest_block=%u",
             static_cast<unsigned>(heap_free),
             static_cast<unsigned>(heap_largest));


    // Extra board-specific tests
    post_test_i2c();
    post_test_gpio();
    post_test_modem();

    // Finally, try to mirror diagnostics to SD card
    post_write_to_sd();

    post_log("=== POST completed ===");

    return ESP_OK;
}
