#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "post_diagnostics.h"
#include "aw9523_traffic_light.h"

static const char* TAG = "VSTPlus";

extern "C" void app_main(void)
{
    // Run Power-On Self Test once at boot
    esp_err_t err = post_run();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "POST failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Hello from VSTPlus on ESP32-S3 (C / ESP-IDF)!");

    //while (true) {
    //    ESP_LOGI(TAG, "Tick...");
    //    vTaskDelay(pdMS_TO_TICKS(1000));
    //}

    // In plaats van de Tick-loop:
    aw9523_traffic_light_run();   // blokkeert, eigen while(true) binnenin
}
