// aw9523_traffic_light.h
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Init AW9523 + I2C-bus (I2C_NUM_1, SDA=13, SCL=14).
// Safe om meerdere keren aan te roepen.
esp_err_t aw9523_traffic_light_init(void);

// Blokkerende demo-loop: stoplicht sequentie (groen-geel-rood) elke 2 seconden.
// Draait in de caller-task; keert niet terug.
void aw9523_traffic_light_run(void);

#ifdef __cplusplus
}
#endif
