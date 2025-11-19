// main/pmu_debug.h
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Gedeelde AXP2101 helpers
esp_err_t axp2101_read_reg(uint8_t reg, uint8_t *value);
esp_err_t axp2101_write_reg(uint8_t reg, uint8_t value);

// Volledige PMU + GPIO dump
void pmu_debug_dump(void);

#ifdef __cplusplus
}
#endif
