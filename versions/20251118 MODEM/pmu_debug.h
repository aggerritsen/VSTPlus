#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Dump AXP2101 registers (0x00–0xFF) en relevante GPIO-levels naar de log.
 *
 * Dit is puur diagnostisch:
 * - I2C0 op SDA=15 / SCL=7 (AXP2101)
 * - GEEN schrijfacties naar de PMU
 * - GPIO's alleen als input gelezen
 */
void pmu_debug_dump_state(void);

#ifdef __cplusplus
}
#endif
