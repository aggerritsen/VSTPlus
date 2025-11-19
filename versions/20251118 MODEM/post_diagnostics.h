#pragma once

#include "esp_err.h"

// Power-On Self Test (POST) entry point.
// Runs once at boot from app_main().
#ifdef __cplusplus
extern "C" {
#endif

esp_err_t post_run(void);

#ifdef __cplusplus
}
#endif
