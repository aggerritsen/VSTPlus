#pragma once

#include "esp_err.h"

// Simple C++ declaration of the POST entry function.
// (Currently used only from C++; if you ever need it from C,
// we can later add `extern "C"` on both declaration & definition.)
esp_err_t post_run(void);
