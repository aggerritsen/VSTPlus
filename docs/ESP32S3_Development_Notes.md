# ESP32-S3 Development Notes  
## Lessons Learned from hello_world, PSRAM, and SDMMC on the LilyGO T-SIM7080G-S3

This document summarizes all important discoveries, pitfalls, configuration requirements, and working solutions encountered while bringing up the ESP32-S3 (LILYGO T-SIM7080G-S3 board) using ESP-IDF.

It covers:

1. Environment setup lessons
2. hello_world basics
3. PSRAM / XIP from PSRAM example
4. SDMMC (SD card on T-SIM7080G-S3)
5. VSCode target misconfiguration & how to fix it
6. SDKConfig options you must know about

## 1. Environment & Build System Lessons

### ESP-IDF Target Mismatch  
A recurring issue occurred where:

- VSCode IDF Terminal showed: esp32s3
- VSCode PowerShell Terminal showed: esp32
- User PowerShell outside VSCode showed: esp32s3

This caused build failures such as:

```
Project sdkconfig was generated for target esp32s3,
but environment variable IDF_TARGET is set to esp32.
```

### Solution — The F1 Command
The correct way to fix this is:

**VSCode → F1 → “IDF: Set Espressif Device Target” → esp32s3**

This updates settings.json, regenerates sdkconfig, and fixes the environment.

## 2. hello_world Example — Lessons Learned

### Build + Flash + Monitor basics  
If hello_world fails, nothing else will work.

### Serial port issues  
If you see:

```
No serial ports found
Could not open COM3
```

then:

- A monitor session is still open (Ctrl+])
- Windows may keep COM port busy
- Device may need power cycle

## 3. PSRAM / XIP-from-PSRAM — Lessons Learned

### PSRAM Works on T-SIM7080G-S3  
```
Found 8MB PSRAM device
Speed: 80MHz
Instructions mapped to SPIRAM
```

### Timer Error in Example  
Original example crashed with:

```
ESP_ERR_INVALID_STATE
```

Fix: ensure timers are not started/deleted too quickly.

### Verified XIP performance  
```
callback(in PSRAM) response time: 0 us
callback(in IRAM) response time: 0 us
```

## 4. SDMMC on the LilyGO T-SIM7080G-S3 — Lessons Learned

### SDMMC Slot Used: SLOT_1 (HS2)

Pins:

| Signal | GPIO |
|--------|------|
| SD_CLK | 38 |
| SD_CMD | 39 |
| SD_D0  | 40 |

### Must Use 1-bit Bus  
```c
slot_config.width = 1;
```

### Working Pin Mapping  
```c
slot_config.clk = GPIO_NUM_38;
slot_config.cmd = GPIO_NUM_39;
slot_config.d0  = GPIO_NUM_40;
```

### Must Enable GPIO Matrix  
```c
slot_config.flags |= SDMMC_SLOT_FLAG_USE_GPIO_MATRIX;
```

### Working Result  
```
Filesystem mounted
Read from file: 'Hello <CARDNAME>!'
Card unmounted
```

## 5. Critical sdkconfig Parameters

### Target Selection
```
CONFIG_IDF_TARGET="esp32s3"
```

### PSRAM
```
CONFIG_SPIRAM
CONFIG_SPIRAM_BOOT_INIT
CONFIG_SPIRAM_FETCH_INSTRUCTIONS
CONFIG_SPIRAM_RODATA
CONFIG_SPIRAM_SPEED_80M
```

### SDMMC
```
CONFIG_EXAMPLE_PIN_CLK=38
CONFIG_EXAMPLE_PIN_CMD=39
CONFIG_EXAMPLE_PIN_D0=40
CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_1
CONFIG_SDMMC_USE_GPIO_MATRIX
```

## 6. Final Working Hardware Map (T-SIM7080G-S3)

| Function | GPIO |
|----------|------|
| UART TX | 44 |
| UART RX | 43 |
| SD_CLK | 38 |
| SD_CMD | 39 |
| SD_D0 | 40 |
| PSRAM | handled internally |

## 7. Summary

You now have:

- Working USB serial  
- Working PSRAM (XIP)  
- Working SDMMC  
- Correct VSCode target configuration  
- Stable sdkconfig baseline
