# ESP-IDF Configuration Guide

This document summarizes all configuration options required for your ESP32‑S3 test series (Hello World → PSRAM → SDMMC → Deep Sleep → Diagnostics).  
It explains what each setting does and where to find it inside **idf.py menuconfig**.

---

## 🔧 1. Set the Correct Target (CRITICAL)

Before configuring anything, run:

```
idf.py set-target esp32s3
```

Or in VS Code:  
**F1 → IDF: ESP Set Espressif Device Target → esp32s3**

---

## 🔥 2. Flash Configuration (Serial Flasher Config)

**Menu:**  
`Serial flasher config`

Set:

- **Flash size → 16 MB**
- **Flash frequency → 80 MHz**
- **SPI mode → DIO**

Resulting config flags:

```
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_ESPTOOLPY_FLASHMODE_DIO=y
```

---

## 🧠 3. PSRAM in Octal Mode

**Menu:**  
`Component config → ESP PSRAM`

Enable:

- Support for external SPI‑connected RAM
- Mode → **Octal Mode PSRAM**
- Set RAM clock speed → **80 MHz**
- Maximum malloc() internal size → **16384 bytes**

Relevant flags:

```
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384
```

---

## 🖥️ 4. Console Output via USB (No UART pins used)

**Menu:**  
`Component config → ESP‑STDIO → Channel for console output`

Set:

- **USB Serial/JTAG Controller**

Flags:

```
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
CONFIG_ESP_CONSOLE_UART_DEFAULT=n
```

This enables:
- Console over USB (UART pins 43/44 freed)
- JTAG debugging
- No serial adapter required

---

## 📜 5. Logging Configuration

**Menu:**  
`Component config → Log`

Set:

- Default log verbosity → **Info**

```
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
CONFIG_BOOTLOADER_LOG_LEVEL_INFO=y
```

---

## 📦 6. Custom Partition Table

**Menu:**  
`Partition Table → Partition Table → Custom partition table CSV`

Set:

- Custom partition file name, e.g. `partitions.csv`

```
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
```

Your CSV must exist in the project folder.

---

## 💾 7. SD Card (SDMMC) Configuration

### Hardware Lines Used (T-SIM7080G-S3)
- **CLK → GPIO 38**
- **CMD → GPIO 39**
- **D0  → GPIO 40**
- 1‑bit mode only.

### Software Settings

`slot_config.width = 1;`  
`slot_config.clk = GPIO_NUM_38;`  
`slot_config.cmd = GPIO_NUM_39;`  
`slot_config.d0  = GPIO_NUM_40;`

FATFS recommended options:

```
CONFIG_FATFS_VOLUME_COUNT=2
CONFIG_FATFS_SECTOR_4096=y
CONFIG_FATFS_ALLOC_PREFER_EXTRAM=y
```

---

## 🛌 8. Deep Sleep Config

**Menu:**  
`Component config → ESP System Settings → Sleep`

Recommended:

```
CONFIG_ESP_SLEEP_DEEP_SLEEP_WAKEUP_DELAY=2000
CONFIG_PM_SLP_IRAM_OPT=y
CONFIG_PM_SLEEP_FUNC_IN_IRAM=y
```

---

## ⚡ 9. CPU Frequency

**Menu:**  
`Component config → ESP System Settings → Default CPU freq`

Set to:

```
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_160=y
```

---

## 🧪 10. Complete Checklist Before Running Diagnostics

| Feature | Required Setting |
|--------|------------------|
| Target | esp32s3 |
| Flash | 16MB / 80MHz / DIO |
| PSRAM | Octal Mode @ 80MHz |
| Console | USB Serial/JTAG |
| Logging | Info |
| Partitions | Custom CSV |
| SDMMC | 1‑bit, pins 38/39/40 |
| Deep sleep | Timer + IRAM settings |
| CPU | 160 MHz |
| PSRAM malloc | Enabled |

This set is **stable, proven, and validated** across all example tests.

---

