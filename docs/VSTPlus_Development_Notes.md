# VSTPlus Development Notes

## Power-On Self Test (POST) Diagnostics

### Overview
The POST system runs automatically at boot and performs hardware verification before the application continues. It logs diagnostics over serial and optionally writes a report to the SD card as `/sdcard/boot_diagnostics.txt`.

### Current POST Features
- Timestamp (RTC-based approximate boot time)
- Chip information (model, cores, revision)
- IDF version
- Flash size detection
- PSRAM detection & heap metrics
- Internal heap metrics
- I²C bus scan
- Modem UART “alive?” check (AT test)
- SD card mount + write test
- Serial + SD logging via internal diagnostics buffer

---

## POST Sequence Flow

1. Reset diagnostics buffer  
2. Log core system data  
3. Log memory and flash characteristics  
4. Run board-level checks:  
   - `post_test_i2c()`  
   - `post_test_gpio()` *(planned)*  
   - `post_test_modem()`  
5. Attempt SD card mount  
6. Write diagnostic file  
7. Clean unmount  
8. Continue to main application

---

## I²C Bus Testing

Pins chosen from LilyGO T‑SIM7080G‑S3 header:

- SDA → **GPIO 8**
- SCL → **GPIO 9**

Scan reports connected devices or “no devices found”.

---

## Modem UART Test

Using hardware UART1 mapped to pins:

- TX → **GPIO 17**
- RX → **GPIO 18**
- Baud → **115200**

Basic test: send “AT”, expect “OK”.  
No network connection required.

---

## SD Card Mounting

Configured for SDMMC slot 1 with custom GPIO matrix mapping:

- CLK → GPIO 38  
- CMD → GPIO 39  
- D0  → GPIO 40  

On success, POST writes the diagnostics buffer to:

```
/sdcard/boot_diagnostics.txt
```

---

## Known Warnings & Behaviour

- `i2c: This driver is an old driver`  
  → ESP-IDF 6 prefers `i2c_master` API (future migration planned).

- SD mounting prints:  
  `input line delay not supported, fallback to 0 delay`  
  → Safe to ignore for 1‑bit SD bus on this board.

- Modem test may fail if modem is not powered or not ready.

---

## Planned Enhancements

- GPIO state reporting for all key header pins  
- Modem power detection (PWRKEY/PWR pin logic check)  
- SPI bus integrity check  
- Sensor detection (BME280, load cells, etc.)  
- Structured JSON diagnostics optional output  
- SD card benchmark

---

## Included Image

```md
![LilyGO T-SIM7080G-S3 Pinout](./Lilygo%20T-SIM7080G-S3%20PINOUT.jpg)
```

---

## Summary

POST system is stable and writes reliable hardware diagnostics both to serial and SD card. Hardware mapping is now consistent with the actual T‑SIM7080G‑S3 layout and modem/I²C integration is functioning as foundational infrastructure for the full application.
