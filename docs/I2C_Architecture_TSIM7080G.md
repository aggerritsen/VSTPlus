# I²C Architecture on the T-SIM7080G-S3

## Overview

| I²C Bus | ESP-IDF ID | Purpose | Connected Devices | GPIO Pins |
|--------|------------|----------|-------------------|-----------|
| **Primary Sensor Bus** | `I2C_NUM_0` | Environment sensors (external) | BME280, etc. | **SDA = GPIO 3**, **SCL = GPIO 43** |
| **PMU Bus** | `I2C_NUM_1` | Power management control | **AXP2101 PMU** | **SDA = GPIO 15**, **SCL = GPIO 7** |

---

## 1. I²C Bus 0 — Sensor Bus (I2C_NUM_0)

Used for external I²C sensors connected to the 4-pin header.

### Pin Assignments

| Function | GPIO |
|----------|------|
| SDA      | **GPIO 3** |
| SCL      | **GPIO 43** |

### Devices

- BME280 / BMP280
- External temperature/humidity/pressure sensors

### Initialization example

```c
i2c_param_config(I2C_NUM_0, ... SDA=3, SCL=43);
i2c_driver_install(I2C_NUM_0, ...);
```

---

## 2. I²C Bus 1 — PMU Control Bus (I2C_NUM_1)

Used exclusively for the **AXP2101 PMU**, which powers and controls the SIM7080G modem.

### Pin Assignments

| Function | GPIO |
|----------|------|
| SDA      | **GPIO 15** |
| SCL      | **GPIO 7**  |

### Responsibilities

- Enabling DCDC3 power rail (modem core power)
- Enabling BLDO2 rail (I/O and digital supply)
- Power gating and power sequencing
- Modem PWRKEY control inside the PMU
- Charger and USB/battery power path logic

### Initialization example

```c
i2c_param_config(I2C_NUM_1, ... SDA=15, SCL=7);
i2c_driver_install(I2C_NUM_1, ...);

axp2101_write(0x84, dcdc3_code);  // Set DCDC3 voltage
axp2101_write(0x97, bldo2_code);  // Set BLDO2 voltage
axp2101_write(0x80, onoff);       // Enable DCDC/LDO rails
```

---

## Why Two Buses Exist

- Prevents sensor bus hang from blocking modem power
- Ensures PMU commands are clean during boot
- Matches hardware routing on the PCB
- Required for reliable SIM7080G startup

---

## Summary Diagram

```
           +---------------------+
           |     ESP32-S3        |
           |                     |
           |   I2C_NUM_0         |
           |   SDA = GPIO 3  ----+----> External Sensors (BME280...)
           |   SCL = GPIO 43 ----+
           |
           |   I2C_NUM_1         |
           |   SDA = GPIO 15 ----+----> AXP2101 PMU ----> DCDC3 / BLDO2 ----> SIM7080G Modem
           |   SCL = GPIO 7  ----+
           |
           +---------------------+
```

---

