# I²C Architecture on the T-SIM7080G-S3

## Overview

| I²C Bus | ESP-IDF ID | Purpose | Connected Devices | GPIO Pins |
|--------|------------|----------|-------------------|-----------|
| **PMU Bus** | `I2C_NUM_0` | Power management control | **AXP2101 PMU** | **SDA = GPIO 15**, **SCL = GPIO 7** |
| **Sensor Bus** | `I2C_NUM_1` | External environment sensors | BME280 / BMP280 / etc. | **SDA = GPIO 3**, **SCL = GPIO 43** |

---

## 1. I²C Bus 0 — PMU Control Bus (`I2C_NUM_0`)

This bus is dedicated to the **AXP2101 PMU**, which manages:

- DCDC3 — modem core rail  
- BLDO2 — modem I/O rail  
- Power gating and ON/OFF logic  
- Charging and power path switching  
- Internal PWRKEY control  

### Pin Assignments

| Function | GPIO |
|----------|------|
| SDA      | **GPIO 15** |
| SCL      | **GPIO 7** |

### Initialization Example

```c
i2c_master_bus_config_t pmu_bus_cfg = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_15,
    .scl_io_num = GPIO_NUM_7,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .flags = { .enable_internal_pullup = true },
};

i2c_master_bus_handle_t pmu_bus = NULL;
i2c_new_master_bus(&pmu_bus_cfg, &pmu_bus);
```

### Devices on This Bus

- **AXP2101 PMU (0x34)** — *the only device on I2C_NUM_0.*

---

## 2. I²C Bus 1 — External Sensor Bus (`I2C_NUM_1`)

This bus connects to all **external I²C sensors**, including:

- BME280  
- BMP280  
- Additional environment monitoring devices  

### Pin Assignments

| Function | GPIO |
|----------|------|
| SDA      | **GPIO 3** |
| SCL      | **GPIO 43** |

### Initialization Example

```c
i2c_master_bus_config_t sensor_bus_cfg = {
    .i2c_port = I2C_NUM_1,
    .sda_io_num = GPIO_NUM_3,
    .scl_io_num = GPIO_NUM_43,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .flags = { .enable_internal_pullup = true },
};

i2c_master_bus_handle_t sensor_bus = NULL;
i2c_new_master_bus(&sensor_bus_cfg, &sensor_bus);
```

---

## Why Two Independent I²C Buses?

### ✔ PMU bus must never hang  
If sensors freeze or the cable gets wet/corroded (common in outdoor hives),  
**I2C_NUM_1 may lock up.**

But **I2C_NUM_0 must remain reliable**, because:

- The PMU powers the modem  
- The PMU controls charging  
- The PMU controls system rails  
- The PMU may need to toggle PWRKEY  

### ✔ Clean separation  
PMU = internal power rail logic  
Sensors = external field wiring

### ✔ Safe boot behavior  
The modem power sequence would fail if PMU I²C were shared or overloaded.

---

## Summary Diagram (Corrected)

```
           +---------------------+
           |     ESP32-S3        |
           |                     |
           |   I2C_NUM_1         |
           |   SDA = GPIO 3  ----+----> External Sensors (BME280...)
           |   SCL = GPIO 43 ----+
           |
           |   I2C_NUM_0         |
           |   SDA = GPIO 15 ----+----> AXP2101 PMU ----> DCDC3 / BLDO2 ----> SIM7080G Modem
           |   SCL = GPIO 7  ----+
           |
           +---------------------+
```

---

## Notes

- Both buses use the **new ESP-IDF I²C master driver (`i2c_new_master_bus`)**.  
- Internal pullups are enabled in software for simplicity.  
- PMU bus must always be initialized before modem bring-up.

---

