# POST and Debug Documentation
The system starts off with a Power On Self Test (POST) validating all required functions and a dump of register and main GPIO setting, writing them to a diagnostics logging on a SD card.

## Architecture of T-SIM7080G-S3
The T-SIM7080G-S3 integrates an ESP32-S3, AXP2101 PMU, SIM7080G modem, and SD slot. Power rails are controlled by the AXP2101.

## Detailed Power Path Explanation
- ESP32-S3 is powered from AXP2101 SYS rail.
- SIM7080G modem receives power from DCDC3 and BLDO2 rails.
- PWRKEY for the modem is controlled internally by AXP2101 GPIO, not the ESP32.
- Therefore esp-modem cannot toggle power; PMU register writes are required.

## How post_diagnostics Works
The POST system validates hardware at boot:
- I2C scan to detect sensors.
- UART1 AT probe to detect modem response.
- PMU configuration for DCDC3 and BLDO2.
- SD card mount test.
- PMU dump printed for debugging.

## How pmu_debug Works
pmu_debug reads all AXP2101 registers (0x00–0xFF) using I2C.  
It also decodes:
- DCDC_ONOFF (0x80)
- LDO_ONOFF0 (0x90)
- DCDC3 voltage code (0x84)
- BLDO2 voltage code (0x97)
and prints readable voltage values and enable states.

## How Your Custom PMU Logic Works
- Initializes a dedicated I2C bus for the PMU.
- Writes DCDC3 voltage code (0x66 → 3.00 V).
- Writes BLDO2 voltage code (0x1C → 3.30 V).
- Sets DCDC_ONOFF (0x80) and LDO_ONOFF0 (0x90).
- Ensures modem power rails are stable before sending AT commands.

## Lessons Learned
1. esp-modem cannot work on this board because power is controlled by AXP2101, not ESP32 GPIOs.
2. Proper PMU configuration is mandatory for modem startup.
3. DCDC3 must be enabled and set between 3.0–3.6 V for SIM7080G stability.
4. Bad I2C wiring or faulty PMU devices cause intermittent modem failures.
5. Full register dumps make diagnosing PMU behavior far easier.

