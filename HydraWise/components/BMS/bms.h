#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

// bq27441-G1 I2C address (7-bit)
#define BQ27441_ADDR                0x55

// Register addresses for State of Charge (SoC)
// 0x1C: SoC LSB, 0x1D: SoC MSB
#define REG_SOC_LSB                 0x1C

extern volatile int battery_level;
void battery_soc_task(void *arg);
void i2c_scan_coulomb(void);