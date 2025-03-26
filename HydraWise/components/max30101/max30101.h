#ifndef MAX30101_H
#define MAX30101_H

#include "driver/i2c.h"
#include "esp_err.h"

// I2C Definitions
#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_MASTER_NUM      I2C_NUM_0
#define MAX30101_ADDR       0x57

// MAX30101 Registers
#define MAX30101_REG_INTR_STATUS_1  0x00
#define MAX30101_REG_FIFO_DATA      0x07

// Function Prototypes
esp_err_t max30101_read_register(uint8_t reg, uint8_t *data);
esp_err_t max30101_read_fifo(uint32_t *buffer, uint8_t num_samples);
esp_err_t i2c_master_init(void);
void max30101_task(void *arg);
void i2c_scan();

#endif // MAX30101_H