#ifndef MAX30101_H
#define MAX30101_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "algorithm.h"


// MAX30101 Registers
#define MAX30101_REG_INTR_STATUS_1  0x00
#define MAX30101_REG_FIFO_DATA      0x07
#define MAX30101_ADDR       0x57
#define FIFO_WR_PTR_REG 0x04
#define FIFO_RD_PTR_REG 0x06
#define FIFO_DATA_REG 0x07
#define MODE_CONFIG_REG 0x09
#define SPO2_CONFIG_REG 0x0A
#define LED_CONFIG_REG  0x0C
#define MULTI_LED_CTRL1 0x11
#define MULTI_LED_CTRL2 0x12
#define REG_TEMP_CONFIG 0x21
#define REG_TEMP_INTR   0x1F
#define REG_TEMP_FRAC   0x20

// Function Prototypes
esp_err_t max30101_read_register(uint8_t reg, uint8_t *data);
esp_err_t max30101_read_fifo(int32_t *red_buffer, int32_t *ir_buffer, uint8_t num_samples);
esp_err_t i2c_master_init(void);
void max30101_task(void *arg);
void i2c_scan_max301x();
int get_average_hr_last_10s(void);

// Global Variables
extern volatile int latest_heart_rate;

typedef struct {
    int value;
    int64_t timestamp_ms;
} HeartRateSample;

#endif // MAX30101_H
