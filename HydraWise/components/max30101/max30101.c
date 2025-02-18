#include <stdio.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max30101.h"

#define ACK_VAL  0x0
#define NACK_VAL 0x1
#define ACK_CHECK_EN  0x1
#define ACK_CHECK_DIS 0x0
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT  I2C_MASTER_READ
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define portTICK_PERIOD_MS ( ( TickType_t ) 1000 / configTICK_RATE_HZ )

static bool i2c_driver_installed = false;  // Tracks if driver is installed
/**
 * @brief Read a single register from MAX30101
 */
esp_err_t max30101_read_register(uint8_t reg, uint8_t *data) {
    printf("Read single register \n");
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Send register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(30 / portTICK_PERIOD_MS);  // Small delay for stable read

    // Read data from the selected register
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief Read 3-byte FIFO data from MAX30101
 */
esp_err_t max30101_read_fifo(uint8_t *data_h, uint8_t *data_m, uint8_t *data_l) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Select FIFO data register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MAX30101_REG_FIFO_DATA, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        printf(" Failed to initiate command queue \n");
        return ret;
    }

    vTaskDelay(30 / portTICK_PERIOD_MS);

    // Read three bytes from FIFO
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);  // Read MSB
    i2c_master_read_byte(cmd, data_m, ACK_VAL);  // Read Middle Byte
    i2c_master_read_byte(cmd, data_l, NACK_VAL); // Read LSB (last byte)
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf(" Failed to initiate command queue \n");
        return ret;
    }
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief I2C master initialization
 */
esp_err_t i2c_master_init(void) {
    // Check if driver is already installed
    if (i2c_driver_installed) {
        printf("I2C driver is already installed. Skipping installation.\n");
        return ESP_OK;
    }
    printf("Initializing I2C...\n");
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        printf("I2C config failed: %s\n", esp_err_to_name(err));
        return err;
    }
    esp_err_t driver = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    if (driver != ESP_OK) {
        printf("I2C driver install failed: %s\n", esp_err_to_name(driver));
    }
    else{
        i2c_driver_installed = true;
    }
    return driver;
}

/**
 * @brief Write to register from MAX30101
 */
esp_err_t max30101_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MAX30101_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}

/**
 * @brief Setup MAX30101 for Heart Rate sampling
 */
esp_err_t max30101_init_hr_mode() {
    printf("Initializing MAX30101 for Heart Rate Mode...\n");

    // Reset MAX30101
    max30101_write_register(0x09, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set Heart Rate mode (uses IR LED only)
    max30101_write_register(0x09, 0x02);  

    // Enable FIFO with max sample storage
    max30101_write_register(0x08, 0x0F);  

    // Set ADC resolution and sample rate (18-bit, 100Hz)
    max30101_write_register(0x0A, 0x27);  

    // Set LED pulse amplitude (IR LED at 24/255 power)
    max30101_write_register(0x0C, 0x24);  
    // Disable Red LED (not needed for HR mode)
    max30101_write_register(0x0D, 0x00); 

    // Enable FIFO Data Ready Interrupt
    max30101_write_register(0x07, 0x40);

    printf("MAX30101 Heart Rate Mode Initialized.\n");
    return ESP_OK;
}

/**
 * @brief Main task to read sensor data
 */
void max30101_task(void *arg) {
    uint8_t intr_status, fifo_data_h, fifo_data_m, fifo_data_l;
    
    // Initialize I2C
    if (i2c_master_init() != ESP_OK) {
        printf("Failed to initialize I2C\n");
        vTaskDelete(NULL);
    }
    printf("I2C Task running for Heart Rate Data Collection \n");
    if (max30101_init_hr_mode() != ESP_OK) {
        printf("Failed to configure sensor for heart rate data collection\n");
        vTaskDelete(NULL);
    }

    while (1) {
        // Check if new data is available
        if (max30101_read_register(MAX30101_REG_INTR_STATUS_1, &intr_status) == ESP_OK) {
            printf("Status Register: %u\n", intr_status);
            if (intr_status & 0x40) {  // FIFO data ready
                // Read FIFO data
                if (max30101_read_fifo(&fifo_data_h, &fifo_data_m, &fifo_data_l) == ESP_OK) {
                    uint32_t sensor_data = ((uint32_t)fifo_data_h << 16) | ((uint32_t)fifo_data_m << 8) | fifo_data_l;
                    printf("Raw Sensor Data: %lu\n", sensor_data);
                } else {
                    printf("Failed to read sensor data\n");
                }
            }
        } else {
            printf("Failed to read interrupt status\n");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void i2c_scan() {
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        printf("I2C initialization failed! Stopping...\n");
        return;
    }

    printf("I2C Initialized Successfully!\n");
    printf("Scanning I2C devices...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            printf("I2C device found at 0x%02X\n", addr);
        }
    }
}

// /**
//  * @brief Entry point of the application
//  */
// void app_main(void) {
//     xTaskCreate(max30101_task, "max30101_task", 4096, NULL, 5, NULL);
// }
