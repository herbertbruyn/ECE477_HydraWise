#include <stdio.h>
// #include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max30101.h"
#include "esp_timer.h"
#include "algorithm.h"
#include "math.h"

// I2C Definitions
#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_MASTER_NUM      I2C_NUM_0
#define FIFO_BUFFER_SIZE 16 // (RED + IR samples)
#define HR_HISTORY_SIZE 100

// #define ALPHA 0.2
double auto_correlationated_data[BUFFER_SIZE];
int32_t ir_buffer[BUFFER_SIZE];
int32_t red_buffer[BUFFER_SIZE];
static bool i2c_driver_installed = false;  // Tracks if driver is installed
volatile int latest_heart_rate = 0;
int64_t start_time_us = 0;
int bufferIndex = 0;
int hr_history_index = 0;
HeartRateSample hr_history[HR_HISTORY_SIZE];

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
 * @brief Write to a MAX30101 register
 */
esp_err_t max30101_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MAX30101_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}

/**
 * @brief Read from a MAX30101 register
 */
esp_err_t max30101_read_register(uint8_t reg, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief Initialize MAX30101 sensor with Green LED configuration
 */
esp_err_t max30101_init() {
    printf("Initializing MAX30101...\n");

    // Reset Sensor
    max30101_write_register(MODE_CONFIG_REG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow time for reset
    
    //Set Set SpO2 mode (uses RED and IR LEDs - uses Slot 1 and 2 only)
    max30101_write_register(0x09, 0x03);  

    // Configure FIFO: Avg = 4, Rollover enabled, Almost full = 15 samples
    max30101_write_register(0x08, 0x4F);  

    // Set ADC resolution and sample rate (18-bit, 100Hz)
    max30101_write_register(0x0A, 0x27);  

    // Set LED pulse amplitude
    max30101_write_register(0x0C, 0x24);  // RED LED On 
    max30101_write_register(0x0D, 0x24);  // IR LED On
    max30101_write_register(0x0E, 0x00);  // GREEN LED Off

    // Enable Multi-LED Mode
    max30101_write_register(0x11, 0x21);  // SLOT1 (RED) = Enabled
    max30101_write_register(0x12, 0x03);  // SLOT2 (IR) = Enabled

    printf("MAX30101 configured for RED and IR LED.\n");

    return ESP_OK;
}

esp_err_t max30101_read_fifo(int32_t *red_buffer, int32_t *ir_buffer, uint8_t num_samples) {
    uint8_t fifo_data[6 * FIFO_BUFFER_SIZE];  // 3 bytes each for RED and IR per sample
    uint16_t bytes_to_read = num_samples * 6;

    // Prepare I2C read command
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, FIFO_DATA_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | I2C_MASTER_READ, true);

    for (uint16_t i = 0; i < bytes_to_read; i++) {
        i2c_master_read_byte(cmd, &fifo_data[i], (i == (bytes_to_read - 1)) ? I2C_MASTER_LAST_NACK : I2C_MASTER_ACK);
    }

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK){
        return ret;
    }
    // Process data into RED and IR buffers + time array
    int64_t now_us = esp_timer_get_time();

    for (uint8_t i = 0; i < num_samples; i++) {
        uint8_t idx = (bufferIndex + i) % BUFFER_SIZE;

        red_buffer[idx] = ((uint32_t)fifo_data[i * 6] << 16) |
                  ((uint32_t)fifo_data[i * 6 + 1] << 8) |
                  fifo_data[i * 6 + 2];

        ir_buffer[idx] = ((uint32_t)fifo_data[i * 6 + 3] << 16) |
                        ((uint32_t)fifo_data[i * 6 + 4] << 8) |
                        fifo_data[i * 6 + 5];

        time_array[idx] = (now_us - start_time_us) / 1000.0f + i * 10.0f;
    }

    // Slide buffer forward
    bufferIndex = (bufferIndex + num_samples) % BUFFER_SIZE;

    return ESP_OK;
}

/**
 * @brief Update FIFO_RD_PTR
 */
esp_err_t max30101_update_fifo_rd_ptr(uint8_t fifo_wr_ptr) {
    return max30101_write_register(FIFO_RD_PTR_REG, fifo_wr_ptr);
}

/**
 * @brief Retrieve MAX30101 temperature
 */
float get_max30101_temp() {
    uint8_t int_temp = 0;
    uint8_t decimal_temp = 0;

    // Start temperature conversion
    max30101_write_register(REG_TEMP_CONFIG, 0x01);

    // Read integer and fractional temperature parts
    max30101_read_register(REG_TEMP_INTR, &int_temp);
    max30101_read_register(REG_TEMP_FRAC, &decimal_temp);

    return int_temp + (decimal_temp / 10.0f);
}

/**
 * @brief Average Heart Rates Calculated over the last 10s
 */
int get_average_hr_last_10s() {
    int64_t now = esp_timer_get_time() / 1000; // current time in ms
    int sum = 0;
    int count = 0;

    for (int i = 0; i < HR_HISTORY_SIZE; ++i) {
        if (now - hr_history[i].timestamp_ms <= 10000) {
            sum += hr_history[i].value;
            count++;
        }
    }

    return (count > 0) ? (sum / count) : 0;
}

/**
 * @brief Read heart rate data
 */
void max30101_task(void *arg) {
    uint8_t fifo_wr_ptr, fifo_rd_ptr;

    // Initialize I2C
    if (i2c_master_init() != ESP_OK) {
        printf("Failed to initialize I2C\n");
        vTaskDelete(NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize MAX30101
    if (max30101_init() != ESP_OK) {
        printf("Failed to initialize MAX30101\n");
        vTaskDelete(NULL);
    }

    // init_time_array();
    start_time_us = esp_timer_get_time();  // microseconds since boot

    while (1) {
        // First transaction: Get FIFO_WR_PTR and FIFO_RD_PTR
        if (max30101_read_register(FIFO_WR_PTR_REG, &fifo_wr_ptr) != ESP_OK ||
            max30101_read_register(FIFO_RD_PTR_REG, &fifo_rd_ptr) != ESP_OK) {
            // printf("Failed to read FIFO pointers\n");
            continue;
        }

        // Calculate available samples
        uint8_t num_available_samples = (fifo_wr_ptr >= fifo_rd_ptr) ? 
                                (fifo_wr_ptr - fifo_rd_ptr) : 
                                (32 + fifo_wr_ptr - fifo_rd_ptr);
        uint8_t num_samples_to_read = num_available_samples > FIFO_BUFFER_SIZE ? FIFO_BUFFER_SIZE : num_available_samples;

        if (num_samples_to_read >= 16) {
            // Second transaction: Read samples
            if (max30101_read_fifo(red_buffer, ir_buffer, num_samples_to_read) == ESP_OK) {
                // printf("Successfully read %d samples from FIFO\n", num_samples_to_read);

                // float temperature = get_max30101_temp();
                uint64_t ir_mean, red_mean;
                double r0_autocorrelation;

                remove_dc_part(ir_buffer, red_buffer, &ir_mean, &red_mean);
                remove_trend_line(ir_buffer);
                remove_trend_line(red_buffer);

                // double pearson_correlation = correlation_datay_datax(red_buffer, ir_buffer);
                int heart_rate = calculate_heart_rate(ir_buffer, &r0_autocorrelation, auto_correlationated_data);
                if (heart_rate > 39 && heart_rate < 201){
                    // Store in history buffer
                    hr_history[hr_history_index].value = heart_rate;
                    hr_history[hr_history_index].timestamp_ms = esp_timer_get_time() / 1000; // Convert to ms
                    hr_history_index = (hr_history_index + 1) % HR_HISTORY_SIZE;
                    //latest_heart_rate = heart_rate;

                }
                // printf("\n HEART RATE: %d\n", latest_heart_rate);
            } else {
                printf("Failed to read FIFO data\n");
                // Third transaction: Update FIFO_RD_PTR if necessary
                max30101_update_fifo_rd_ptr(fifo_wr_ptr);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Delay 10 ms to match 100Hz sample rate
    }
}

void i2c_scan_max301x() {
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