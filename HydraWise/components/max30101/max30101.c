#include <stdio.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max30101.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define MAX30101_ADDR 0x57

#define FIFO_WR_PTR_REG 0x04
#define FIFO_RD_PTR_REG 0x06
#define FIFO_DATA_REG 0x07
#define MODE_CONFIG_REG 0x09
#define SPO2_CONFIG_REG 0x0A
#define LED_CONFIG_REG  0x0C
#define MULTI_LED_CTRL1 0x11
#define MULTI_LED_CTRL2 0x12

#define BUFFER_SIZE 32
#define THRESHOLD 20000  // Adjust threshold based on IR intensity
#define BUFFER_SIZE 100  // Buffer to hold 50 data samples

uint32_t hrBuffer[BUFFER_SIZE]; // Store heart rate values
static uint32_t lastPeakTime = 0;
static uint8_t numBeats = 0;
static uint32_t timeStamps[BUFFER_SIZE]; // Store time of each detected beat
static uint8_t bufferIndex = 0;

static bool i2c_driver_installed = false;  // Tracks if driver is installed

/**
 * @brief Detect Heart Rates
 */
void detect_heart_rate() {
    uint32_t currentTime;
    uint32_t timeDiff;
    for (uint8_t i = 1; i < BUFFER_SIZE - 1; i++) {
        if ((hrBuffer[i] > THRESHOLD) && (hrBuffer[i] > hrBuffer[i - 1]) && (hrBuffer[i] > hrBuffer[i + 1])) { // check for peak above threshold
            currentTime = esp_timer_get_time() / 1000; // Get time in milliseconds
            timeDiff = currentTime - lastPeakTime;

            if (timeDiff > 300) {  // Ignore noise (minimum time between beats ~300ms)
                timeStamps[numBeats % BUFFER_SIZE] = currentTime;
                numBeats++;
                lastPeakTime = currentTime;
            }
        }
    }
}

/**
 * @brief Convert Heart Rate to BPM
 */
uint8_t calculate_bpm() {
    if (numBeats < 2) return 0;  // Not enough data

    uint32_t totalTime = timeStamps[(numBeats - 1) % BUFFER_SIZE] - timeStamps[0];
    uint32_t avgTimeBetweenBeats = totalTime / (numBeats - 1);

    return (60000 / avgTimeBetweenBeats); // Convert ms to BPM
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

    // // Reset Sensor
    // max30101_write_register(MODE_CONFIG_REG, 0x40);
    // vTaskDelay(pdMS_TO_TICKS(100)); // Allow time for reset

    // // Set SpO2 Mode for multi-LED (Green LED Only)
    // max30101_write_register(MODE_CONFIG_REG, 0x03);

    // // Set ADC resolution and sample rate (18-bit, 100Hz)
    // max30101_write_register(SPO2_CONFIG_REG, 0x27);

    // // Configure LED Power - Green LED on, IR/Red LEDs off
    // max30101_write_register(LED_CONFIG_REG, 0x00); // Red LED Off
    // max30101_write_register(LED_CONFIG_REG + 1, 0x00); // IR LED Off
    // max30101_write_register(LED_CONFIG_REG + 2, 0xFF); // Green LED Max Power

    // // Configure Multi-LED Mode (Enable Green LED only)
    // max30101_write_register(MULTI_LED_CTRL1, 0x03); // Green LED in Slot 1
    // max30101_write_register(MULTI_LED_CTRL2, 0x00); // Other Slots Disabled
    

    //Set Heart Rate mode (uses Multi LED Mode - Green and IR)
    max30101_write_register(0x09, 0x07);  

    // Enable FIFO with max sample storage
    max30101_write_register(0x08, 0x4F);  

    // Set ADC resolution and sample rate (18-bit, 100Hz)
    max30101_write_register(0x0A, 0x27);  

    // Set LED pulse amplitude
    max30101_write_register(0x0C, 0x00);  // RED LED Off (not needed)
    max30101_write_register(0x0D, 0x00);  // IR LED Off
    max30101_write_register(0x0E, 0xFF);  // GREEN LED On (Increase Power)

    // Enable Multi-LED Mode (Green in SLOT1)
    max30101_write_register(0x11, 0x03);  // SLOT1 = GREEN LED
    max30101_write_register(0x12, 0x00);  // SLOT2 = Disable
    max30101_write_register(0x13, 0x00);  // SLOT3 = Disable
    max30101_write_register(0x14, 0x00);  // SLOT4 = Disable

    printf("MAX30101 configured for Green LED.\n");

    return ESP_OK;
}

/**
 * @brief Read FIFO data
 */
esp_err_t max30101_read_fifo(uint32_t *buffer, uint8_t num_samples) {
    if (num_samples > BUFFER_SIZE) {
        num_samples = BUFFER_SIZE;
    }

    uint8_t fifo_data[3 * BUFFER_SIZE];  // 3 bytes per sample

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, FIFO_DATA_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | I2C_MASTER_READ, true);

    for (uint8_t i = 0; i < num_samples * 3; i++){
        i2c_master_read_byte(cmd, &fifo_data[i], (i == (num_samples * 3 - 1)) ? I2C_MASTER_LAST_NACK : I2C_MASTER_ACK);
    }

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return ret;

    // Process FIFO data
    for (uint8_t i = 0; i < num_samples; i++) {
        buffer[i] = ((uint32_t)fifo_data[i * 3] << 16) | ((uint32_t)fifo_data[i * 3 + 1] << 8) | fifo_data[i * 3 + 2];
        printf("Sample %d - Green LED: %lu\n", i, buffer[i]);
    }

    return ESP_OK;
}

/**
 * @brief Update FIFO_RD_PTR
 */
esp_err_t max30101_update_fifo_rd_ptr(uint8_t fifo_wr_ptr) {
    return max30101_write_register(FIFO_RD_PTR_REG, fifo_wr_ptr);
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
    // Initialize MAX30101
    if (max30101_init() != ESP_OK) {
        printf("Failed to initialize MAX30101\n");
        vTaskDelete(NULL);
    }

    while (1) {
        // First transaction: Get FIFO_WR_PTR and FIFO_RD_PTR
        if (max30101_read_register(FIFO_WR_PTR_REG, &fifo_wr_ptr) != ESP_OK ||
            max30101_read_register(FIFO_RD_PTR_REG, &fifo_rd_ptr) != ESP_OK) {
            printf("Failed to read FIFO pointers\n");
            continue;
        }

        // Calculate available samples
        uint8_t num_available_samples = (fifo_wr_ptr - fifo_rd_ptr) % 32;
        uint8_t num_samples_to_read = num_available_samples > BUFFER_SIZE ? BUFFER_SIZE : num_available_samples;

        if (num_samples_to_read > 0) {
            // Second transaction: Read samples
            if (max30101_read_fifo(hrBuffer, num_samples_to_read) == ESP_OK) {
                printf("Successfully read %d samples from FIFO\n", num_samples_to_read);
                detect_heart_rate()
            } else {
                printf("Failed to read FIFO data\n");
            }

            // Third transaction: Update FIFO_RD_PTR if necessary
            if (max30101_update_fifo_rd_ptr(fifo_wr_ptr) != ESP_OK) {
                printf("Failed to update FIFO_RD_PTR\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before next read
    }
}

// #include <stdio.h>
// #include "driver/i2c_master.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "max30101.h"
// #include "esp_timer.h"

// #define ACK_VAL  0x0
// #define NACK_VAL 0x1
// #define ACK_CHECK_EN  0x1
// #define ACK_CHECK_DIS 0x0
// #define WRITE_BIT I2C_MASTER_WRITE
// #define READ_BIT  I2C_MASTER_READ
// #define I2C_MASTER_NUM I2C_NUM_0
// #define I2C_MASTER_TX_BUF_DISABLE 0
// #define I2C_MASTER_RX_BUF_DISABLE 0
// #define portTICK_PERIOD_MS ( ( TickType_t ) 1000 / configTICK_RATE_HZ )

// #define THRESHOLD 20000  // Adjust threshold based on IR intensity
// #define BUFFER_SIZE 100  // Buffer to hold 50 data samples
// uint32_t irBuffer[BUFFER_SIZE]; // Store 50 IR values
// static uint32_t lastPeakTime = 0;
// static uint8_t numBeats = 0;
// static uint32_t timeStamps[BUFFER_SIZE]; // Store time of each detected beat
// static uint8_t bufferIndex = 0;

// static bool i2c_driver_installed = false;  // Tracks if driver is installed

// void detect_heart_rate() {
//     uint32_t currentTime;
//     uint32_t timeDiff;
//     for (uint8_t i = 1; i < BUFFER_SIZE - 1; i++) {
//         if ((irBuffer[i] > THRESHOLD) && (irBuffer[i] > irBuffer[i - 1]) && (irBuffer[i] > irBuffer[i + 1])) { // check for peak above threshold
//             currentTime = esp_timer_get_time() / 1000; // Get time in milliseconds
//             timeDiff = currentTime - lastPeakTime;

//             if (timeDiff > 300) {  // Ignore noise (minimum time between beats ~300ms)
//                 timeStamps[numBeats % BUFFER_SIZE] = currentTime;
//                 numBeats++;
//                 lastPeakTime = currentTime;
//             }
//         }
//     }
// }

// uint8_t calculate_bpm() {
//     if (numBeats < 2) return 0;  // Not enough data

//     uint32_t totalTime = timeStamps[(numBeats - 1) % BUFFER_SIZE] - timeStamps[0];
//     uint32_t avgTimeBetweenBeats = totalTime / (numBeats - 1);

//     return (60000 / avgTimeBetweenBeats); // Convert ms to BPM
// }

// /**
//  * @brief Read a single register from MAX30101
//  */
// esp_err_t max30101_read_register(uint8_t reg, uint8_t *data) {
//     printf("Read single register \n");
//     int ret;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
//     // Send register address
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     vTaskDelay(30 / portTICK_PERIOD_MS);  // Small delay for stable read

//     // Read data from the selected register
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
//     i2c_master_read_byte(cmd, data, NACK_VAL);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);

//     return ret;
// }

// /**
//  * @brief Read 3-byte FIFO data from MAX30101
//  */
// esp_err_t max30101_read_fifo(uint32_t *green_value) {
//     uint8_t fifo_data[6] = {0};  // Buffer for Green LED + Additional Data
//     esp_err_t ret;
    
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, MAX30101_REG_FIFO_DATA, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
    
//     ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     if (ret != ESP_OK) {
//         printf("Failed to initiate FIFO read command\n");
//         return ret;
//     }

//     vTaskDelay(pdMS_TO_TICKS(10));  // Short delay

//     // Read 6 bytes from FIFO
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
//     i2c_master_read(cmd, fifo_data, 6, I2C_MASTER_LAST_NACK);
//     i2c_master_stop(cmd);
    
//     ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
    
//     if (ret == ESP_OK) {
//         *green_value = ((uint32_t)fifo_data[0] << 16) | ((uint32_t)fifo_data[1] << 8) | fifo_data[2];  // Extract Green LED value
//     } else {
//         printf("FIFO Read Error\n");
//     }

//     return ret;
// }


// /**
//  * @brief I2C master initialization
//  */
// esp_err_t i2c_master_init(void) {
//     // Check if driver is already installed
//     if (i2c_driver_installed) {
//         printf("I2C driver is already installed. Skipping installation.\n");
//         return ESP_OK;
//     }
//     printf("Initializing I2C...\n");
//     int i2c_master_port = I2C_MASTER_NUM;
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = I2C_MASTER_FREQ_HZ,
//     };
//     esp_err_t err = i2c_param_config(i2c_master_port, &conf);
//     if (err != ESP_OK) {
//         printf("I2C config failed: %s\n", esp_err_to_name(err));
//         return err;
//     }
//     esp_err_t driver = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
//     if (driver != ESP_OK) {
//         printf("I2C driver install failed: %s\n", esp_err_to_name(driver));
//     }
//     else{
//         i2c_driver_installed = true;
//     }
//     return driver;
// }

// /**
//  * @brief Write to register from MAX30101
//  */
// esp_err_t max30101_write_register(uint8_t reg, uint8_t value) {
//     uint8_t data[2] = {reg, value};
//     return i2c_master_write_to_device(I2C_MASTER_NUM, MAX30101_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
// }

// /**
//  * @brief Setup MAX30101 for Heart Rate sampling
//  */
// esp_err_t max30101_init_hr_mode() {
//     max30101_write_register(0x09, 0x40); // Reset device
//     vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset to complete
    
//     printf("Initializing MAX30101 for Heart Rate Mode...\n");

//     // Set Heart Rate mode (uses Multi LED Mode - Green and IR)
//     max30101_write_register(0x09, 0x07);  

//     // Enable FIFO with max sample storage
//     max30101_write_register(0x08, 0x4F);  

//     // Set ADC resolution and sample rate (18-bit, 100Hz)
//     max30101_write_register(0x0A, 0x27);  

//     // Set LED pulse amplitude
//     max30101_write_register(0x0C, 0x00);  // RED LED Off (not needed)
//     max30101_write_register(0x0D, 0x00);  // IR LED Off
//     max30101_write_register(0x0E, 0xFF);  // GREEN LED On (Increase Power)

//     // Enable Multi-LED Mode (Green in SLOT1)
//     max30101_write_register(0x11, 0x03);  // SLOT1 = GREEN LED
//     max30101_write_register(0x12, 0x00);  // SLOT2 = Disable
//     max30101_write_register(0x13, 0x00);  // SLOT3 = Disable
//     max30101_write_register(0x14, 0x00);  // SLOT4 = Disable

//     // Enable FIFO Data Ready Interrupt
//     max30101_write_register(0x02, 0x40);
//     vTaskDelay(pdMS_TO_TICKS(100));

//     printf("MAX30101 Heart Rate Mode Initialized.\n");

//     uint8_t mode;
//     max30101_read_register(0x09, &mode);
//     printf("MAX30101 Mode Register: 0x%02X\n", mode);
//     uint8_t fifo_config;
//     max30101_read_register(0x08, &fifo_config);
//     printf("FIFO Config Register: 0x%02X\n", fifo_config);
//     uint8_t led_ir, led_red, led_green;
//     max30101_read_register(0x0C, &led_ir);
//     max30101_read_register(0x0D, &led_red);
//     max30101_read_register(0x0E, &led_green);
//     printf("LED1 (IR) Amplitude: 0x%02X | LED2 (Red) Amplitude: 0x%02X | LED3 (Green) Amplitude: 0x%02X\n", led_ir, led_red, led_green);
//     uint8_t intr_status;
//     max30101_read_register(0x07, &intr_status);
//     printf("Interrupt Status Register: 0x%02X\n", intr_status);
//     uint8_t intr_enable;
//     max30101_read_register(0x07, &intr_enable);
//     printf("Interrupt Enable Register: 0x%02X\n", intr_enable);


//     uint8_t fifo_data[6];
//     max30101_read_register(0x07, fifo_data);
//     printf("FIFO Data: %02X %02X %02X %02X %02X %02X\n", 
//        fifo_data[0], fifo_data[1], fifo_data[2], 
//        fifo_data[3], fifo_data[4], fifo_data[5]);

//     return ESP_OK;
// }

// /**
//  * @brief Main task to read sensor data
//  */
// void max30101_task(void *arg) {
//     uint8_t intr_status; 
//     uint32_t fifo_data = 0;
//     uint8_t bpm = 0;
    
//     // Initialize I2C
//     if (i2c_master_init() != ESP_OK) {
//         printf("Failed to initialize I2C\n");
//         vTaskDelete(NULL);
//     }
//     printf("I2C Task running for Heart Rate Data Collection \n");
//     if (max30101_init_hr_mode() != ESP_OK) {
//         printf("Failed to configure sensor for heart rate data collection\n");
//         vTaskDelete(NULL);
//     }

//     while (1) {
//         // Check if new data is available
//         if (max30101_read_register(MAX30101_REG_INTR_STATUS_1, &intr_status) == ESP_OK) {
//             // printf("Status Register: %u\n", intr_status);
//             if (intr_status & 0x40) {  // FIFO data ready
//                 // Read FIFO data
//                 if (max30101_read_fifo(&fifo_data) == ESP_OK) {
                    
//                     // Store in circular buffer
//                     irBuffer[bufferIndex] = fifo_data;
//                     bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
//                     detect_heart_rate();
//                     bpm = calculate_bpm();
//                     printf("Raw Sensor Data: %lu\n", fifo_data);
//                     printf("Heart Rate (bpm): %d \n", bpm);
//                     uint8_t d[6];
//                     max30101_read_register(0x07, d);
//                     printf("FIFO Data: %02X %02X %02X %02X %02X %02X\n", 
//                     d[0], d[1], d[2], 
//                     d[3], d[4], d[5]);
//                 } else {
//                     printf("Failed to read sensor data\n");
//                 }
//             }
//         } else {
//             printf("Failed to read interrupt status\n");
//         }
//         // Clear Interrupts
//         max30101_write_register(MAX30101_REG_INTR_STATUS_1, 0x00);

//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }

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

// // /**
// //  * @brief Entry point of the application
// //  */
// // void app_main(void) {
// //     xTaskCreate(max30101_task, "max30101_task", 4096, NULL, 5, NULL);
// // }


// // #include <stdio.h>
// // #include "driver/i2c.h"

// // #define I2C_MASTER_SCL_IO    22    // GPIO for SCL
// // #define I2C_MASTER_SDA_IO    21    // GPIO for SDA
// // #define I2C_MASTER_NUM       I2C_NUM_0
// // #define I2C_MASTER_FREQ_HZ   100000
// // #define I2C_MASTER_TX_BUF_DISABLE 0
// // #define I2C_MASTER_RX_BUF_DISABLE 0
// // #define MAX30101_ADDR        0x57  // MAX30101 I2C address

// // #define FIFO_WR_PTR_REG      0x04
// // #define FIFO_RD_PTR_REG      0x06
// // #define FIFO_DATA_REG        0x07

// // // I2C Initialization
// // void i2c_master_init() {
// //     i2c_config_t conf = {
// //         .mode = I2C_MODE_MASTER,
// //         .sda_io_num = I2C_MASTER_SDA_IO,
// //         .scl_io_num = I2C_MASTER_SCL_IO,
// //         .sda_pullup_en = GPIO_PULLUP_ENABLE,
// //         .scl_pullup_en = GPIO_PULLUP_ENABLE,
// //         .master.clk_speed = I2C_MASTER_FREQ_HZ,
// //     };
// //     i2c_param_config(I2C_MASTER_NUM, &conf);
// //     i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0);
// // }

// // // Function to read a single register from MAX30101
// // uint8_t read_register(uint8_t reg_addr) {
// //     uint8_t data;
// //     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
// //     i2c_master_start(cmd);
// //     i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | I2C_MASTER_WRITE, true);
// //     i2c_master_write_byte(cmd, reg_addr, true);
// //     i2c_master_start(cmd);
// //     i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | I2C_MASTER_READ, true);
// //     i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
// //     i2c_master_stop(cmd);
// //     i2c_cmd_link_delete(cmd);
// //     return data;
// // }

// // // Function to read FIFO data and print raw values
// // void read_fifo_data() {
// //     uint8_t fifo_wr_ptr = read_register(FIFO_WR_PTR_REG);
// //     uint8_t fifo_rd_ptr = read_register(FIFO_RD_PTR_REG);
// //     uint8_t num_samples = (fifo_wr_ptr - fifo_rd_ptr) & 0x1F; // Handling wrap-around

// //     if (num_samples == 0) return;
    
// //     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
// //     i2c_master_start(cmd);
// //     i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | I2C_MASTER_WRITE, true);
// //     i2c_master_write_byte(cmd, FIFO_DATA_REG, true);
// //     i2c_master_start(cmd);
// //     i2c_master_write_byte(cmd, (MAX30101_ADDR << 1) | I2C_MASTER_READ, true);

// //     for (uint8_t i = 0; i < num_samples; i++) {
// //         uint8_t led1[3];
// //         i2c_master_read_byte(cmd, &led1[0], I2C_MASTER_ACK);
// //         i2c_master_read_byte(cmd, &led1[1], I2C_MASTER_ACK);
// //         i2c_master_read_byte(cmd, &led1[2], (i == num_samples - 1) ? I2C_MASTER_LAST_NACK : I2C_MASTER_ACK);
// //         uint32_t raw_data = (led1[0] << 16) | (led1[1] << 8) | led1[2];
// //         printf("Raw HR Data: %lu\n", raw_data);
// //     }

// //     i2c_master_stop(cmd);
// //     i2c_cmd_link_delete(cmd);

// //     // Update FIFO_RD_PTR if needed
// //     i2c_cmd_handle_t cmd_write = i2c_cmd_link_create();
// //     i2c_master_start(cmd_write);
// //     i2c_master_write_byte(cmd_write, (MAX30101_ADDR << 1) | I2C_MASTER_WRITE, true);
// //     i2c_master_write_byte(cmd_write, FIFO_RD_PTR_REG, true);
// //     i2c_master_write_byte(cmd_write, fifo_wr_ptr, true);
// //     i2c_master_stop(cmd_write);
// //     i2c_cmd_link_delete(cmd_write);
// // }

// // void app_main() {
// //     i2c_master_init();
// //     printf("Starting Heart Rate Monitoring...\n");
// //     while (1) {
// //         read_fifo_data();
// //         vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 second
// //     }
// // }
