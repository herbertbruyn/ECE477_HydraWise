#include "bms.h"

// I2C master configuration
#define I2C_MASTER_SCL_IO           16          // GPIO number for I2C SCL
#define I2C_MASTER_SDA_IO           17          // GPIO number for I2C SDA
#define I2C_MASTER_NUM              I2C_NUM_1   // I2C port number
#define I2C_MASTER_FREQ_HZ          100000      // I2C clock frequency
#define I2C_MASTER_TIMEOUT_MS       1000        // Timeout for I2C operations

static const char *TAG = "BatteryGauge";
volatile int battery_level = 0;

/**
 * @brief Initialize the I2C master.
 */
esp_err_t i2c_master_init_bms(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        printf("I2C config failed: %s\n", esp_err_to_name(err));
        return err;
    }
    esp_err_t driver = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (driver != ESP_OK) {
        printf("I2C driver install failed: %s\n", esp_err_to_name(driver));
    }
    return driver;
}

/**
 * @brief Reads a 16-bit word from the bq27441 starting at the specified register.
 *
 * @param reg The register address to read from.
 * @param data Pointer to store the 16-bit value read.
 * @return esp_err_t ESP_OK on success.
 */
static esp_err_t bq27441_read_word(uint8_t reg, uint16_t *data)
{
    uint8_t buf[2] = {0};
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, BQ27441_ADDR,
                                                 &reg, 1, buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err == ESP_OK) {
        // Combine LSB and MSB (LSB first)
        *data = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    }
    return err;
}

/**
 * @brief Task that periodically reads the battery state-of-charge (SoC) as a percentage.
 */
void battery_soc_task(void *arg)
{
    i2c_master_init_bms();
    vTaskDelay(pdMS_TO_TICKS(100));

    uint16_t soc_raw = 0;
    while (1) {
        if (bq27441_read_word(REG_SOC_LSB, &soc_raw) == ESP_OK) {
            // According to the datasheet, the SoC value represents percentage directly.
            // For example, a value of 50 means 50%.
            battery_level = soc_raw;
            ESP_LOGI(TAG, "Battery SoC: %d%%", soc_raw);
        } else {
            ESP_LOGE(TAG, "Failed to read battery state-of-charge");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Read every 1 second
    }
}

void i2c_scan_coulomb() {
    esp_err_t err = i2c_master_init_bms();
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
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/i2c.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include "nvs_flash.h"
// #include "nvs.h"

// // I2C master configuration
// #define I2C_MASTER_SCL_IO           22          // GPIO for I2C SCL
// #define I2C_MASTER_SDA_IO           21          // GPIO for I2C SDA
// #define I2C_MASTER_NUM              I2C_NUM_0   // I2C port number
// #define I2C_MASTER_FREQ_HZ          100000      // I2C clock frequency
// #define I2C_MASTER_TIMEOUT_MS       1000        // Timeout for I2C operations

// // bq27441-G1 I2C address (7-bit)
// #define BQ27441_ADDR                0x55

// // Register addresses for Coulomb Counter or State-of-Charge (example)
// // In this example, we use Remaining Capacity registers (0x0C/0x0D) as a proxy.
// #define REG_REMAINING_CAPACITY_LSB  0x0C

// // GPIO used for simulating a toggle switch (battery connected/disconnected)
// #define TOGGLE_SWITCH_GPIO          4

// static const char *TAG = "FuelGauge";

// // NVS namespace and key
// #define STORAGE_NAMESPACE "storage"
// #define KEY_COULOMB "coulomb_val"

// // Forward declarations
// static esp_err_t bq27441_read_word(uint8_t reg, uint16_t *data);
// static void save_coulomb_value(uint16_t value);
// static uint16_t load_coulomb_value(void);
// static void recalibrate_gauge(uint16_t stored_value);

// /**
//  * @brief Read a 16-bit word from the bq27441 starting at the specified register.
//  */
// static esp_err_t bq27441_read_word(uint8_t reg, uint16_t *data)
// {
//     uint8_t buf[2] = {0};
//     esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, BQ27441_ADDR,
//                                                  &reg, 1, buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
//     if (err == ESP_OK) {
//         // Combine LSB and MSB (LSB first)
//         *data = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
//     }
//     return err;
// }

// /**
//  * @brief Save the coulomb counter value in NVS.
//  */
// static void save_coulomb_value(uint16_t value)
// {
//     nvs_handle_t my_handle;
//     esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
//     if (err == ESP_OK) {
//         err = nvs_set_u16(my_handle, KEY_COULOMB, value);
//         if (err == ESP_OK) {
//             err = nvs_commit(my_handle);
//             if (err == ESP_OK) {
//                 ESP_LOGI(TAG, "Saved coulomb value: %d", value);
//             }
//         }
//         nvs_close(my_handle);
//     } else {
//         ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
//     }
// }

// /**
//  * @brief Load the coulomb counter value from NVS.
//  */
// static uint16_t load_coulomb_value(void)
// {
//     uint16_t stored_value = 0;
//     nvs_handle_t my_handle;
//     esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
//     if (err == ESP_OK) {
//         err = nvs_get_u16(my_handle, KEY_COULOMB, &stored_value);
//         if (err == ESP_OK) {
//             ESP_LOGI(TAG, "Loaded stored coulomb value: %d", stored_value);
//         } else {
//             ESP_LOGW(TAG, "No stored coulomb value found, using default 0");
//         }
//         nvs_close(my_handle);
//     } else {
//         ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
//     }
//     return stored_value;
// }

// /**
//  * @brief Simulate gauge recalibration by restoring the stored coulomb counter value.
//  * In an actual design, you might write to a gauge register or adjust your algorithm.
//  */
// static void recalibrate_gauge(uint16_t stored_value)
// {
//     ESP_LOGI(TAG, "Recalibrating gauge with stored value: %d", stored_value);
//     // Here, implement any necessary gauge recalibration steps.
//     // For demonstration, we just log the stored value.
// }

// /**
//  * @brief Task that monitors the toggle switch and reads the coulomb counter.
//  */
// static void fuel_gauge_task(void *arg)
// {
//     bool battery_connected = true;  // Assume initially connected
//     uint16_t coulomb_val = 0;
//     while (1) {
//         // Read the toggle switch state
//         int switch_state = gpio_get_level(TOGGLE_SWITCH_GPIO);
//         if (switch_state == 0 && battery_connected) {
//             // Simulate disconnect: battery now disconnected from the coulomb counter circuit.
//             battery_connected = false;
//             // Save the current coulomb counter value
//             if (bq27441_read_word(REG_REMAINING_CAPACITY_LSB, &coulomb_val) == ESP_OK) {
//                 save_coulomb_value(coulomb_val);
//             } else {
//                 ESP_LOGE(TAG, "Failed to read coulomb counter during disconnect");
//             }
//         } else if (switch_state == 1 && !battery_connected) {
//             // Simulate reconnect: battery reconnected
//             battery_connected = true;
//             // Load the stored coulomb value and recalibrate gauge
//             uint16_t stored_val = load_coulomb_value();
//             recalibrate_gauge(stored_val);
//         }
//         // If battery is connected, continue reading coulomb counter normally
//         if (battery_connected) {
//             if (bq27441_read_word(REG_REMAINING_CAPACITY_LSB, &coulomb_val) == ESP_OK) {
//                 ESP_LOGI(TAG, "Battery coulomb counter: %d", coulomb_val);
//             } else {
//                 ESP_LOGE(TAG, "Failed to read coulomb counter");
//             }
//         } else {
//             ESP_LOGW(TAG, "Battery disconnected; gauge is paused.");
//         }
//         vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 1 second between checks
//     }
// }

// /**
//  * @brief Initialize I2C master.
//  */
// static void i2c_master_init(void)
// {
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = I2C_MASTER_FREQ_HZ,
//     };
//     i2c_param_config(I2C_MASTER_NUM, &conf);
//     i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
// }

// /**
//  * @brief Initialize the toggle switch GPIO.
//  */
// static void toggle_switch_init(void)
// {
//     gpio_config_t io_conf = {
//         .pin_bit_mask = (1ULL << TOGGLE_SWITCH_GPIO),
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_ENABLE,   // Use internal pull-up; assume switch pulls to ground when off
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE,
//     };
//     gpio_config(&io_conf);
// }

// /**
//  * @brief Main application entry point.
//  */
// void app_main(void)
// {
//     // Initialize NVS
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
//         ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         nvs_flash_erase();
//         nvs_flash_init();
//     }

//     i2c_master_init();
//     toggle_switch_init();

//     // Create the fuel gauge monitoring task
//     xTaskCreate(fuel_gauge_task, "fuel_gauge_task", 4096, NULL, 10, NULL);
// }
