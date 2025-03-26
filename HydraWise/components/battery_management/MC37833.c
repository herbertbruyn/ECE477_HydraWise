#include "driver/gpio.h"
#include "esp_log.h"

// CHECK WITH HARDWARE

#define STAT1_PIN GPIO_NUM_4    // GPIO pin for STAT1
#define STAT2_PIN GPIO_NUM_5    // GPIO pin for STAT2

void app_main(void) {
    // configure GPIOs for STAT1 and STAT2
    gpio_config_t io_conf {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << STAT1_PIN) | (1ULL << STAT2_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);

    // read STAT1 and STAT2
    while (1) {
        int stat1_level = gpio_get_level(STAT1_PIN);
        int stat2_level = gpio_get_level(STAT2_PIN);

        if (stat1_level == 0 && stat2_level == 0) {
            ESP_LOGI("MC37833", "Battery is charging");
        } else if (stat1_level == 1 && stat2_level == 0) {
            ESP_LOGI("MC37833", "Battery is fully charged");
        } else if (stat1_level == 0 && stat2_level == 1) {
            ESP_LOGI("MC37833", "Battery is discharging");
        } else {
            ESP_LOGI("MC37833", "Battery is not connected");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // perhaps
        vTaskDelay(pdMS_TO_TICKS(1000)); // or this
    }

    // testing my push with my github username hopefully
}