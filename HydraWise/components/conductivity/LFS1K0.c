#include <stdio.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"
#include "esp_err.h"
#include "esp_log.h"

#define SIGNAL_PIN GPIO_NUM_32 // Set this to GPIO pin

#define ADC_PIN ADC1_CHANNEL_5 // Set this to ADC pin
#define ADC_ATTEN ADC_ATTEN_DB_6 // Set this to ADC attenuation
#define ADC_WIDTH ADC_WIDTH_BIT_12 // Set this to ADC width

// Initialize GPIO pin
esp_err_t init_gpio() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SIGNAL_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&io_conf);
}

// Initialize ADC
esp_err_t init_adc() {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_PIN, ADC_ATTEN);
    return ESP_OK;
}

// Read ADC value - returns raw voltage
uint32t_t read_adc() {
    int raw = adc1_get_raw(ADC_PIN);
    return raw;
}