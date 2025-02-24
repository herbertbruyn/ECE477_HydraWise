#include <stdio.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"
#include "esp_err.h"
#include "esp_log.h"

#define SIGNAL_PIN GPIO_NUM_32 // Set this to GPIO pin

#define ADC_CHANNEL_0 ADC1_CHANNEL_0 // Pin for V+
#define ADC_CHANNEL_1 ADC1_CHANNEL_1 // Pin for V-
#define ADC_CHANNEL_2 ADC1_CHANNEL_2 // Pin for T1 (Temperature sensor)
#define ADC_CHANNEL_3 ADC1_CHANNEL_3 // Pin for T2
#define ADC_ATTEN ADC_ATTEN_DB_6 // Set this to ADC attenuation
#define ADC_WIDTH ADC_WIDTH_BIT_12 // Set this to ADC width
#define RTD_current .003 // 3mA for PT1000
#define R0 1000 // 1kΩ for PT1000

// Constants for resistance equation
#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12 //

// Initialize GPIO pin
esp_err_t init_gpio() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SIGNAL_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE // Disable interrupts - ours comes from different source 
    };
    return gpio_config(&io_conf);
}

// Initialize ADC
esp_err_t init_adc() {
    adc1_config_width(ADC_WIDTH);
    // Configure each channel
    adc1_config_channel_atten(ADC_CHANNEL_0, ADC_ATTEN);
    adc1_config_channel_atten(ADC_CHANNEL_1, ADC_ATTEN);
    adc1_config_channel_atten(ADC_CHANNEL_2, ADC_ATTEN);
    adc1_config_channel_atten(ADC_CHANNEL_3, ADC_ATTEN);
    return ESP_OK;
}

// Read ADC value - returns raw voltage
float read_adc_voltage(adc1_channel_t channel) {
    int raw = adc1_get_raw(channel);
    float voltage = (raw / 4095.0) * 2.2;
    return voltage;
}

// Find temp from voltage
float calculate_temperature(float voltage) {
    float resistance = voltage / RTD_current;


    float discriminant = A * A - 4 * B * (1 - (resistance / R0));
    float temp = (-A + sqrt(discriminant)) / (2 * B);
    return temp;
}

// Calculate resistanc
float calculate_resistance() {
    // Get ADC values
    float vplus = read_adc_voltage(ADC_CHANNEL_0);
    float vminus = read_adc_voltage(ADC_CHANNEL_1);
    float t1 = calculate_temperature(read_adc_voltage(ADC_CHANNEL_2));
    float t2 = calculate_temperature(read_adc_voltage(ADC_CHANNEL_3));

    // Find temp (C)
    float t = (t1 + t2) / 2;

    // Calculate resistance - differeent equations depending on if temp is below or above 0 C
    if (t < 0) {
        return R0 * (1 + A * t + B * t * t);
    } else {
        return R0 * (1 + A * t + B * t * t + C * (t - 100) * t * t * t);
    }
}
