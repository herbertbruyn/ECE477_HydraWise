#include "conductivity.h"
#include "driver/gpio.h"

static const char *TAG = "Conductivity Sensor";
#define GPIO_OUTPUT_IO     14
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO)

void trigger_conductivity_measurement_task(void *param)
{
    // Clear Peak Detector
    // Configure GPIO 14 as output
    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Set GPIO 14 HIGH
    gpio_set_level(GPIO_OUTPUT_IO, 1);
    ESP_LOGI("GPIO", "GPIO 14 set HIGH");

    // Delay 2 seconds (2000 milliseconds)
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Set GPIO 14 LOW
    gpio_set_level(GPIO_OUTPUT_IO, 0);
    ESP_LOGI("GPIO", "GPIO 14 set LOW");

    // Start reading
    ESP_LOGI(TAG, "Starting DAC waveform");
    gen_cos_wave();      // From your dac.c

    ESP_LOGI(TAG, "Starting ADC sampling");
    adc();               // From your adc.c

    ESP_LOGI(TAG, "Conductivity measurement completed");
    ESP_LOGI(TAG, "Stopping DAC");
    stop_dac();  // Stop DAC wave output

    vTaskDelete(NULL);  // Kill the task after 10 seconds
}
