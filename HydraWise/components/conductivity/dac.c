#include "conductivity.h"
#include "driver/dac_cosine.h"

static const char *TAG = "DAC_Cosine_Wave";
static dac_cosine_handle_t chan0_handle = NULL; // Make this static/global in your module

// Function to generate a cosine wave on DAC Channel 0 (GPIO25)
void gen_cos_wave(void)
{
    // Configure the DAC channel 0 (GPIO25) for generating a cosine wave with 0.5 amplitude
    dac_cosine_config_t cos0_cfg = {
        .chan_id = DAC_CHAN_0,   // Use DAC Channel 0 (GPIO25)
        .freq_hz = 100000,         // Frequency for channel 0
        .clk_src = DAC_COSINE_CLK_SRC_DEFAULT,  // Default clock source
        .offset = -45,           // Offset to center the wave
        .phase = DAC_COSINE_PHASE_0,  // Phase starting at 0 degrees
        .atten = DAC_COSINE_ATTEN_DB_12,
        .flags.force_set_freq = false,  // Allow frequency to be set without forced updates
    };

    // Initialize and start the DAC channel 0 for continuous cosine output
    ESP_ERROR_CHECK(dac_cosine_new_channel(&cos0_cfg, &chan0_handle));
    ESP_ERROR_CHECK(dac_cosine_start(chan0_handle));

    ESP_LOGI(TAG, "Cosine wave generation started on channel 0 (GPIO25) at 150 Hz frequency, 0.5 amplitude.");
}

void stop_dac()
{
    if (chan0_handle) {
        ESP_ERROR_CHECK(dac_cosine_stop(chan0_handle));
        ESP_ERROR_CHECK(dac_cosine_del_channel(chan0_handle));
        chan0_handle = NULL;
        ESP_LOGI("DAC", "Cosine wave stopped and channel deleted");
    }
}