/* 
 * Modified for reading voltage up to 1.1V from GPIO32 using ESP32 ADC1_CHANNEL_4
 */
 #include "conductivity.h"
 #include "esp_adc/adc_continuous.h"
 #include "nimble/nimble_port.h"
 #include "host/ble_gatt.h"
 #include "host/ble_hs.h"
 #include "host/util/util.h"
 #include "esp_adc/adc_cali.h"
 #include "esp_adc/adc_cali_scheme.h"

 
 #define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
 #define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
 #define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
 #define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
 #define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_6 // 6dB for 0-1.6V
 #define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH
 #define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
 #define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
 #define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
 #define EXAMPLE_READ_LEN                    256
 
 static adc_channel_t channel[1] = {ADC_CHANNEL_4}; // GPIO32
 
 static TaskHandle_t s_task_handle;
 static const char *TAG = "ADC_EXAMPLE";
 
 static adc_cali_handle_t adc_cali_handle = NULL;
 static bool do_calibration = false;

 static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = EXAMPLE_ADC_BIT_WIDTH,
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        *out_handle = handle;
        return true;
    } else {
        ESP_LOGW(TAG, "Calibration scheme not supported or failed");
        return false;
    }
}

 static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
 {
     BaseType_t mustYield = pdFALSE;
     vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
     return (mustYield == pdTRUE);
 }
 
 static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
 {
     adc_continuous_handle_t handle = NULL;
 
     adc_continuous_handle_cfg_t adc_config = {
         .max_store_buf_size = 1024,
         .conv_frame_size = EXAMPLE_READ_LEN,
     };
     ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));
 
     adc_continuous_config_t dig_cfg = {
         .sample_freq_hz = 20000,
         .conv_mode = EXAMPLE_ADC_CONV_MODE,
         .format = EXAMPLE_ADC_OUTPUT_TYPE,
     };
 
     adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
     dig_cfg.pattern_num = channel_num;
     for (int i = 0; i < channel_num; i++) {
         adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
         adc_pattern[i].channel = channel[i] & 0x7;
         adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
         adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;
     }
     dig_cfg.adc_pattern = adc_pattern;
     ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
 
     *out_handle = handle;
 }
 
 void adc(void)
 {
     esp_err_t ret;
     uint32_t ret_num = 0;
     uint8_t result[EXAMPLE_READ_LEN] = {0};
     double highest_voltage = 0;
    
     uint32_t count = 0;
 
     s_task_handle = xTaskGetCurrentTaskHandle();
 
     adc_continuous_handle_t handle = NULL;
     continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);
 
     adc_continuous_evt_cbs_t cbs = {
         .on_conv_done = s_conv_done_cb,
     };
     ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
     ESP_ERROR_CHECK(adc_continuous_start(handle));

     do_calibration = adc_calibration_init(EXAMPLE_ADC_UNIT, EXAMPLE_ADC_ATTEN, &adc_cali_handle);

     int64_t start_time = esp_timer_get_time(); // microseconds

     while ((esp_timer_get_time() - start_time) < 10000000) { // run for 10 seconds
        
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        while ((esp_timer_get_time() - start_time) < 10000000) {
            ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);

                    if (chan_num == ADC_CHANNEL_4) {
                        int voltage_mv = 0;
                        if (do_calibration) {
                            esp_err_t cal_ret = adc_cali_raw_to_voltage(adc_cali_handle, data, &voltage_mv);
                            if (cal_ret != ESP_OK) {
                                ESP_LOGW(TAG, "ADC calibration failed");
                                continue;
                            }
                        } else {
                            voltage_mv = (int)((float)data / ((1 << EXAMPLE_ADC_BIT_WIDTH) - 1) * 1600); // fallback if calibration fail
                        }
                        float voltage = voltage_mv / 1000.0f; // convert mV to V

                        if (voltage > highest_voltage){
                            highest_voltage = voltage;
                        }
                        count++;
                        ESP_LOGI(TAG, "GPIO32 (Channel %d): Raw: %d, Voltage: %.3f V", chan_num, data, voltage);
                    }
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }
    if (count > 0){
        // Send data over BLE
        struct os_mbuf *avg_voltage_om = ble_hs_mbuf_from_flat(&highest_voltage, sizeof(highest_voltage));
        int rc = ble_gattc_notify_custom(conn_handle_global, conductivity_handle, avg_voltage_om);

        if (rc != 0) {
            ESP_LOGE(TAG, "❌ Failed to send peak voltage over BLE: %d", rc);
        } else {
            ESP_LOGI(TAG, "✅ Peak voltage sent over BLE: %.3f V", highest_voltage);
        }
    }else {
        ESP_LOGW(TAG, "No ADC samples collected.");
    }
    ESP_LOGI(TAG, "Peak Voltage: %.3f V", highest_voltage);
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
    if (do_calibration) {
        ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(adc_cali_handle));
    }
 }