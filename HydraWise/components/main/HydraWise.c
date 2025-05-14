#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max30101.h"
#include "HydraWiseBLE.h"
#include "conductivity.h"
#include "bms.h"
#include "driver/gpio.h"

#define BLINK_GPIO GPIO_NUM_13  // Use GPIO2 (built-in LED on many dev boards)

void app_main(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BLINK_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    // BLE Set Up
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    nimble_port_init();
    ble_svc_gap_device_name_set("HydraWise");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);
    // Note: The above task will send heart rate notifications every 3 seconds
    // This will allow the ESP32 to send heart rate notifications to connected clients.
    // The application will now start advertising and waiting for connections.
    // The notify_heart_rate_task will run in parallel and send notifications to the connected client.
    // i2c_scan_coulomb();
    while (1) {
        gpio_set_level(BLINK_GPIO, 1); // LED ON
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(BLINK_GPIO, 0); // LED OFF
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// #define BLINK_GPIO GPIO_NUM_13  // Use GPIO2 (built-in LED on many dev boards)
// #include "driver/gpio.h"
// void app_main(void) {
//     gpio_config_t io_conf = {
//         .pin_bit_mask = (1ULL << BLINK_GPIO),
//         .mode = GPIO_MODE_OUTPUT,
//         .pull_up_en = 0,
//         .pull_down_en = 0,
//         .intr_type = GPIO_INTR_DISABLE,
//     };
//     gpio_config(&io_conf);
//     // BLE Set Up
//     // nvs_flash_init();
//     // nimble_port_init();
//     // ble_svc_gap_device_name_set("HydraWise-BLE-Server");
//     // ble_svc_gap_init();
//     // ble_svc_gatt_init();
//     // ble_gatts_count_cfg(gatt_svcs);
//     // ble_gatts_add_svcs(gatt_svcs);
//     // ble_hs_cfg.sync_cb = ble_app_on_sync;
//     // nimble_port_freertos_init(host_task);
//     // Note: The above task will send heart rate notifications every 3 seconds
//     // This will allow the ESP32 to send heart rate notifications to connected clients.
//     // The application will now start advertising and waiting for connections.
//     // The notify_heart_rate_task will run in parallel and send notifications to the connected client.
//     // gen_cos_wave();
//     // i2c_scan();
//     // Configure GPIO21 and GPIO22 as outputs
//     // gpio_reset_pin(GPIO_NUM_21);
//     // gpio_reset_pin(GPIO_NUM_22);

//     // gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);
//     // gpio_set_direction(GPIO_NUM_22, GPIO_MODE_OUTPUT);

//     // // Set them HIGH
//     // gpio_set_level(GPIO_NUM_21, 1);
//     // gpio_set_level(GPIO_NUM_22, 1);
//     // xTaskCreate(max30101_task, "max30101_task", 4096, NULL, 5, NULL);
//     while (1) {
//         gpio_set_level(BLINK_GPIO, 1); // LED ON
//         vTaskDelay(pdMS_TO_TICKS(500));
//         gpio_set_level(BLINK_GPIO, 0); // LED OFF
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }

// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "nvs_flash.h"

// #include "nimble/nimble_port.h"
// #include "nimble/nimble_port_freertos.h"
// #include "host/ble_hs.h"
// #include "host/ble_gap.h"
// #include "services/gap/ble_svc_gap.h"
// #include "services/gatt/ble_svc_gatt.h"
// uint8_t ble_addr_type;


// static const char *TAG = "BLE_MAIN";

// void ble_app_advertise(void);

// static void ble_app_on_sync(void) {
//     ESP_LOGI(TAG, "BLE stack initialized.");

    
//     int rc = ble_hs_id_infer_auto(0, &ble_addr_type);  // Pass a valid pointer

//     if (rc != 0) {
//         ESP_LOGE(TAG, "Failed to infer BLE address type: %d", rc);
//         return;
//     }

//     ble_app_advertise();  // continue as usual
// }


// /// GAP event handler (we can expand this for connect/disconnect)
// static int ble_gap_event(struct ble_gap_event *event, void *arg) {
//     switch (event->type) {
//         case BLE_GAP_EVENT_CONNECT:
//             ESP_LOGI(TAG, "BLE GAP: connect event, status=%d", event->connect.status);
//             if (event->connect.status != 0) {
//                 ble_app_advertise();  // Restart advertising
//             }
//             break;
//         case BLE_GAP_EVENT_DISCONNECT:
//             ESP_LOGI(TAG, "BLE GAP: disconnect");
//             ble_app_advertise();
//             break;
//         default:
//             break;
//     }
//     return 0;
// }

// /// Start BLE advertising
// void ble_app_advertise(void) {
//     struct ble_hs_adv_fields fields = {0};
//     const char *name = "HydraWise-BLE-Server";
//     fields.name = (uint8_t *)name;
//     fields.name_len = strlen(name);
//     fields.name_is_complete = 1;

//     static const ble_uuid16_t svc_uuids[] = {
//         BLE_UUID16_INIT(0x180D), // Heart Rate Service
//         BLE_UUID16_INIT(0x180F), // Battery Service
//         BLE_UUID16_INIT(0x181C), // Conductivity Service
//         // BLE_UUID16_INIT(0x180A), // Device Information Service
//         // BLE_UUID16_INIT(0x180C), // Custom Command Control Service
//         BLE_UUID16_INIT(0x180E), // Button Service
//     };
//     fields.uuids16 = (ble_uuid16_t *)svc_uuids;
//     fields.num_uuids16 = sizeof(svc_uuids) / sizeof(svc_uuids[0]);
//     fields.uuids16_is_complete = 1;

//     ble_gap_adv_set_fields(&fields);

//     // Start advertising
//     ESP_LOGI(TAG, "Starting BLE advertisement...");

//     struct ble_gap_adv_params adv_params;
//     memset(&adv_params, 0, sizeof(adv_params));
//     adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
//     adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

//     int adv_rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
//     if (adv_rc != 0) {
//         ESP_LOGE(TAG, "Failed to start advertising: %d", adv_rc);
//     } else {
//         ESP_LOGI(TAG, "Advertising started successfully");
//     }
// }

// void host_task(void *param) {
//     nimble_port_run();  // Runs the NimBLE host
//     nimble_port_freertos_deinit();
// }

// void app_main(void) {
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     nimble_port_init();
//     ble_svc_gap_init();
//     ble_svc_gatt_init();

//     ble_hs_cfg.sync_cb = ble_app_on_sync;

//     nimble_port_freertos_init(host_task);
// }
