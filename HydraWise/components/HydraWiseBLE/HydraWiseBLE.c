#include "HydraWiseBLE.h"
#include "conductivity.h"
#include "max30101.h"
#include "bms.h"

// Define device
char *TAG = "HydraWise-BLE-Server";
#define CONFIG_IDF_TARGET_ESP32 1
uint8_t ble_addr_type;
volatile uint16_t conn_handle_global = 0; // Global connection handle to track the current connection
uint16_t hrm_handle = 0; // Handle for Heart Rate Measurement characteristic
uint16_t conductivity_handle = 0; // Handle for Conductivity characteristic
uint16_t battery_handle = 0;
uint8_t button_state = 0; // 0 = STOPPED, 1 = STARTED
uint16_t button_char_handle = 0; // Handle for button characteristic
void ble_app_advertise(void);

/*
-------------------------------------------

ESP 32 FUNCTIONALITIES

---------------------------------------------
1. Device Name: HydraWise-BLE-Server
2. Services:
   - Heart Rate Service
   - Conductivity Service
   - Battery Level Service
   - Device Information Service
3. Characteristics:
    - Heart Rate Measurement (Notify)
    - Conductivity Measurement (Notify)
    - Battery Level (Notify)
    - Device Name (Read/Write)
    - Device Information (Read/Write)
    - Custom Commands (e.g., "LIGHT ON", "LIGHT OFF")
4. Access Control:
    - Heart Rate Measurement: Read and Notify
    - Conductivity Measurement: Read and Notify
    - Battery Level: Read and Notify
    - Device Name: Read and Write
    - Device Information: Read and Write
    - Custom Commands: Write only (to control external devices)
5. Connection Handling:
    - Handle connection and disconnection events
    - Retry advertising on disconnection
    - Store connection handle for further communication
6. Button State:
    - Button state is used to control the device's operation (START/STOP)
    - The button state is used to control the data collection process
7. FreeRTOS:
    - Use FreeRTOS for task management
    - Create tasks for heart rate and conductivity notifications
---------------------------------------------
*/
// Write data to ESP32 defined as server
int device_write(uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt, void *arg) {
    printf("Received WRITE (handle: %d, conn: %d)\n", attr_handle, conn_handle);

    // Print raw bytes
    printf("Raw data: ");
    for (int i = 0; i < ctxt->om->om_len; i++) {
        printf("%02x ", ctxt->om->om_data[i]);
    }
    printf("\n");

    // Convert to string (with null terminator)
    char buf[ctxt->om->om_len + 1];
    memcpy(buf, ctxt->om->om_data, ctxt->om->om_len);
    buf[ctxt->om->om_len] = '\0';

    ESP_LOGI(TAG, "Command received: '%s'", buf);

    // You can add parsing logic here
    if (strcmp(buf, "START") == 0) {
        button_state = 1;
        printf("Starting...\n");
        ESP_LOGI(TAG, "START command received. Notifying button state");
    
        // Start conductivity data collection
        ESP_LOGI(TAG, "Triggering DAC + ADC for 10 seconds...");
        xTaskCreate(trigger_conductivity_measurement_task, "conductivity_task", 4096, NULL, 5, NULL);
    } else if (strcmp(buf, "STOP") == 0) {
        button_state = 0;
        printf("Stopping...\n");
        ESP_LOGI(TAG, "STOP command received. Notifying button state.");
    }
    
    // notify client about button state change
    if (ble_gap_conn_find(conn_handle_global, NULL) == 0 && button_char_handle != 0) {
        ESP_LOGI(TAG, "🔔 Notifying client with button_state = %d", button_state);
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&button_state, sizeof(button_state));
        int rc = ble_gattc_notify_custom(conn_handle_global, button_char_handle, om);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to send button state notification: %d", rc);
        } else {
            ESP_LOGI(TAG, "Button state notification sent: %d", button_state);
        }
    }
    return 0;
}

// extravagant read data from ESP32 defined as a server
int device_read(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (attr_handle == hrm_handle) {
        ESP_LOGI(TAG, "💓 Client is reading Heart Rate characteristic");
        int dummy_hr = 75; // send as a 4-byte integer
        os_mbuf_append(ctxt->om, &dummy_hr, sizeof(dummy_hr));
    }
    else if (attr_handle == conductivity_handle) {
        ESP_LOGI(TAG, "💧 Client is reading Conductivity characteristic");
        float dummy_conductivity = 1.23f;
        os_mbuf_append(ctxt->om, &dummy_conductivity, sizeof(dummy_conductivity));
    }
    else if (attr_handle == button_char_handle) {
        ESP_LOGI(TAG, "📥 Client is reading Button state characteristic");
        os_mbuf_append(ctxt->om, &button_state, sizeof(button_state));
        // tell me what the button state is
        ESP_LOGI(TAG, "Button state: %d", button_state);
    }
    else if (attr_handle == battery_handle) {
        uint8_t battery_level = 88;
        os_mbuf_append(ctxt->om, &battery_level, sizeof(battery_level));
    }    
    else {
        ESP_LOGW(TAG, "⚠️ Unknown characteristic read (handle: %d)", attr_handle);
        os_mbuf_append(ctxt->om, "Unknown", 7);
    }

    return 0;
}

// heart rate characteristic
const struct ble_gatt_chr_def heart_rate_chr[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x2A37), // HEART RATE MEASUREMENT
        .access_cb = device_read,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    },
    {
        0, // NULL TERMINATOR
    }
};

struct ble_gatt_chr_def conductivity_chr[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x272B), // Hydration MEASUREMENT
        .access_cb = device_read,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    },
    {
        0
    },  // Terminator
};

// battery level characteristic
const struct ble_gatt_chr_def battery_level_chr[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x2A19), // BATTERY LEVEL
        .access_cb = device_read,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    },
    {
        0, // NULL TERMINATOR
    }
};

// button characteristic
const struct ble_gatt_chr_def button_chr[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x2A3F), // button
        .access_cb = device_write,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_WRITE,
    },
    {
        0, // NULL TERMINATOR
    }
};

// heart rate notification task with FreeRTOS
void notify_heart_rate_task(void *param) {
    while (1) {
        if (ble_gap_conn_find(conn_handle_global, NULL) == 0) {
            int hr_to_send = get_average_hr_last_10s(); // Calculated average heart rate data
            printf("HEART RATE: %d\n", hr_to_send);
            struct os_mbuf *hr_om = ble_hs_mbuf_from_flat(&hr_to_send, sizeof(hr_to_send));
            int rc = ble_gattc_notify_custom(conn_handle_global, hrm_handle, hr_om);
            if (rc != 0) {
                ESP_LOGE(TAG, "❌ Failed to send heart rate notification: %d", rc);
            } else {
                ESP_LOGI(TAG, "💓 Heart rate notification sent: %d BPM", hr_to_send);
            }
        } else {
            ESP_LOGI(TAG, "⏸ Heart rate task paused - not connected.");
        }

        vTaskDelay(pdMS_TO_TICKS(10000)); // Every 10 seconds
    }
}

void notify_battery_task(void *param) {
    while (1) {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(conn_handle_global, &desc) == 0) {
            // int bat_level_send = battery_level;
            int bat_level_send = 88;  // % battery
            struct os_mbuf *batt_om = ble_hs_mbuf_from_flat(&bat_level_send, sizeof(bat_level_send));
            int rc = ble_gattc_notify_custom(conn_handle_global, battery_handle, batt_om);
            if (rc != 0) {
                ESP_LOGE(TAG, "❌ Failed to send battery notification: %d", rc);
            } else {
                ESP_LOGI(TAG, "🔋 Battery level notification sent: %d%%", bat_level_send);
            }
        } else {
            ESP_LOGI(TAG, "⏸ Battery task paused - not connected.");
        }

        vTaskDelay(pdMS_TO_TICKS(10000)); // Every 10 seconds
    }
}


// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
const struct ble_gatt_svc_def gatt_svcs[] = {
    // button characteristic service for on/off in collecting data
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180E), // Use alert notification service (repurposed to act as button alert state)
        .characteristics = button_chr,  // Characteristic: random uuid
    },

    // Battery Service (0x180F)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180F), // Service UUID: Battery Service
        .characteristics = battery_level_chr, // Characteristic: 0x2A19
    },

    // Heart Rate Service (0x180D)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180D), // Service UUID: Heart Rate Service
        .characteristics = heart_rate_chr, // Characteristic: 0x2A37
    },

    // Conductivity Service (custom UUID)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x181C), // Service UUID: Custom Conductivity
        .characteristics = conductivity_chr, // Characteristic: 128-bit UUID
    },

    // Device Information Service (0x180A)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A), // Service UUID: Device Info
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A29), // Characteristic: Manufacturer Name
                .access_cb = device_read,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A24), // Characteristic: Model Number
                .access_cb = device_read,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 } // Terminator
        }
    },

    // Custom Command Control Service (0x180C)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180C), // Custom Service
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A00), // Characteristic: Device Name Write
                .access_cb = device_write,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            { 0 } // Terminator
        }
    },
    { 0 } // End of services
};


// BLE event handling
int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event -> type)
    {
        // Advertise if connected
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI("GAP", "✅ Device connected successfully");
                conn_handle_global = 1; // Set to 1 to indicate connected state

                // DOUBLE CHECK VALUES
                ESP_LOGI("GAP", "event->connect.conn_handle = %d", event->connect.conn_handle);
                conn_handle_global = event->connect.conn_handle;
                ESP_LOGI("GAP", "✅ conn_handle_global set to %d", conn_handle_global);

                // Start tasks once
                static bool tasks_started = false;
                if (!tasks_started) {
                    tasks_started = true;
                    xTaskCreate(max30101_task, "max30101_task", 4096, NULL, 5, NULL);
                    xTaskCreate(notify_heart_rate_task, "hr_notify_task", 4096, NULL, 5, NULL);
                    // xTaskCreate(battery_soc_task, "battery_soc_task", 4096, NULL, 10, NULL);
                    xTaskCreate(notify_battery_task, "battery_notify_task", 4096, NULL, 5, NULL); // Integrate actual BMS
                    ESP_LOGI("GAP", "✅ Notification tasks started");
                }
            } else {
                ESP_LOGE("GAP", "❌ Connection failed with status: %d", event->connect.status);
                ble_app_advertise(); // restart advertising
            }
            break;

        // advertise again after completion of event
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECT");
            conn_handle_global = 0;  // Reset connection handle
            ble_app_advertise();     // Restart advertising
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI("GAP", "BLE GAP EVENT ADV COMPLETE");
            ble_app_advertise();
            break;
        default:
            break;
    }
    return 0;          
}

// for apple
void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    // Include flags for general discoverability and BLE-only support
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Include complete device name
    const char *device_name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    // Include the Heart Rate Service UUID and battery in the advertisement
    // const uint16_t svc_uuid = 0x180D;
    static const ble_uuid16_t svc_uuids[] = {
        BLE_UUID16_INIT(0x180D), // Heart Rate Service
        BLE_UUID16_INIT(0x180F), // Battery Service
        BLE_UUID16_INIT(0x181C), // Conductivity Service
        // BLE_UUID16_INIT(0x180A), // Device Information Service
        // BLE_UUID16_INIT(0x180C), // Custom Command Control Service
        BLE_UUID16_INIT(0x180E), // Button Service
    };
    fields.uuids16 = (ble_uuid16_t *)svc_uuids;
    fields.num_uuids16 = sizeof(svc_uuids) / sizeof(svc_uuids[0]);
    fields.uuids16_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    // Start advertising
    ESP_LOGI(TAG, "Starting BLE advertisement...");

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    int adv_rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (adv_rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", adv_rc);
    } else {
        ESP_LOGI(TAG, "Advertising started successfully");
    }
}


// The application
void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
    ESP_LOGI(TAG, "Attempting to locate Heart Rate Characteristic UUID: 0x2A37 in Service UUID: 0x180D");

    uint16_t def_handle;
    uint16_t val_handle;

    int rc = ble_gatts_find_chr(
        BLE_UUID16_DECLARE(0x180D),     // heart rate service
        BLE_UUID16_DECLARE(0x2A37),     // heart rate characteristic
        &def_handle,
        &val_handle
    );

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to find Heart Rate Measurement characteristic: %d", rc);
    } else {
        hrm_handle = val_handle;
        ESP_LOGI(TAG, "Heart Rate Measurement characteristic handle: %d", hrm_handle);
    }

    // conductivity handle
    ESP_LOGI(TAG, "Attempting to locate Conductivity Characteristic UUID: 0xAA5B9750C9824CE690C754C0C8C6AE84 in Service UUID: 0x181C");

    rc = ble_gatts_find_chr(
        BLE_UUID16_DECLARE(0x181C),     // hydration service
        BLE_UUID16_DECLARE(0x272B),     // hydration characteristic
        &def_handle,
        &val_handle
    );

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to find Conductivity characteristic: %d", rc);
    } else {
        conductivity_handle = val_handle;
        ESP_LOGI(TAG, "Conductivity characteristic handle: %d", conductivity_handle);
    }

    // button handle
    ESP_LOGI(TAG, "Locating Button Characteristic UUID: random in Service UUID: 0x180E");
    rc = ble_gatts_find_chr(
        BLE_UUID16_DECLARE(0x180E),
        BLE_UUID16_DECLARE(0x2A3F), // button
        &def_handle,
        &val_handle
    );

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to find Button characteristic: %d", rc);
    } else {
        button_char_handle = val_handle;
        ESP_LOGI(TAG, "Button characteristic handle: %d", button_char_handle);
    }

    // battery handle
    ESP_LOGI(TAG, "Locating Battery Characteristic UUID: 0x2A19 in Service UUID: 0x180F");

    rc = ble_gatts_find_chr(
        BLE_UUID16_DECLARE(0x180F),
        BLE_UUID16_DECLARE(0x2A19), 
        &def_handle,
        &val_handle
    );

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to find Battery characteristic: %d", rc);
    } else {
        battery_handle = val_handle;
        ESP_LOGI(TAG, "Battery characteristic handle: %d", battery_handle);
    }

}

// the inifinite task
void host_task(void *param) {
    nimble_port_run();
}
