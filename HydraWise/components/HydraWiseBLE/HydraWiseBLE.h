#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#include <stdint.h>
#include "host/ble_gatt.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_random.h"
#include "sdkconfig.h"

// ---------------- Constants & Macros ----------------

#define CONFIG_IDF_TARGET_ESP32 1

// ---------------- Global Variables ----------------

extern uint8_t ble_addr_type;
extern uint8_t button_state;

extern uint16_t hrm_handle;
extern uint16_t conductivity_handle;
extern uint16_t button_char_handle;
extern uint16_t volatile conn_handle_global;

// ---------------- Function Declarations ----------------

// Advertising
void ble_app_advertise(void);

// Event handling
void ble_app_on_sync(void);
void host_task(void *param);

// Tasks
void notify_heart_rate_task(void *param);

// Access Callbacks
int device_read(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

// Declare only
extern const struct ble_gatt_chr_def heart_rate_chr[];
extern const ble_uuid128_t conductivity_uuid;
extern struct ble_gatt_chr_def conductivity_chr[];
extern const struct ble_gatt_chr_def battery_level_chr[];
extern const struct ble_gatt_chr_def button_chr[];
extern const struct ble_gatt_svc_def gatt_svcs[];


#endif // BLE_SERVER_H
