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
#include "sdkconfig.h"

// Define device
char *TAG = "HydraWise BLE";
uint8_t ble_addr_type;
void ble_app_advertise(void);


void app_main() {
    nvs_flash_init();
    esp_nimble_hci_and_controller_init();
    nimble_port_init(NULL);
    
}