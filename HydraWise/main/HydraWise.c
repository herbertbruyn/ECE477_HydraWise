#include <stdio.h>
#include "max30101.h"

void app_main(void) {
    i2c_scan();
    xTaskCreate(max30101_task, "max30101_task", 4096, NULL, 5, NULL);
}