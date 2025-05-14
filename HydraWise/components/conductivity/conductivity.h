#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "HydraWiseBLE.h"
#include "esp_timer.h"
#include "esp_check.h"
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/semphr.h"

void gen_cos_wave(void);
void stop_dac();
void adc(void);
void trigger_conductivity_measurement_task(void *param);

