idf_component_register(SRCS "HydraWiseBLE.c"
                       INCLUDE_DIRS "."
                       REQUIRES bt freertos esp_event nvs_flash conductivity max30101 BMS)
