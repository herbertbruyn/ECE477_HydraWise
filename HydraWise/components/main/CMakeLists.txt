idf_component_register(SRCS "HydraWise.c"
                       INCLUDE_DIRS "../components/max30101" "../components/conductivity" "../components/HydraWiseBLE" "../components/BMS"
                       REQUIRES max30101 conductivity HydraWiseBLE BMS bt freertos esp_event nvs_flash)
