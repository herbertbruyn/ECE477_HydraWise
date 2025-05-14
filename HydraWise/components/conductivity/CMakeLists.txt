idf_component_register(SRCS "dac.c" "adc.c" "conductivity.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_rom esp_adc esp_timer bt HydraWiseBLE)
