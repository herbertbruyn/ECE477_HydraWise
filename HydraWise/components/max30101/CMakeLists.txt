idf_component_register(SRCS "max30101.c" "heartRate.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer)
