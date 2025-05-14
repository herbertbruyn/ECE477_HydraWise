idf_component_register(SRCS "max30101.c" "algorithm.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer)
