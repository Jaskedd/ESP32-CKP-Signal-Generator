idf_component_register(SRCS "main.c" "menu.c"
                    PRIV_REQUIRES driver esp_adc esp_driver_gptimer hal
                    INCLUDE_DIRS "")
