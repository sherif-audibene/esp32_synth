idf_component_register(SRCS "blink_example_main.c"
                       INCLUDE_DIRS "." 
                       REQUIRES driver spiffs  nvs_flash fatfs sdmmc esp_timer tinyusb
                       )





