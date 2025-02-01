idf_component_register(SRCS "wavetable_manager.c" "blink_example_main.c"
                       INCLUDE_DIRS "." 
                       REQUIRES driver spiffs  nvs_flash fatfs sdmmc esp_timer tinyusb
                       )





