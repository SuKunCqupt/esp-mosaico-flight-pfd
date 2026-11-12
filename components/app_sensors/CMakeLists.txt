idf_component_register(SRCS "sensors_esp.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES app_fusion esp_board_manager bmi270 bmm150 nvs_flash esp_timer esp_driver_i2c
                  esp_rom freertos log heap)
