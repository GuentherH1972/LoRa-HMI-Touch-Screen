cmake_minimum_required(VERSION 3.12.4)

idf_component_register(
    SRC_DIRS
        "."
        "ui_tem_hum"
        "ui_tem_hum/fonts"
        "ui_tem_hum/images"
        "ui_tem_hum/styles"
        "ui_door"
        "ui_door/fonts"
        "ui_door/images"
        "ui_door/styles"
        "ui_water_leak"
        "ui_water_leak/fonts"
        "ui_water_leak/images"
        "ui_water_leak/styles"
        "ui_occupied"
        "ui_occupied/fonts"
        "ui_occupied/images"
        "ui_occupied/styles"
        "ui_button"
        "ui_button/fonts"
        "ui_button/images"
        "ui_button/styles"
        "ui_alarm"
        "ui_alarm/fonts"
        "ui_alarm/images"
        "ui_alarm/styles"

    INCLUDE_DIRS
        "."
        "ui_tem_hum"
        "ui_tem_hum/fonts"
        "ui_tem_hum/images"
        "ui_tem_hum/styles"
        "ui_door"
        "ui_door/fonts"
        "ui_door/images"
        "ui_door/styles"
        "ui_water_leak"
        "ui_water_leak/fonts"
        "ui_water_leak/images"
        "ui_water_leak/styles"
        "ui_occupied"
        "ui_occupied/fonts"
        "ui_occupied/images"
        "ui_occupied/styles"
        "ui_button"
        "ui_button/fonts"
        "ui_button/images"
        "ui_button/styles"
        "ui_alarm"
        "ui_alarm/fonts"
        "ui_alarm/images"
        "ui_alarm/styles"
        
    REQUIRES
        lvgl
	    ui
        nvs_flash
        # freertos
        # esp_driver_ledc
        esp_lcd
)
