SET(SOURCES screens/ui_ScreenBoot.c
    screens/ui_ScreenMain.c
    screens/ui_ScreenSetting.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_product_1_png.c
    images/ui_img_product_2_png.c
    images/ui_img_product_3_png.c
    images/ui_img_product_4_png.c
    images/ui_img_product_5_png.c
    images/ui_img_product_6_png.c
    images/ui_img_boot_logo_png.c
    images/ui_img_home_page_png.c
    images/ui_img_setting_png.c
    images/ui_img_logo_png.c
    images/ui_img_profile_png.c
    images/ui_img_last_seen_png.c
    images/ui_img_battery_png.c
    fonts/ui_font_Font12.c
    fonts/ui_font_Font16.c)

add_library(ui ${SOURCES})
