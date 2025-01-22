SET(SOURCES styles/ui_occupied_style.c
    ui_occupied.c
    ui_occupied_events.c
    images/ui_img_signal_empty_png.c
    images/ui_img_signal_full_png.c
    images/ui_img_battery_empty_png.c
    images/ui_img_battery_full_png.c
    images/ui_img_arc_red_png.c
    images/ui_img_arc_green_png.c
    images/ui_img_last_seen_png.c
    images/ui_img_set_top_png.c
    images/ui_img_cancel_set_top_png.c
    images/ui_img_delete_png.c
    ui_temporary_image.c
    fonts/ui_font_Font12.c
    fonts/ui_font_Font16.c)

add_library(ui_occupied ${SOURCES})
