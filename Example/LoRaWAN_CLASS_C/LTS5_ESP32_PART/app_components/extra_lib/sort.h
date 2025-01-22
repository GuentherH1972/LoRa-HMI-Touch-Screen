#ifndef _SORT_H
#define _SORT_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDENCIES --------------------------------------------------------- */

#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/ledc.h"

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define  NULL_TYPE          (0x00)
#define  TEM_HUM_TYPE       (0x01)
#define  DOOR_TYPE          (0x02)
#define  WATER_LEAK_TYPE    (0x03)
#define  OCCUPIED_TYPE      (0x04)
#define  BUTTON_TYPE        (0x05)
#define  ALARM_TYPE         (0x06)

#define DOOR_STATUS_OPEN    (0x00)
#define DOOR_STATUS_CLOSE   (0x01)

#define WATER_LEAK_STATUS_NORMAL    (0x00)
#define WATER_LEAK_STATUS_LEAKING   (0x01)

#define OCCUPIED_STATUS_FREE       (0x00)
#define OCCUPIED_STATUS_OCCUPIED   (0x01)

#define BUTTON_STATUS_OFF    (0x00)
#define BUTTON_STATUS_ON     (0x01)

#define ALARM_STATUS_OFF     (0x00)
#define ALARM_STATUS_ALARM   (0x01)

#define ALARM_TIMER_DISABLE  (0x00)
#define ALARM_TIMER_ENABLE   (0x01)

#define ALARM_TIMER_STATUS_OFF  (0x00)
#define ALARM_TIMER_STATUS_ON   (0x01)

#define TIME_SORT_LASTEST   (0x00)
#define TIME_SORT_EARLIEST  (0x01)

#define BATTERY_LEVEL_SORT_HIGHEST   (0x00)
#define BATTERY_LEVEL_SORT_LOWEST    (0x01)

#define BRIGHTNESS_start (30)
#define BRIGHTNESS_max (1020)
#define BRIGHTNESS_STEP ((BRIGHTNESS_max - BRIGHTNESS_start) / 9) 

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- EXTERNAL VARIABLES --------------------------------------------------------- */

extern esp_lcd_panel_handle_t panel_handle_for_avoid_screen_drift;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef struct 
{ 
    lv_obj_t * ui_PanelSensorTemHum;
    lv_obj_t * ui_PanelSensorNodeTemHum;
    lv_obj_t * ui_PanelChartTemHum;
    lv_obj_t * ui_ChartDisplayTemHum;
    lv_obj_t * ui_LabelTypeTemHum;
    lv_obj_t * ui_LabelDevEUITemHum;
    lv_obj_t * ui_LabelNameTemHum;
    lv_obj_t * ui_PanelSignalStrengthTemHum;
    lv_obj_t * ui_SliderSignalStrengthTemHum;
    lv_obj_t * ui_LabelSignalStrengthValueTemHum;
    lv_obj_t * ui_PanelBatteryTemHum;
    lv_obj_t * ui_SliderBatteryTemHum;
    lv_obj_t * ui_LabelBatteryTemHum;
    lv_obj_t * ui_PanelTemTemHum;
    lv_obj_t * ui_LabelTemTemHum;
    lv_obj_t * ui_LabelTemValueTemHum;
    lv_obj_t * ui_PanelHumTemHum;
    lv_obj_t * ui_LabelHumTemHum;
    lv_obj_t * ui_LabelHumValueTemHum;
    lv_obj_t * ui_PanelExtTemHum;
    lv_obj_t * ui_LabelExtTemHum;
    lv_obj_t * ui_LabelExtValueTemHum;
    lv_obj_t * ui_PanelTimePassedTemHum;
    lv_obj_t * ui_ImageTimePassedTemHum;
    lv_obj_t * ui_LabelTimePassedValueTemHum;
    lv_obj_t * ui_PanelTopTemHum;
    lv_obj_t * ui_ImageButtonTopTemHum;
    lv_obj_t * ui_PanelSensorNodeDeleteTemHum;
    lv_obj_t * ui_ImageDeleteTemHum;
} tem_hum_panel;

typedef struct 
{
    lv_obj_t * ui_PanelSensorDoor;
    lv_obj_t * ui_PanelSensorNodeDoor;
    lv_obj_t * ui_PanelChartDoor;
    lv_obj_t * ui_ChartDisplayDoor;
    lv_obj_t * ui_LabelTypeDoor;
    lv_obj_t * ui_LabelDevEUIDoor;
    lv_obj_t * ui_LabelNameDoor;
    lv_obj_t * ui_PanelSignalStrengthDoor;
    lv_obj_t * ui_ImageSignalStrengthDoor;
    lv_obj_t * ui_SliderSignalStrengthDoor;
    lv_obj_t * ui_LabelSignalStrengthValueDoor;
    lv_obj_t * ui_PanelBatteryDoor;
    lv_obj_t * ui_ImageBatteryDoor;
    lv_obj_t * ui_SliderBatteryDoor;
    lv_obj_t * ui_LabelBatteryDoor;
    lv_obj_t * ui_PanelCurrentStatusOpenDoor;
    lv_obj_t * ui_LabelCurrentStatusOpenDoor;
    lv_obj_t * ui_ImageCurrentStatusOpenDoor;
    lv_obj_t * ui_PanelCurrentStatusCloseDoor;
    lv_obj_t * ui_LabelCurrentStatusCloseDoor;
    lv_obj_t * ui_ImageCurrentStatusCloseDoor;
    lv_obj_t * ui_PanelLastDoor;
    lv_obj_t * ui_LabelLastDoor;
    lv_obj_t * ui_LabelLastValueDoor;
    lv_obj_t * ui_PanelTimesDoor;
    lv_obj_t * ui_LabelTimesDoor;
    lv_obj_t * ui_LabelTimesValueDoor;
    lv_obj_t * ui_PanelTimePassedDoor;
    lv_obj_t * ui_ImageTimePassedDoor;
    lv_obj_t * ui_LabelTimePassedValueDoor;
    lv_obj_t * ui_PanelTopDoor;
    lv_obj_t * ui_ImageButtonTopDoor;
    lv_obj_t * ui_PanelSensorNodeDeleteDoor;
    lv_obj_t * ui_ImageDeleteDoor;
} door_panel;

typedef struct 
{
    lv_obj_t * ui_PanelSensorWaterLeak;
    lv_obj_t * ui_PanelSensorNodeWaterLeak;
    lv_obj_t * ui_PanelChartWaterLeak;
    lv_obj_t * ui_ChartDisplayWaterLeak;
    lv_obj_t * ui_LabelTypeWaterLeak;
    lv_obj_t * ui_LabelDevEUIWaterLeak;
    lv_obj_t * ui_LabelNameWaterLeak;
    lv_obj_t * ui_PanelCurrentStatusLeakingWaterLeak;
    lv_obj_t * ui_LabelCurrentStatusLeakingWaterLeak;
    lv_obj_t * ui_ImageCurrentStatusLeakingWaterLeak;
    lv_obj_t * ui_PanelCurrentStatusNormalWaterLeak;
    lv_obj_t * ui_LabelCurrentStatusNormalWaterLeak;
    lv_obj_t * ui_ImageCurrentStatusNormalWaterLeak;
    lv_obj_t * ui_PanelSignalStrengthWaterLeak;
    lv_obj_t * ui_ImageSignalStrengthWaterLeak;
    lv_obj_t * ui_SliderSignalStrengthWaterLeak;
    lv_obj_t * ui_LabelSignalStrengthValueWaterLeak;
    lv_obj_t * ui_PanelBatteryWaterLeak;
    lv_obj_t * ui_ImageBatteryWaterLeak;
    lv_obj_t * ui_SliderBatteryWaterLeak;
    lv_obj_t * ui_LabelBatteryWaterLeak;
    lv_obj_t * ui_PanelLastWaterLeak;
    lv_obj_t * ui_LabelLastWaterLeak;
    lv_obj_t * ui_LabelLastValueWaterLeak;
    lv_obj_t * ui_PanelTimesWaterLeak;
    lv_obj_t * ui_LabelTimesWaterLeak;
    lv_obj_t * ui_LabelTimesValueWaterLeak;
    lv_obj_t * ui_PanelTimePassedWaterLeak;
    lv_obj_t * ui_ImageTimePassedWaterLeak;
    lv_obj_t * ui_LabelTimePassedValueWaterLeak;
    lv_obj_t * ui_PanelTopWaterLeak;
    lv_obj_t * ui_ImageButtonTopWaterLeak;
    lv_obj_t * ui_PanelSensorNodeDeleteWaterLeak;
    lv_obj_t * ui_ImageDeleteWaterLeak;
} water_leak_panel;

typedef struct 
{
    lv_obj_t * ui_PanelSensorOccupied;
    lv_obj_t * ui_PanelSensorNodeOccupied;
    lv_obj_t * ui_PanelChartOccupied;
    lv_obj_t * ui_ChartDisplayOccupied;
    lv_obj_t * ui_LabelTypeOccupied;
    lv_obj_t * ui_LabelDevEUIOccupied;
    lv_obj_t * ui_LabelNameOccupied;
    lv_obj_t * ui_PanelSignalStrengthOccupied;
    lv_obj_t * ui_ImageSignalStrengthOccupied;
    lv_obj_t * ui_SliderSignalStrengthOccupied;
    lv_obj_t * ui_LabelSignalStrengthValueOccupied;
    lv_obj_t * ui_PanelBatteryOccupied;
    lv_obj_t * ui_ImageBatteryOccupied;
    lv_obj_t * ui_SliderBatteryOccupied;
    lv_obj_t * ui_LabelBatteryOccupied;
    lv_obj_t * ui_PanelLabelOccupied;
    lv_obj_t * ui_LabelOccupiedOccupied;
    lv_obj_t * ui_LabelFreeOccupied;
    lv_obj_t * ui_PanelImageOccupied;
    lv_obj_t * ui_ImageOccupiedOccupied;
    lv_obj_t * ui_ImageFreeOccupied;
    lv_obj_t * ui_PanelTimePassedOccupied;
    lv_obj_t * ui_ImageTimePassedOccupied;
    lv_obj_t * ui_LabelTimePassedValueOccupied;
    lv_obj_t * ui_PanelTopOccupied;
    lv_obj_t * ui_ImageButtonTopOccupied;
    lv_obj_t * ui_PanelSensorNodeDeleteOccupied;
    lv_obj_t * ui_ImageDeleteOccupied;
} occupied_panel;

typedef struct 
{
    lv_obj_t * ui_PanelSensorButton;
    lv_obj_t * ui_PanelSensorNodeButton;
    lv_obj_t * ui_PanelChartButton;
    lv_obj_t * ui_ChartDisplayButton;
    lv_obj_t * ui_LabelTypeButton;
    lv_obj_t * ui_LabelDevEUIButton;
    lv_obj_t * ui_LabelNameButton;
    lv_obj_t * ui_PanelSignalStrengthButton;
    lv_obj_t * ui_ImageSignalStrengthButton;
    lv_obj_t * ui_SliderSignalStrengthButton;
    lv_obj_t * ui_LabelSignalStrengthValueButton;
    lv_obj_t * ui_PanelBatteryButton;
    lv_obj_t * ui_ImageBatteryButton;
    lv_obj_t * ui_SliderBatteryButton;
    lv_obj_t * ui_LabelBatteryButton;
    lv_obj_t * ui_PanelButtonButton;
    lv_obj_t * ui_SwitchButtonButton;
    lv_obj_t * ui_LabelButtonONButton;
    lv_obj_t * ui_LabelButtonOFFButton;
    lv_obj_t * ui_PanelTimePassedButton;
    lv_obj_t * ui_ImageTimePassedButton;
    lv_obj_t * ui_LabelTimePassedValueButton;
    lv_obj_t * ui_PanelTopButton;
    lv_obj_t * ui_ImageButtonTopButton;
    lv_obj_t * ui_PanelSensorNodeDeleteButton;
    lv_obj_t * ui_ImageDeleteButton;

    bool button_press_flag;
    bool button_status;
} button_panel;

typedef struct 
{
    lv_obj_t * ui_PanelSensorAlarm;
    lv_obj_t * ui_PanelSensorNodeAlarm;
    lv_obj_t * ui_PanelChartAlarm;
    lv_obj_t * ui_ChartDisplayAlarm;
    lv_obj_t * ui_LabelTypeAlarm;
    lv_obj_t * ui_LabelDevEUIAlarm;
    lv_obj_t * ui_LabelNameAlarm;
    lv_obj_t * ui_PanelSignalStrengthAlarm;
    lv_obj_t * ui_ImageSignalStrengthAlarm;
    lv_obj_t * ui_SliderSignalStrengthAlarm;
    lv_obj_t * ui_LabelSignalStrengthValueAlarm;
    lv_obj_t * ui_PanelBatteryAlarm;
    lv_obj_t * ui_ImageBatteryAlarm;
    lv_obj_t * ui_SliderBatteryAlarm;
    lv_obj_t * ui_LabelBatteryAlarm;
    lv_obj_t * ui_PanelAlarmAlarm;
    lv_obj_t * ui_ImageAlarmAlarm;
    lv_obj_t * ui_PanelTimePassedAlarm;
    lv_obj_t * ui_ImageTimePassedAlarm;
    lv_obj_t * ui_LabelTimePassedValueAlarm;
    lv_obj_t * ui_PanelTopAlarm;
    lv_obj_t * ui_ImageButtonTopAlarm;
    lv_obj_t * ui_PanelSensorNodeDeleteAlarm;
    lv_obj_t * ui_ImageDeleteAlarm;

    uint8_t timer_alarm_en_flag;
    uint8_t timer_exec_status;

} alarm_panel;

typedef struct 
{
    uint8_t * dev_name;
    uint8_t * dev_eui;
    uint8_t   mod;

    uint8_t * temp1; 
    uint8_t * temp2;
    uint8_t * humidity; 

    uint8_t * status;

    uint8_t * bat;
    int8_t    data_rssi;
} message;

typedef struct {
    union {
        tem_hum_panel       tem_hum;
        door_panel          door;
        water_leak_panel    water_leak;
        occupied_panel      occupied;
        button_panel        button;
        alarm_panel         alarm;
    } panel_union;
    uint8_t panel_type;
} panel_with_type;

typedef struct 
{
    panel_with_type panel_obj;                // panel_obj是LVGL的一个传感器面板
	uint8_t         panel_obj_index;          // 索引代表元素的实际位置
    uint8_t         top_flag;                 // 这是置顶标志，用于表示传感器是否被置顶
    time_t          sec_arrive;               // 记录开始时间，即消息的到来时间
} panel_all;

typedef struct 
{
    panel_all * array;  
    size_t      size;  
    size_t      capacity;  
} DynamicArray;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

uint16_t x_by_index(uint16_t index);
uint16_t y_by_index(uint16_t index);
time_t get_sec(void);
void timer_callback(lv_timer_t * timer);
void timer_alarm_callback(lv_timer_t * timer);
void config_save(int32_t brightness, uint16_t tem_unit, uint16_t boot_screen);
void initArray(DynamicArray *arr, size_t initialCapacity);
void freeArray(DynamicArray *arr);
void deleteElement(DynamicArray *arr, size_t index);
panel_all * find_upper_by_SensorPanel(DynamicArray *arr, lv_obj_t * PanelSensor);
void reposArray1(DynamicArray *arr, lv_obj_t * PanelSensor);
void reposArray2(DynamicArray *arr, lv_obj_t * PanelSensor);
void reposArray_by_time(DynamicArray *arr, uint8_t sort_type);
void reposArray_by_battery(DynamicArray *arr, uint8_t sort_type);
void panel_index_restore(DynamicArray *arr);

void time_sort_value_set(uint16_t new_value);
uint16_t time_sort_value_get(void);
void battery_sort_value_set(uint16_t new_value);
uint16_t battery_sort_value_get(void);
void nvs_store_complete_flag_set(bool new_value);
bool nvs_store_complete_flag_get(void);
void save_button_press_flag_set(bool new_value);
bool save_button_press_flag_get(void);
uint8_t button_uplink_num_get(void);
void button_uplink_num_set(uint8_t button_uplink_num_temp);
DynamicArray * arr_get(void);
void system_begin_sec_set(time_t new_value);
time_t system_begin_sec_get(void);
void lvgl_lock_get(void);
void lvgl_lock_release(void);

panel_with_type create_tem_hum(uint8_t index, lv_obj_t *parent); 
panel_with_type create_door(uint8_t index, lv_obj_t *parent);
panel_with_type create_water_leak(uint8_t index, lv_obj_t *parent);
panel_with_type create_occupied(uint8_t index, lv_obj_t *parent);
panel_with_type create_button(uint8_t index, lv_obj_t *parent);
panel_with_type create_alarm(uint8_t index, lv_obj_t *parent);

void panel_update(message * mes, DynamicArray * arr, lv_obj_t * parent);

void sort_init(void);

#endif // _SORT_H

/* --- EOF ------------------------------------------------------------------ */
