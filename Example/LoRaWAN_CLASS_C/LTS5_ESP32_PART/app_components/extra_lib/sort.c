/* -------------------------------------------------------------------------- */
/* --- DEPENDENCIES --------------------------------------------------------- */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"

#include "lv_obj.h"
#include "lv_color.h"
#include "lv_timer.h"
#include "lvgl.h"

#include "sort.h"
#include "ui.h"

#include "ui_tem_hum.h"
#include "ui_door.h"
#include "ui_water_leak.h"
#include "ui_occupied.h"
#include "ui_button.h"
#include "ui_alarm.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define  PANEL_START_X            (-249)
#define  PANEL_START_Y            (-90)
#define  PANEL_COL_NUM            (4)
#define  PANEL_X_DISTANCE         (8)
#define  PANEL_Y_DISTANCE         (8)
#define  PANEL_WIDTH              (156)
#define  PANEL_HEIGHT             (160)
#define  MAX_PANEL_NUM            (147)
#define  INIT_PANEL_CAPACITY      (12)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

static const char * TAG_SORT = "sort";

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static DynamicArray arr;
static time_t sec_now = 0;
static time_t system_begin_sec;
static lv_timer_t * timer_lvgl;
static lv_timer_t * timer_alarm_lvgl;

static uint8_t panel_index_store_temp[MAX_PANEL_NUM] = {0};
static uint16_t time_sort_value = 0;
static uint16_t battery_sort_value = 0;

static bool nvs_store_complete_flag = true;
static bool save_button_press_flag = false; // true: "save" button is pressed
static uint8_t button_uplink_num = 0;

static SemaphoreHandle_t Mutex;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void panel_index_store(DynamicArray *arr);
static void repos_by_index(DynamicArray *arr);
static void tem_unit_convert(uint8_t type, DynamicArray * arr);
static void resizeArray(DynamicArray *arr, size_t newCapacity);
static void addElement(DynamicArray *arr, uint16_t type, lv_obj_t * parent);
static uint16_t find_physical_index(DynamicArray *arr, uint16_t index);
static uint16_t count_top_num(DynamicArray *arr);
static long get_battery_text_value(panel_with_type * panel);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static void repos_by_index(DynamicArray *arr) 
{
    for(uint16_t i = 0;i < arr->size;i++) 
    {
        switch(arr->array[i].panel_obj.panel_type) 
        {
            case TEM_HUM_TYPE:
                lv_obj_set_x(arr->array[i].panel_obj.panel_union.tem_hum.ui_PanelSensorTemHum, x_by_index(arr->array[i].panel_obj_index));
                lv_obj_set_y(arr->array[i].panel_obj.panel_union.tem_hum.ui_PanelSensorTemHum, y_by_index(arr->array[i].panel_obj_index));
                break;
            case DOOR_TYPE:
                lv_obj_set_x(arr->array[i].panel_obj.panel_union.door.ui_PanelSensorDoor, x_by_index(arr->array[i].panel_obj_index));
                lv_obj_set_y(arr->array[i].panel_obj.panel_union.door.ui_PanelSensorDoor, y_by_index(arr->array[i].panel_obj_index));
                break;
            case WATER_LEAK_TYPE:
                lv_obj_set_x(arr->array[i].panel_obj.panel_union.water_leak.ui_PanelSensorWaterLeak, x_by_index(arr->array[i].panel_obj_index));
                lv_obj_set_y(arr->array[i].panel_obj.panel_union.water_leak.ui_PanelSensorWaterLeak, y_by_index(arr->array[i].panel_obj_index));
                break;
            case OCCUPIED_TYPE:
                lv_obj_set_x(arr->array[i].panel_obj.panel_union.occupied.ui_PanelSensorOccupied, x_by_index(arr->array[i].panel_obj_index));
                lv_obj_set_y(arr->array[i].panel_obj.panel_union.occupied.ui_PanelSensorOccupied, y_by_index(arr->array[i].panel_obj_index));
                break;
            case BUTTON_TYPE:
                lv_obj_set_x(arr->array[i].panel_obj.panel_union.button.ui_PanelSensorButton, x_by_index(arr->array[i].panel_obj_index));
                lv_obj_set_y(arr->array[i].panel_obj.panel_union.button.ui_PanelSensorButton, y_by_index(arr->array[i].panel_obj_index));
                break;
            case ALARM_TYPE:
                lv_obj_set_x(arr->array[i].panel_obj.panel_union.alarm.ui_PanelSensorAlarm, x_by_index(arr->array[i].panel_obj_index));
                lv_obj_set_y(arr->array[i].panel_obj.panel_union.alarm.ui_PanelSensorAlarm, y_by_index(arr->array[i].panel_obj_index));
                break;
        }
    }
}

/**
 * convert temperature unit
 *
 * @param type 1: built-in temperature sensor;
 *             2: external temperature sensor;
 * @param arr panel array pointer
 * @return void
 */
static void tem_unit_convert(uint8_t type, DynamicArray * arr)
{
	char * str = NULL;
	char * endptr = NULL;
	double tem_num = 0.0, tem_num_C = 0.0, tem_num_F = 0.0;
	char temp[20] = {'\0'};
	memset(temp, '\0', sizeof(temp));
	uint16_t option_index = lv_dropdown_get_selected(ui_DropdownTemUnit);
	for(uint16_t i = 0;i<arr->size;i++) 
    {
		if(arr->array[i].panel_obj.panel_type == TEM_HUM_TYPE) 
        {
			if(type == 1)
            {
                str = lv_label_get_text(arr->array[i].panel_obj.panel_union.tem_hum.ui_LabelTemValueTemHum);
            }
			else if(type == 2)
            {
                str = lv_label_get_text(arr->array[i].panel_obj.panel_union.tem_hum.ui_LabelExtValueTemHum);
            }

			tem_num = strtod(str, &endptr);
			if(strcmp(endptr, "℃") == 0) 
            {
				tem_num_C = tem_num;
				tem_num_F = tem_num * 1.8 + 32;
			}
				
			else if(strcmp(endptr, "℉") == 0) 
            {
				tem_num_C = (tem_num - (double)32) / 1.8;
				tem_num_F = tem_num;
			}

			if(option_index == 0) 
            {
				snprintf(temp, sizeof(temp), "%.2lf", tem_num_C);
				strcat(temp, "℃");
			} 
            else if(option_index == 1) 
            {
				snprintf(temp, sizeof(temp), "%.2lf", tem_num_F);
				strcat(temp, "℉");
			}

			if(type == 1)
            {
                lv_label_set_text(arr->array[i].panel_obj.panel_union.tem_hum.ui_LabelTemValueTemHum, temp);
            }
				
			else if(type == 2)
            {
                lv_label_set_text(arr->array[i].panel_obj.panel_union.tem_hum.ui_LabelExtValueTemHum, temp);
            }

			str = NULL;
			endptr = NULL;
			tem_num = 0.0;
			memset(temp, '\0', sizeof(temp));
		}
	}
}

// Dynamically increase array capacity  
static void resizeArray(DynamicArray *arr, size_t newCapacity) 
{
    panel_all *newArray = (panel_all *)realloc(arr->array, newCapacity * sizeof(panel_all));  
    if(newArray == NULL) 
    {
        printf("Memory allocation failed\r\n");
        exit(EXIT_FAILURE);  
    }  
    arr->array = newArray;  
    arr->capacity = newCapacity;  
}  


/**
 * add a new panel to array
 *
 * @param arr panel array pointer
 * @param type type of the panel to be added
 * @param parent container LVGL obj of the panel to be added
 * @return void
 * @note This function do not check input panel type is vaild or not. The check work should be done outside.
 */
static void addElement(DynamicArray *arr, uint16_t type, lv_obj_t *parent) 
{
    printf("\r\n********************************************\r\n");
    printf("start to add an element\r\n");

    if(arr->size == MAX_PANEL_NUM) 
    {
        ESP_LOGI(TAG_SORT, "current arr size is %d, execeed maximum limitation, so can not add any more", arr->size);
        return;
    }

    if(arr->size >= arr->capacity) 
    {
        ESP_LOGI(TAG_SORT, "arr size:%d >= arr capacity:%d", arr->size, arr->capacity);
        size_t newCapacity = arr->capacity * 2;
        if(newCapacity > MAX_PANEL_NUM) 
        {
            newCapacity = MAX_PANEL_NUM;
        }  
        resizeArray(arr, newCapacity);  
    }

    memset((arr->array + arr->size), 0, sizeof(panel_all));

    uint16_t cnt_top = count_top_num(arr);
    printf("Number of top pinned panels is: %d\r\n", cnt_top);

    arr->array[arr->size].panel_obj_index = cnt_top;
    for(uint16_t i = 0;i < arr->size;i++) 
    {
        if(arr->array[i].panel_obj_index >= cnt_top) 
        {
            arr->array[i].panel_obj_index++;
        }
    }
    
    switch(type) 
    {
        case TEM_HUM_TYPE: 
            printf("Input panel type is %d, create a TEM_HUM_TYPE panel\r\n", type);
            arr->array[arr->size].panel_obj = create_tem_hum(arr->array[arr->size].panel_obj_index, parent);
            arr->size++;
            break;
        case DOOR_TYPE:
            printf("Input panel type is %d, create a DOOR_TYPE panel\r\n", type);
            arr->array[arr->size].panel_obj = create_door(arr->array[arr->size].panel_obj_index, parent);
            arr->size++;
            break;
        case WATER_LEAK_TYPE:
            printf("Input panel type is %d, create a WATER_LEAK_TYPE panel\r\n", type);
            arr->array[arr->size].panel_obj = create_water_leak(arr->array[arr->size].panel_obj_index, parent);
            arr->size++;
            break;
        case OCCUPIED_TYPE:
            printf("Input panel type is %d, create a OCCUPIED_TYPE panel\r\n", type);
            arr->array[arr->size].panel_obj = create_occupied(arr->array[arr->size].panel_obj_index, parent);
            arr->size++;
            break;
        case BUTTON_TYPE:
            printf("Input panel type is %d, create a BUTTON_TYPE panel\r\n", type);
            arr->array[arr->size].panel_obj = create_button(arr->array[arr->size].panel_obj_index, parent);
            arr->size++;
            break;
        case ALARM_TYPE:
            printf("Input panel type is %d, create a ALARM_TYPE panel\r\n", type);
            arr->array[arr->size].panel_obj = create_alarm(arr->array[arr->size].panel_obj_index, parent);
            arr->size++;
            break;
        default:
            printf("Input type is a unknown panel type, its type is: %d\r\n", type);
            return;
    }
    
    arr->array[arr->size].top_flag = 0;
    arr->array[arr->size].sec_arrive = get_sec();

    panel_index_store(arr);
    repos_by_index(arr);

    printf("finish to add an element\r\n");
    printf("********************************************\r\n");
}

// Find the physical index value of the array corresponding to the current index element
static uint16_t find_physical_index(DynamicArray *arr, uint16_t index) 
{
    uint16_t array_pyhsical_index = -1;
    for(uint16_t i = 0; i < arr->size; i++) 
    {
        if(arr->array[i].panel_obj_index == index) 
        {
            array_pyhsical_index = i;
            break;
        }
    }
    return array_pyhsical_index;
}

// Count the number of top mounted panels
static uint16_t count_top_num(DynamicArray *arr) 
{
    uint16_t cnt = 0;
    for(uint16_t i = 0;i < arr->size;i++) 
    {
        if(arr->array[i].top_flag == 1) 
        {
            cnt++;
        }
    }
    return cnt;
}

static long get_battery_text_value(panel_with_type * panel) 
{
    char * str = NULL;
    long battery_level = 0;
    char * endptr = NULL;
    errno = 0;  // reset errno
    switch(panel->panel_type) 
    {
        case TEM_HUM_TYPE:
            str = lv_label_get_text(panel->panel_union.tem_hum.ui_LabelBatteryTemHum);
            break;
        case DOOR_TYPE:
            str = lv_label_get_text(panel->panel_union.door.ui_LabelBatteryDoor);
            break;
        case WATER_LEAK_TYPE:
            str = lv_label_get_text(panel->panel_union.water_leak.ui_LabelBatteryWaterLeak);
            break;
        case OCCUPIED_TYPE:
            str = lv_label_get_text(panel->panel_union.occupied.ui_LabelBatteryOccupied);
            break;
        case BUTTON_TYPE:
            str = lv_label_get_text(panel->panel_union.button.ui_LabelBatteryButton);
            break;
        case ALARM_TYPE:
            str = lv_label_get_text(panel->panel_union.alarm.ui_LabelBatteryAlarm);
            break;
        default:
            break;
    }
    battery_level = strtol(str, &endptr, 10);
    // Check whether the conversion is successful
    if(errno == ERANGE) 
    {
        printf("Overflow or underflow occurred during conversion.\r\n");
    }
    else if(endptr == str)
    {
        printf("No digits were found.\r\n");
    }
    else if(*endptr != '\0')
    {
        printf("Further characters after number: %s\r\n", endptr);
        printf("The number is: %ld\r\n", battery_level);
    }
    else
    {
        printf("The number is: %ld\r\n", battery_level);
    }
    return battery_level;
}

static void panel_index_store(DynamicArray *arr) 
{
    for(uint16_t i = 0;i < arr->size;i++) 
    {
        panel_index_store_temp[i] = arr->array[i].panel_obj_index;
    }
}

static void time_process(lv_obj_t * ui_LabelTimePassedValue, time_t sec_distance) 
{
    int days = sec_distance / (24 * 60 * 60);  
    int hours = (sec_distance % (24 * 60 * 60)) / (60 * 60);  
    int minutes = (sec_distance % (60 * 60)) / 60;  
    int seconds = sec_distance % 60;
    lv_label_set_text_fmt(ui_LabelTimePassedValue, "%02d: %02d: %02d: %02d", days, hours, minutes, seconds);
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

/**
 * @brief get x by computing its logical index
 * @param index element logical index
 * @return x
 */
uint16_t x_by_index(uint16_t index) 
{
    return (index % PANEL_COL_NUM) * (PANEL_WIDTH + PANEL_X_DISTANCE) + (PANEL_START_X);
}

/**
 * @brief get y by computing its logical index
 * @param index element logical index
 * @return y
 */
uint16_t y_by_index(uint16_t index) 
{
    return (index / PANEL_COL_NUM) * (PANEL_HEIGHT + PANEL_Y_DISTANCE) + (PANEL_START_Y);
}

time_t get_sec(void) 
{
    time_t now_sec;
	time(&now_sec);
	return now_sec;
}

void config_save(int32_t brightness, uint16_t tem_unit, uint16_t boot_screen) 
{
    esp_err_t err = nvs_flash_init();
    if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    nvs_handle_t my_handle;
	err = nvs_open("setting_args", NVS_READWRITE, &my_handle); // NVS_READWRITE
	if(err != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!\r\n", esp_err_to_name(err));
    } 
    else 
    {
        printf("Done\r\n");
	}

	int32_t brightness_write = brightness;
    printf("brightness is %ld\r\n", brightness_write);
	err = nvs_set_i32(my_handle, "brightness", brightness_write);
	printf((err != ESP_OK) ? "Failed!\r\n" : "Done\r\n");

    uint16_t tem_unit_write = tem_unit;
    printf("tem_unit is %d\r\n", tem_unit_write);
	err = nvs_set_u16(my_handle, "tem_unit", tem_unit_write);
	printf((err != ESP_OK) ? "Failed!\r\n" : "Done\r\n");

    uint16_t boot_screen_write = boot_screen;
    printf("switch boot screen is %d\r\n", boot_screen_write);
	err = nvs_set_u16(my_handle, "boot_screen", boot_screen_write);
	printf((err != ESP_OK) ? "Failed!\r\n" : "Done\r\n");

	printf("Committing updates in NVS ... ");
	err = nvs_commit(my_handle);
	printf((err != ESP_OK) ? "Failed!\r\n" : "Done\r\n");

    nvs_close(my_handle);

    esp_lcd_rgb_panel_restart(panel_handle_for_avoid_screen_drift);   // avoid boot screen drift
}

// Initialize dynamic array  
void initArray(DynamicArray *arr, size_t initialCapacity) 
{
    arr->array = (panel_all *)malloc(initialCapacity * sizeof(panel_all));  
    arr->size = 0;  
    arr->capacity = initialCapacity;  
}  

// Release the memory of dynamic arrays  
void freeArray(DynamicArray *arr) 
{
    free(arr->array);  
    arr->array = NULL;  
    arr->size = 0;  
    arr->capacity = 0;  
}  

// Remove the element at the specified position from the array  
void deleteElement(DynamicArray *arr, size_t index) 
{
    printf("\r\n********************************************\r\n");
    printf("start to delete an element\r\n");
    size_t size_temp = arr->size;

    if(index >= size_temp) 
    {
        printf("Index %d out of bounds\r\n", index);
        return;  
    }

    uint16_t array_pyhsical_index = find_physical_index(arr, index);
    printf("array size: %d  delete physical index: %d  delete logical index: %d\r\n", size_temp, array_pyhsical_index, index);
    
    switch(arr->array[array_pyhsical_index].panel_obj.panel_type) 
    {
        case TEM_HUM_TYPE:
            printf("Delete a TEM_HUM_TYPE panel, its type is: %d\r\n", arr->array[array_pyhsical_index].panel_obj.panel_type);
            lv_obj_remove_style_all(arr->array[array_pyhsical_index].panel_obj.panel_union.tem_hum.ui_PanelSensorTemHum);
            lv_obj_del(arr->array[array_pyhsical_index].panel_obj.panel_union.tem_hum.ui_PanelSensorTemHum);
            break;
        case DOOR_TYPE:
            printf("Delete a DOOR_TYPE panel, its type is: %d\r\n", arr->array[array_pyhsical_index].panel_obj.panel_type);
            lv_obj_remove_style_all(arr->array[array_pyhsical_index].panel_obj.panel_union.door.ui_PanelSensorDoor);
            lv_obj_del(arr->array[array_pyhsical_index].panel_obj.panel_union.door.ui_PanelSensorDoor);
            break;
        case WATER_LEAK_TYPE:
            printf("Delete a WATER_LEAK_TYPE panel, its type is: %d\r\n", arr->array[array_pyhsical_index].panel_obj.panel_type);
            lv_obj_remove_style_all(arr->array[array_pyhsical_index].panel_obj.panel_union.water_leak.ui_PanelSensorWaterLeak);
            lv_obj_del(arr->array[array_pyhsical_index].panel_obj.panel_union.water_leak.ui_PanelSensorWaterLeak);
            break;
        case OCCUPIED_TYPE:
            printf("Delete a OCCUPIED_TYPE panel, its type is: %d\r\n", arr->array[array_pyhsical_index].panel_obj.panel_type);
            lv_obj_remove_style_all(arr->array[array_pyhsical_index].panel_obj.panel_union.occupied.ui_PanelSensorOccupied);
            lv_obj_del(arr->array[array_pyhsical_index].panel_obj.panel_union.occupied.ui_PanelSensorOccupied);
            break;
        case BUTTON_TYPE:
            printf("Delete a BUTTON_TYPE panel, its type is: %d\r\n", arr->array[array_pyhsical_index].panel_obj.panel_type);
            lv_obj_remove_style_all(arr->array[array_pyhsical_index].panel_obj.panel_union.button.ui_PanelSensorButton);
            lv_obj_del(arr->array[array_pyhsical_index].panel_obj.panel_union.button.ui_PanelSensorButton);
            break;
        case ALARM_TYPE:
            printf("Delete a ALARM_TYPE panel, its type is: %d\r\n", arr->array[array_pyhsical_index].panel_obj.panel_type);
            lv_obj_remove_style_all(arr->array[array_pyhsical_index].panel_obj.panel_union.alarm.ui_PanelSensorAlarm);
            lv_obj_del(arr->array[array_pyhsical_index].panel_obj.panel_union.alarm.ui_PanelSensorAlarm);
            break;
    }

    if(index < size_temp - 1) 
    {
        uint16_t temp = 0;
        for(uint16_t i = 0; i < size_temp; i++) 
        {
            for(uint16_t j = 0; j < size_temp; j++) 
            {
                if(arr->array[j].panel_obj_index == index + temp) 
                {
                    uint16_t physical_index = find_physical_index(arr, arr->array[j].panel_obj_index + 1);  //  Find the physical index corresponding to the next logical index
                    
                    arr->array[j] = arr->array[physical_index];
                    arr->array[j].panel_obj_index -= 1;
                    temp++;
                    break;
                }
            }
            if(index + temp >= size_temp - 1) 
            {
                break;  
            }
        }
    }

    uint16_t in = find_physical_index(arr,size_temp - 1);
    for(uint16_t i = in;i < size_temp - 1;i++) 
    {
        arr->array[i] = arr->array[i + 1];
    }
    
    arr->size--;
    printf("array size after delete an element %d\r\n", arr->size);
    panel_index_store(arr);
    repos_by_index(arr);

    printf("finish to delete an element\r\n");
    printf("********************************************\r\n");
}

// Find sensor node objects based on sub objects
panel_all * find_upper_by_SensorPanel(DynamicArray *arr, lv_obj_t * PanelSensor) 
{
    for(uint16_t i = 0;i < arr->size;i++) 
    {
        switch(arr->array[i].panel_obj.panel_type) 
        {
            case TEM_HUM_TYPE:
                if(arr->array[i].panel_obj.panel_union.tem_hum.ui_PanelSensorTemHum == PanelSensor)
                {
                    return (arr->array + i);
                }
                break;
            case DOOR_TYPE:
                if(arr->array[i].panel_obj.panel_union.door.ui_PanelSensorDoor == PanelSensor)
                {
                    return (arr->array + i);
                }
                break;
            case WATER_LEAK_TYPE:
                if(arr->array[i].panel_obj.panel_union.water_leak.ui_PanelSensorWaterLeak == PanelSensor)
                {
                    return (arr->array + i);
                }
                break;
            case OCCUPIED_TYPE:
                if(arr->array[i].panel_obj.panel_union.occupied.ui_PanelSensorOccupied == PanelSensor)
                {
                    return (arr->array + i);
                }
                break;
            case BUTTON_TYPE:
                if(arr->array[i].panel_obj.panel_union.button.ui_PanelSensorButton == PanelSensor)
                {
                    return (arr->array + i);
                }
                    
                break;
            case ALARM_TYPE:
                if(arr->array[i].panel_obj.panel_union.alarm.ui_PanelSensorAlarm == PanelSensor)
                {
                    return (arr->array + i);
                }
                break;
        }
    }
    return (panel_all *)NULL;
}

// Sort 1: Pin an element to the top
void reposArray1(DynamicArray *arr, lv_obj_t * PanelSensor) 
{
    printf("********************************************\r\n");
    printf("start to pin an element to top\r\n");

    uint16_t happen_index = -1;
    panel_all * happen_obj = find_upper_by_SensorPanel(arr, PanelSensor);
    happen_index = happen_obj->panel_obj_index;
    printf("logical index: %d pin to top\r\n", happen_index);
    if(happen_index == -1) 
    {
        return;
    }
        
    happen_obj->top_flag = 1;
    for(uint16_t i = 0;i < arr->size;i++) 
    {
        if(arr->array[i].panel_obj_index > happen_index) 
        {
            continue;
        } 
        else if(arr->array[i].panel_obj_index < happen_index) 
        {
            arr->array[i].panel_obj_index++;
        } 
        else if(arr->array[i].panel_obj_index == happen_index) 
        {
            arr->array[i].panel_obj_index = 0;
        }
    }
    panel_index_store(arr);
    repos_by_index(arr);

    printf("finish to pin an element to top\r\n");
    printf("********************************************\r\n");
}

// Sort 2: Unpin the top element
void reposArray2(DynamicArray *arr, lv_obj_t * PanelSensor) 
{
    printf("********************************************\r\n");
    printf("start to unpin an element from top\r\n");

    uint16_t happen_index = -1;
    panel_all * happen_obj = find_upper_by_SensorPanel(arr, PanelSensor);
    happen_index = happen_obj->panel_obj_index;

    printf("logical index: %d unpin from top\r\n", happen_index);
    
    if(happen_index == -1)
    {
        return;
    }
        
    happen_obj->top_flag = 0;
    uint16_t count_top = count_top_num(arr);
    for(uint16_t i = 0;i < arr->size;i++) 
    {
        if(arr->array[i].panel_obj_index == happen_index) 
        {
            arr->array[i].panel_obj_index = count_top;
        }
        else if(arr->array[i].panel_obj_index > happen_index && arr->array[i].panel_obj_index < count_top + 1) 
        {
            arr->array[i].panel_obj_index--;
        }
    }
    panel_index_store(arr);
    repos_by_index(arr);
    printf("finish to unpin an element from top\r\n");
    printf("********************************************\r\n");
}


/**
 * @brief sort panel by time. TIME_SORT_EARLIEST: Arrival times ranging from early to late; TIME_SORT_LASTEST: Arrival times ranging from late to early
 */
void reposArray_by_time(DynamicArray *arr, uint8_t sort_type) 
{
    if(time_sort_value == 0) 
    {
        panel_index_store(arr);
    }

    if(sort_type == TIME_SORT_EARLIEST)
    {
        printf("\r\n\nEarliest sort\r\n");
        time_t min_begin_sec = 0;
        uint16_t min_sec_index_temp = 0;
        uint16_t index_temp = 0;
        uint16_t num_top = count_top_num(arr);
        uint16_t num_temp = num_top;
        uint16_t swap_data_temp;
        for(uint16_t j = 0;j < (arr->size - num_top);j++) 
        {
            for(uint16_t i = 0;i < arr->size;i++) 
            {
                if(arr->array[i].panel_obj_index < num_temp)
                {
                    continue;
                }
                else 
                {
                    min_begin_sec = arr->array[i].sec_arrive;
                    min_sec_index_temp = i;
                    break;
                }
            }
            for(uint16_t i = 0;i < arr->size;i++) 
            {
                if(arr->array[i].panel_obj_index < num_temp) 
                {
                    continue;
                }
                else 
                {
                    if(arr->array[i].panel_obj_index == num_temp)
                    {
                        index_temp = i;
                    }
                    if(arr->array[i].sec_arrive < min_begin_sec) 
                    {
                        min_begin_sec = arr->array[i].sec_arrive;
                        min_sec_index_temp = i;
                    }
                }
            }
            if(min_sec_index_temp != index_temp) 
            {
                printf("swap happen!\r\n");
                swap_data_temp = arr->array[index_temp].panel_obj_index;
                arr->array[index_temp].panel_obj_index = arr->array[min_sec_index_temp].panel_obj_index;
                arr->array[min_sec_index_temp].panel_obj_index = swap_data_temp;
            }
            num_temp++;
        }
    }
    else if(sort_type == TIME_SORT_LASTEST)
    {
        printf("\r\n\nLastesd sort\r\n");
        time_t max_begin_sec = 0;
        uint16_t max_sec_index_temp = 0;
        uint16_t index_temp = 0;
        uint16_t num_top = count_top_num(arr);
        uint16_t num_temp = num_top;
        uint16_t swap_data_temp;
        for(uint16_t j = 0;j < (arr->size - num_top);j++) 
        {
            for(uint16_t i = 0;i < arr->size;i++) 
            {
                if(arr->array[i].panel_obj_index < num_temp) 
                {
                    continue;
                }
                else 
                {
                    max_begin_sec = arr->array[i].sec_arrive;
                    max_sec_index_temp = i;
                    break;
                }
            }
            for(uint16_t i = 0;i < arr->size;i++) 
            {
                if(arr->array[i].panel_obj_index < num_temp)
                {
                    continue;
                }
                else 
                {
                    if(arr->array[i].panel_obj_index == num_temp)
                    {
                        index_temp = i;
                    }
                    if(arr->array[i].sec_arrive > max_begin_sec) 
                    {
                        max_begin_sec = arr->array[i].sec_arrive;
                        max_sec_index_temp = i;
                    }
                }
            }
            if(index_temp != max_sec_index_temp) 
            {
                printf("swap happen!\r\n");
                swap_data_temp = arr->array[index_temp].panel_obj_index;
                arr->array[index_temp].panel_obj_index = arr->array[max_sec_index_temp].panel_obj_index;
                arr->array[max_sec_index_temp].panel_obj_index = swap_data_temp;
            }
            num_temp++;
        }
    }  
    repos_by_index(arr);
}

/**
 * @brief sort panel by bat level. BATTERY_LEVEL_SORT_LOWEST: bat level from low to high; BATTERY_LEVEL_SORT_HIGHEST: bat level from high to low
 */
void reposArray_by_battery(DynamicArray *arr, uint8_t sort_type) 
{
    if(battery_sort_value == 0) 
    {
        panel_index_store(arr);
    }

    if(sort_type == BATTERY_LEVEL_SORT_LOWEST)
    {
        long min_battery = 0;
        uint16_t min_battery_index_temp = 0;
        uint16_t index_temp = 0;
        uint16_t num_top = count_top_num(arr);
        uint16_t num_temp = num_top;
        uint16_t swap_data_temp;
        for(uint16_t j = 0;j < (arr->size - num_top);j++) 
        {
            for(uint16_t i = 0;i < arr->size;i++) 
            {
                if(arr->array[i].panel_obj_index < num_temp) 
                {
                    continue;
                }
                else 
                {
                    min_battery = get_battery_text_value(&(arr->array[i].panel_obj));
                    min_battery_index_temp = i;
                    break;
                }
            }
            for(uint16_t i = 0;i < arr->size;i++) 
            {
                if(arr->array[i].panel_obj_index < num_temp) 
                {
                    continue;
                }
                else 
                {
                    long battery_temp = get_battery_text_value(&(arr->array[i].panel_obj));
                    if(arr->array[i].panel_obj_index == num_temp) 
                    {
                        index_temp = i;
                    }
                    if(battery_temp < min_battery) 
                    {
                        min_battery = battery_temp;
                        min_battery_index_temp = i;
                    }
                }
            }
            if(index_temp != min_battery_index_temp) 
            {
                swap_data_temp = arr->array[index_temp].panel_obj_index;
                arr->array[index_temp].panel_obj_index = arr->array[min_battery_index_temp].panel_obj_index;
                arr->array[min_battery_index_temp].panel_obj_index = swap_data_temp;
            }
            num_temp++;
        }
    }
    else if(sort_type == BATTERY_LEVEL_SORT_HIGHEST)
    {
        long max_battery = 0;
        uint16_t max_battery_index_temp = 0;
        uint16_t index_temp = 0;
        uint16_t num_top = count_top_num(arr);
        uint16_t num_temp = num_top;
        uint16_t swap_data_temp;
        for(uint16_t j = 0;j < (arr->size - num_top);j++) 
        {
            for(uint16_t i = 0;i < arr->size;i++) 
            {
                if(arr->array[i].panel_obj_index < num_temp) 
                {
                    continue;
                }
                else 
                {
                    max_battery = get_battery_text_value(&(arr->array[i].panel_obj));
                    max_battery_index_temp = i;
                    break;
                }
            }
            for(uint16_t i = 0;i < arr->size;i++) 
            {
                if(arr->array[i].panel_obj_index < num_temp) 
                {
                    continue;
                }
                else 
                {
                    long battery_temp = get_battery_text_value(&(arr->array[i].panel_obj));
                    if(arr->array[i].panel_obj_index == num_temp)
                    {
                        index_temp = i;
                    }
                    if(battery_temp > max_battery) {
                        max_battery = battery_temp;
                        max_battery_index_temp = i;
                    }
                }
            }
            if(index_temp != max_battery_index_temp) 
            {
                swap_data_temp = arr->array[index_temp].panel_obj_index;
                arr->array[index_temp].panel_obj_index = arr->array[max_battery_index_temp].panel_obj_index;
                arr->array[max_battery_index_temp].panel_obj_index = swap_data_temp;
            }
            num_temp++;
        }
    }
    repos_by_index(arr);
}

void panel_index_restore(DynamicArray *arr) 
{
    for(uint16_t i = 0;i < arr->size;i++) 
    {
        arr->array[i].panel_obj_index = panel_index_store_temp[i];
    }
        
    repos_by_index(arr);
}

void timer_callback(lv_timer_t * timer) 
{
    DynamicArray * arr_temp = &arr;
    uint32_t time_offset = 0;
    uint32_t time_begin_temp = 0;
    uint32_t time_end_temp = 0;
    
	time_begin_temp = sec_now;
    sec_now = get_sec();
    time_end_temp = sec_now;
    time_offset = time_end_temp - time_begin_temp;
    if(time_offset > 1000000000) 
    {
        for(int i = 0;i < arr_temp->size;i++) 
        {
            arr_temp->array[i].sec_arrive = arr_temp->array[i].sec_arrive + time_offset;
        }
    }

    tzset();
    uint8_t button_uplink_num_temp = 0;
    for(uint16_t i = 0;i < arr_temp->size;i++) 
    {
		time_t sec_distance = sec_now - arr_temp->array[i].sec_arrive;
        switch(arr_temp->array[i].panel_obj.panel_type) 
        {
            case TEM_HUM_TYPE:
                time_process(arr_temp->array[i].panel_obj.panel_union.tem_hum.ui_LabelTimePassedValueTemHum, sec_distance);
                break;
            case DOOR_TYPE:
                time_process(arr_temp->array[i].panel_obj.panel_union.door.ui_LabelTimePassedValueDoor, sec_distance);
                break;
            case WATER_LEAK_TYPE:
                time_process(arr_temp->array[i].panel_obj.panel_union.water_leak.ui_LabelTimePassedValueWaterLeak, sec_distance);
                break;
            case OCCUPIED_TYPE:
                time_process(arr_temp->array[i].panel_obj.panel_union.occupied.ui_LabelTimePassedValueOccupied, sec_distance);
                break;
            case BUTTON_TYPE:
                time_process(arr_temp->array[i].panel_obj.panel_union.button.ui_LabelTimePassedValueButton, sec_distance);
                if(arr_temp->array[i].panel_obj.panel_union.button.button_press_flag == true) {
                    button_uplink_num_temp++;
                }
                break;
            case ALARM_TYPE:
                time_process(arr_temp->array[i].panel_obj.panel_union.alarm.ui_LabelTimePassedValueAlarm, sec_distance);
                break;
        }
	}
    if(button_uplink_num_temp > 0) {
        button_uplink_num = button_uplink_num_temp;
    } 
    int days = (sec_now - system_begin_sec) / (24 * 60 * 60);  
    int hours = ((sec_now - system_begin_sec) % (24 * 60 * 60)) / (60 * 60);  
    int minutes = ((sec_now - system_begin_sec) % (60 * 60)) / 60;  
    int seconds = (sec_now - system_begin_sec) % 60;
    lv_label_set_text_fmt(ui_LabelBootTimeValue, "%02d:%02d:%02d:%02d", days, hours, minutes, seconds);

    tem_unit_convert(1, &arr);
    tem_unit_convert(2, &arr);
}

void timer_alarm_callback(lv_timer_t * timer) 
{
    DynamicArray * arr_temp = &arr;

    for(uint16_t i = 0;i < arr_temp->size;i++) 
    {
        if(arr_temp->array[i].panel_obj.panel_type == ALARM_TYPE) 
        {
            if(arr_temp->array[i].panel_obj.panel_union.alarm.timer_alarm_en_flag == ALARM_TIMER_ENABLE) 
            {
                if(arr_temp->array[i].panel_obj.panel_union.alarm.timer_exec_status == ALARM_TIMER_STATUS_OFF) 
                {
                    lv_obj_set_style_img_recolor_opa(arr_temp->array[i].panel_obj.panel_union.alarm.ui_ImageAlarmAlarm, LV_OPA_50, LV_STATE_DEFAULT);
                    lv_obj_set_style_img_recolor(arr_temp->array[i].panel_obj.panel_union.alarm.ui_ImageAlarmAlarm, lv_color_hex(0x3C2626), LV_STATE_DEFAULT);
                    arr_temp->array[i].panel_obj.panel_union.alarm.timer_exec_status = ALARM_TIMER_STATUS_ON;
                    uint16_t delay_time = (esp_random() / 0xFFFFFFFF) * 200;
                    vTaskDelay(delay_time / portTICK_PERIOD_MS);
                }
                else if(arr_temp->array[i].panel_obj.panel_union.alarm.timer_exec_status == ALARM_TIMER_STATUS_ON) 
                {
                    lv_obj_set_style_img_recolor_opa(arr_temp->array[i].panel_obj.panel_union.alarm.ui_ImageAlarmAlarm, LV_OPA_0, LV_STATE_DEFAULT);
                    lv_obj_set_style_img_recolor(arr_temp->array[i].panel_obj.panel_union.alarm.ui_ImageAlarmAlarm, lv_color_hex(0x3C2626), LV_STATE_DEFAULT);
                    arr_temp->array[i].panel_obj.panel_union.alarm.timer_exec_status = ALARM_TIMER_STATUS_OFF;
                    uint16_t delay_time = (esp_random() / 0xFFFFFFFF) * 200;
                    vTaskDelay(delay_time / portTICK_PERIOD_MS);
                }
            }
            else if(arr_temp->array[i].panel_obj.panel_union.alarm.timer_alarm_en_flag == ALARM_TIMER_DISABLE) 
            {
                lv_obj_set_style_img_recolor_opa(arr_temp->array[i].panel_obj.panel_union.alarm.ui_ImageAlarmAlarm, LV_OPA_50, LV_STATE_DEFAULT);
                lv_obj_set_style_img_recolor(arr_temp->array[i].panel_obj.panel_union.alarm.ui_ImageAlarmAlarm, lv_color_hex(0x3C2626), LV_STATE_DEFAULT);
            }
        }
    }

    lv_label_set_text_fmt(ui_LabelPanelNumberValue, "%d", arr_temp->size);
}

void time_sort_value_set(uint16_t new_value)
{
    time_sort_value = new_value;
}
uint16_t time_sort_value_get(void)
{
    return time_sort_value;
}
void battery_sort_value_set(uint16_t new_value)
{
    battery_sort_value = new_value;
}
uint16_t battery_sort_value_get(void)
{
    return battery_sort_value;
}
void nvs_store_complete_flag_set(bool new_value)
{
    nvs_store_complete_flag = new_value;
}
bool nvs_store_complete_flag_get(void)
{
    return nvs_store_complete_flag;
}
void save_button_press_flag_set(bool new_value)
{
    save_button_press_flag = new_value;
}
bool save_button_press_flag_get(void)
{
    return save_button_press_flag;
}
uint8_t button_uplink_num_get(void)
{
    return button_uplink_num;
}
void button_uplink_num_set(uint8_t button_uplink_num_temp)
{
    button_uplink_num = button_uplink_num_temp;
}
DynamicArray * arr_get(void)
{
    return &arr;
}
void system_begin_sec_set(time_t new_value)
{
    system_begin_sec = new_value;
}
time_t system_begin_sec_get(void)
{
    return system_begin_sec;
}
void lvgl_lock_get(void)
{
    xSemaphoreTake(Mutex, portMAX_DELAY);
}
void lvgl_lock_release(void)
{
    xSemaphoreGive(Mutex);
}


// By DEV Name, DEV EUI, Panel type, the three uniquely determine whether a message will be updated to an existing panel
void panel_update(message * mes, DynamicArray * arr, lv_obj_t * parent) 
{
    char dest1[10] = {0};
    char dest2[10] = {0};
    
    strncpy(dest1, ((char *)(mes->dev_eui) + 5), 3);
    printf("\r\n");
    for(int i = 0;i < strlen(dest1);i++) 
    {
        ESP_LOGI(TAG_SORT, "dest1: %02X", dest1[i]);//\r\n  d
    }

    sprintf(dest2 + 0, "%02X", *(dest1 + 0));
    sprintf(dest2 + 2, "%02X", *(dest1 + 1));
    sprintf(dest2 + 4, "%02X", *(dest1 + 2));
    ESP_LOGI(TAG_SORT, "dest2: %s", dest2);//\r\n

    printf("\r\n");
    
    // refresh old panel
    for(uint16_t i = 0;i < arr->size;i++) 
    {
        if(arr->array[i].panel_obj.panel_type != mes->mod) 
        {
            continue;
        }

        
        switch(mes->mod) 
        {
            case TEM_HUM_TYPE:
                if(strcmp(lv_label_get_text(arr->array[i].panel_obj.panel_union.tem_hum.ui_LabelNameTemHum), (char *)(mes->dev_name)) == 0 && strcmp(lv_label_get_text(arr->array[i].panel_obj.panel_union.tem_hum.ui_LabelDevEUITemHum), dest2) == 0) 
                { 
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.tem_hum.ui_LabelTemValueTemHum, "%.2f℃", ((double)(*(mes->temp1)) + (double)*((mes->temp1)+1)*256)/100.0); //set tem1  //采用小端模式存储数据
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.tem_hum.ui_LabelExtValueTemHum, "%.2f℃", ((double)(*(mes->temp2)) + (double)*((mes->temp2)+1)*256)/100.0); //set tem2
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.tem_hum.ui_LabelHumValueTemHum, "%.2f%%RH", ((double)(*(mes->humidity)) + (double)*((mes->humidity)+1)*256)/10.0); //set hum
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.tem_hum.ui_LabelBatteryTemHum, "%dmV", *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256); //set bat label
                    lv_slider_set_value(arr->array[i].panel_obj.panel_union.tem_hum.ui_SliderBatteryTemHum, *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256, LV_ANIM_OFF); //set bat slider
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.tem_hum.ui_LabelSignalStrengthValueTemHum, "Rssi: %d", mes->data_rssi); //set signal strength label
                    lv_slider_set_value(arr->array[i].panel_obj.panel_union.tem_hum.ui_SliderSignalStrengthTemHum, mes->data_rssi, LV_ANIM_OFF); //set signal strength slider
                    arr->array[i].sec_arrive = get_sec(); //record arrive time
                    NewPanelArrive_Animation(ui_ContainerRefreshMessageNotice, 1000);
                    return;
                }
                break;
            case DOOR_TYPE:
                if(strcmp(lv_label_get_text(arr->array[i].panel_obj.panel_union.door.ui_LabelNameDoor), (char *)(mes->dev_name)) == 0 && strcmp(lv_label_get_text(arr->array[i].panel_obj.panel_union.door.ui_LabelDevEUIDoor), dest2) == 0) 
                {
                    //set door_status
                    if(*(mes->status) == DOOR_STATUS_OPEN) 
                    {
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.door.ui_PanelCurrentStatusCloseDoor, LV_OBJ_FLAG_HIDDEN) == false)
                        {
                            lv_obj_add_flag(arr->array[i].panel_obj.panel_union.door.ui_PanelCurrentStatusCloseDoor, LV_OBJ_FLAG_HIDDEN);
                        }
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.door.ui_PanelCurrentStatusOpenDoor, LV_OBJ_FLAG_HIDDEN) == true)
                        {
                            lv_obj_clear_flag(arr->array[i].panel_obj.panel_union.door.ui_PanelCurrentStatusOpenDoor, LV_OBJ_FLAG_HIDDEN);
                        }
                    }
                    else if(*(mes->status) == DOOR_STATUS_CLOSE) 
                    {
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.door.ui_PanelCurrentStatusOpenDoor, LV_OBJ_FLAG_HIDDEN) == false)
                        {
                            lv_obj_add_flag(arr->array[i].panel_obj.panel_union.door.ui_PanelCurrentStatusOpenDoor, LV_OBJ_FLAG_HIDDEN);
                        }
                            
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.door.ui_PanelCurrentStatusCloseDoor, LV_OBJ_FLAG_HIDDEN) == true)
                        {
                            lv_obj_clear_flag(arr->array[i].panel_obj.panel_union.door.ui_PanelCurrentStatusCloseDoor, LV_OBJ_FLAG_HIDDEN);
                        }
                    }
                    
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.door.ui_LabelBatteryDoor, "%dmV", *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256); //set bat label
                    lv_slider_set_value(arr->array[i].panel_obj.panel_union.door.ui_SliderBatteryDoor, *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256, LV_ANIM_OFF); //set bat slider
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.door.ui_LabelSignalStrengthValueDoor, "Rssi: %d", mes->data_rssi); //set signal strength label
                    lv_slider_set_value(arr->array[i].panel_obj.panel_union.door.ui_SliderSignalStrengthDoor, mes->data_rssi, LV_ANIM_OFF); //set signal strength slider
                    arr->array[i].sec_arrive = get_sec(); //record arrive time
                    NewPanelArrive_Animation(ui_ContainerRefreshMessageNotice, 1000);
                    return;
                }
                break;
            case WATER_LEAK_TYPE:
                if(strcmp(lv_label_get_text(arr->array[i].panel_obj.panel_union.water_leak.ui_LabelNameWaterLeak), (char *)(mes->dev_name)) == 0 && strcmp(lv_label_get_text(arr->array[i].panel_obj.panel_union.water_leak.ui_LabelDevEUIWaterLeak), dest2) == 0) 
                {
                    //set water_leak_status
                    if(*(mes->status) == WATER_LEAK_STATUS_NORMAL) 
                    {
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusLeakingWaterLeak, LV_OBJ_FLAG_HIDDEN) == false)
                        {
                            lv_obj_add_flag(arr->array[i].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusLeakingWaterLeak, LV_OBJ_FLAG_HIDDEN);
                        }
                            
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusNormalWaterLeak, LV_OBJ_FLAG_HIDDEN) == true)
                        {
                            lv_obj_clear_flag(arr->array[i].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusNormalWaterLeak, LV_OBJ_FLAG_HIDDEN);
                        }
                    }
                    else if(*(mes->status) == WATER_LEAK_STATUS_LEAKING) 
                    {
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusNormalWaterLeak, LV_OBJ_FLAG_HIDDEN) == false)
                        {
                            lv_obj_add_flag(arr->array[i].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusNormalWaterLeak, LV_OBJ_FLAG_HIDDEN);
                        }
                            
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusLeakingWaterLeak, LV_OBJ_FLAG_HIDDEN) == true)
                        {
                            lv_obj_clear_flag(arr->array[i].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusLeakingWaterLeak, LV_OBJ_FLAG_HIDDEN);
                        }
                    }
                    
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.water_leak.ui_LabelBatteryWaterLeak, "%dmV", *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256); //set bat label
                    lv_slider_set_value(arr->array[i].panel_obj.panel_union.water_leak.ui_SliderBatteryWaterLeak, *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256, LV_ANIM_OFF); //set bat slider
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.water_leak.ui_LabelSignalStrengthValueWaterLeak, "Rssi: %d", mes->data_rssi); //set signal strength label
                    lv_slider_set_value(arr->array[i].panel_obj.panel_union.water_leak.ui_SliderSignalStrengthWaterLeak, mes->data_rssi, LV_ANIM_OFF); //set signal strength slider
                    arr->array[i].sec_arrive = get_sec(); //record arrive time
                    NewPanelArrive_Animation(ui_ContainerRefreshMessageNotice, 1000);
                    return;
                }
                break;
            case OCCUPIED_TYPE:
                if(strcmp(lv_label_get_text(arr->array[i].panel_obj.panel_union.occupied.ui_LabelNameOccupied), (char *)(mes->dev_name)) == 0 && strcmp(lv_label_get_text(arr->array[i].panel_obj.panel_union.occupied.ui_LabelDevEUIOccupied), dest2) == 0) 
                {
                    //set occupied_status
                    if(*(mes->status) == OCCUPIED_STATUS_FREE) 
                    {
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.occupied.ui_LabelOccupiedOccupied, LV_OBJ_FLAG_HIDDEN) == false) 
                        {
                            lv_obj_add_flag(arr->array[i].panel_obj.panel_union.occupied.ui_LabelOccupiedOccupied, LV_OBJ_FLAG_HIDDEN);
                        }
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.occupied.ui_ImageOccupiedOccupied, LV_OBJ_FLAG_HIDDEN) == false) 
                        {
                            lv_obj_add_flag(arr->array[i].panel_obj.panel_union.occupied.ui_ImageOccupiedOccupied, LV_OBJ_FLAG_HIDDEN);
                        }
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.occupied.ui_LabelFreeOccupied, LV_OBJ_FLAG_HIDDEN) == true) 
                        {
                            lv_obj_clear_flag(arr->array[i].panel_obj.panel_union.occupied.ui_LabelFreeOccupied, LV_OBJ_FLAG_HIDDEN);
                        }
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.occupied.ui_ImageFreeOccupied, LV_OBJ_FLAG_HIDDEN) == true) 
                        {
                            lv_obj_clear_flag(arr->array[i].panel_obj.panel_union.occupied.ui_ImageFreeOccupied, LV_OBJ_FLAG_HIDDEN);
                        }
                    }
                    else if(*(mes->status) == OCCUPIED_STATUS_OCCUPIED) 
                    {
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.occupied.ui_LabelFreeOccupied, LV_OBJ_FLAG_HIDDEN) == false) 
                        {
                            lv_obj_add_flag(arr->array[i].panel_obj.panel_union.occupied.ui_LabelFreeOccupied, LV_OBJ_FLAG_HIDDEN);
                        }
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.occupied.ui_ImageFreeOccupied, LV_OBJ_FLAG_HIDDEN) == false) 
                        {
                            lv_obj_add_flag(arr->array[i].panel_obj.panel_union.occupied.ui_ImageFreeOccupied, LV_OBJ_FLAG_HIDDEN);
                        }
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.occupied.ui_LabelOccupiedOccupied, LV_OBJ_FLAG_HIDDEN) == true) 
                        {
                            lv_obj_clear_flag(arr->array[i].panel_obj.panel_union.occupied.ui_LabelOccupiedOccupied, LV_OBJ_FLAG_HIDDEN);
                        }
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.occupied.ui_ImageOccupiedOccupied, LV_OBJ_FLAG_HIDDEN) == true) 
                        {
                            lv_obj_clear_flag(arr->array[i].panel_obj.panel_union.occupied.ui_ImageOccupiedOccupied, LV_OBJ_FLAG_HIDDEN);
                        }
                    }
                    
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.occupied.ui_LabelBatteryOccupied, "%dmV", *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256); //set bat label
                    lv_slider_set_value(arr->array[i].panel_obj.panel_union.occupied.ui_SliderBatteryOccupied, *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256, LV_ANIM_OFF); //set bat slider
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.occupied.ui_LabelSignalStrengthValueOccupied, "Rssi: %d", mes->data_rssi); //set signal strength label
                    lv_slider_set_value(arr->array[i].panel_obj.panel_union.occupied.ui_SliderSignalStrengthOccupied, mes->data_rssi, LV_ANIM_OFF); //set signal strength slider
                    arr->array[i].sec_arrive = get_sec(); //record arrive time
                    NewPanelArrive_Animation(ui_ContainerRefreshMessageNotice, 1000);
                    return;
                }
                break;
            case BUTTON_TYPE:
                if(strcmp(lv_label_get_text(arr->array[i].panel_obj.panel_union.button.ui_LabelNameButton), (char *)(mes->dev_name)) == 0 && strcmp(lv_label_get_text(arr->array[i].panel_obj.panel_union.button.ui_LabelDevEUIButton), dest2) == 0) 
                {
                    //set button_status
                    if(*(mes->status) == BUTTON_STATUS_OFF) 
                    {
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.button.ui_LabelButtonONButton, LV_OBJ_FLAG_HIDDEN) == false) 
                        {
                            lv_obj_add_flag(arr->array[i].panel_obj.panel_union.button.ui_LabelButtonONButton, LV_OBJ_FLAG_HIDDEN);
                        }
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.button.ui_LabelButtonOFFButton, LV_OBJ_FLAG_HIDDEN) == true) 
                        {
                            lv_obj_clear_flag(arr->array[i].panel_obj.panel_union.button.ui_LabelButtonOFFButton, LV_OBJ_FLAG_HIDDEN);
                        }
                        if(lv_obj_has_state(arr->array[i].panel_obj.panel_union.button.ui_SwitchButtonButton, LV_STATE_CHECKED) == true) 
                        {
                            lv_obj_clear_state(arr->array[i].panel_obj.panel_union.button.ui_SwitchButtonButton, LV_STATE_CHECKED);
                        }
                    }
                    else if(*(mes->status) == BUTTON_STATUS_ON) 
                    {
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.button.ui_LabelButtonOFFButton, LV_OBJ_FLAG_HIDDEN) == false) 
                        {
                            lv_obj_add_flag(arr->array[i].panel_obj.panel_union.button.ui_LabelButtonOFFButton, LV_OBJ_FLAG_HIDDEN);
                        }
                        if(lv_obj_has_flag(arr->array[i].panel_obj.panel_union.button.ui_LabelButtonONButton, LV_OBJ_FLAG_HIDDEN) == true) 
                        {
                            lv_obj_clear_flag(arr->array[i].panel_obj.panel_union.button.ui_LabelButtonONButton, LV_OBJ_FLAG_HIDDEN);
                        }
                        if(lv_obj_has_state(arr->array[i].panel_obj.panel_union.button.ui_SwitchButtonButton, LV_STATE_CHECKED) == false) 
                        {
                            lv_obj_add_state(arr->array[i].panel_obj.panel_union.button.ui_SwitchButtonButton, LV_STATE_CHECKED);
                        }
                    }
                    
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.button.ui_LabelBatteryButton, "%dmV", *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256); //set bat label
                    lv_slider_set_value(arr->array[i].panel_obj.panel_union.button.ui_SliderBatteryButton, *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256, LV_ANIM_OFF); //set bat slider
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.button.ui_LabelSignalStrengthValueButton, "Rssi: %d", mes->data_rssi); //set signal strength label
                    lv_slider_set_value(arr->array[i].panel_obj.panel_union.button.ui_SliderSignalStrengthButton, mes->data_rssi, LV_ANIM_OFF); //set signal strength slider
                    arr->array[i].sec_arrive = get_sec(); //record arrive time
                    NewPanelArrive_Animation(ui_ContainerRefreshMessageNotice, 1000);
                    return;
                }
                break;

            case ALARM_TYPE:
                if(strcmp(lv_label_get_text(arr->array[i].panel_obj.panel_union.alarm.ui_LabelNameAlarm), (char *)(mes->dev_name)) == 0 && strcmp(lv_label_get_text(arr->array[i].panel_obj.panel_union.alarm.ui_LabelDevEUIAlarm), dest2) == 0) 
                {
                    //set alarm_status
                    if(*(mes->status) == ALARM_STATUS_ALARM) 
                    {
                        arr->array[arr->size - 1].panel_obj.panel_union.alarm.timer_alarm_en_flag = ALARM_TIMER_ENABLE;
                    }
                    else if(*(mes->status) == ALARM_STATUS_OFF) 
                    {
                        arr->array[arr->size - 1].panel_obj.panel_union.alarm.timer_alarm_en_flag = ALARM_TIMER_DISABLE;
                    }
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.alarm.ui_LabelBatteryAlarm, "%dmV", *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256); //set bat label
                    lv_slider_set_value(arr->array[i].panel_obj.panel_union.alarm.ui_SliderBatteryAlarm, *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256, LV_ANIM_OFF); //set bat slider
                    lv_label_set_text_fmt(arr->array[i].panel_obj.panel_union.alarm.ui_LabelSignalStrengthValueAlarm, "Rssi: %d", mes->data_rssi); //set signal strength label
                    lv_slider_set_value(arr->array[i].panel_obj.panel_union.alarm.ui_SliderSignalStrengthAlarm, mes->data_rssi, LV_ANIM_OFF); //set signal strength slider
                    arr->array[i].sec_arrive = get_sec(); //record arrive time
                    NewPanelArrive_Animation(ui_ContainerRefreshMessageNotice, 1000);
                    return;
                }
                break;
        }
    }
    
    // add new panel
    addElement(arr, mes->mod, parent); 
    
    // initialize new panel
    switch(mes->mod) 
    {
        case TEM_HUM_TYPE:
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.tem_hum.ui_LabelNameTemHum,"%s", (char *)(mes->dev_name)); //set dev_name
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.tem_hum.ui_LabelDevEUITemHum,"%s", dest2); //set dev_eui
            
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.tem_hum.ui_LabelTemValueTemHum, "%.2f℃", ((double)(*(mes->temp1)) + (double)*((mes->temp1) + 1)*256)/100.0); //set tem1  //采用小端模式存储数据，由于空了2个字节，所以数据是像这样的 7F FF 00 00
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.tem_hum.ui_LabelExtValueTemHum, "%.2f℃", ((double)(*(mes->temp2)) + (double)*((mes->temp2) + 1)*256)/100.0); //set tem2
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.tem_hum.ui_LabelHumValueTemHum, "%.2f%%RH", ((double)(*(mes->humidity)) + (double)*((mes->humidity) + 1)*256)/10.0); //set hum
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.tem_hum.ui_LabelBatteryTemHum, "%dmV", *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256); //set bat label
            lv_slider_set_value(arr->array[arr->size - 1].panel_obj.panel_union.tem_hum.ui_SliderBatteryTemHum, *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256, LV_ANIM_OFF); //set bat slider
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.tem_hum.ui_LabelSignalStrengthValueTemHum, "Rssi: %d", mes->data_rssi); //set signal strength label
            lv_slider_set_value(arr->array[arr->size - 1].panel_obj.panel_union.tem_hum.ui_SliderSignalStrengthTemHum, mes->data_rssi, LV_ANIM_OFF); //set signal strength slider
            arr->array[arr->size - 1].sec_arrive = get_sec(); //record arrive time
            NewPanelArrive_Animation(ui_ContainerNewMessageNotice, 1000);//0
            break;

        case DOOR_TYPE:
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_LabelNameDoor,"%s", (char *)(mes->dev_name)); //set dev_name
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_LabelDevEUIDoor,"%s", dest2); //set dev_eui

            //set door_status
            if(*(mes->status) == DOOR_STATUS_OPEN) 
            {
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_PanelCurrentStatusCloseDoor, LV_OBJ_FLAG_HIDDEN) == false)
                {
                    lv_obj_add_flag(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_PanelCurrentStatusCloseDoor, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_PanelCurrentStatusOpenDoor, LV_OBJ_FLAG_HIDDEN) == true)
                {
                    lv_obj_clear_flag(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_PanelCurrentStatusOpenDoor, LV_OBJ_FLAG_HIDDEN);
                }
            }
            else if(*(mes->status) == DOOR_STATUS_CLOSE) 
            {
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_PanelCurrentStatusOpenDoor, LV_OBJ_FLAG_HIDDEN) == false)
                {
                    lv_obj_add_flag(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_PanelCurrentStatusOpenDoor, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_PanelCurrentStatusCloseDoor, LV_OBJ_FLAG_HIDDEN) == true)
                {
                    lv_obj_clear_flag(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_PanelCurrentStatusCloseDoor, LV_OBJ_FLAG_HIDDEN);
                }
            }

            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_LabelBatteryDoor, "%dmV", *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256); //set bat label
            lv_slider_set_value(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_SliderBatteryDoor, *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256, LV_ANIM_OFF); //set bat slider
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_LabelSignalStrengthValueDoor, "Rssi: %d", mes->data_rssi);
            lv_slider_set_value(arr->array[arr->size - 1].panel_obj.panel_union.door.ui_SliderSignalStrengthDoor, mes->data_rssi, LV_ANIM_OFF);
            arr->array[arr->size - 1].sec_arrive = get_sec();
            NewPanelArrive_Animation(ui_ContainerNewMessageNotice, 1000);//0
            break;
        case WATER_LEAK_TYPE:
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_LabelNameWaterLeak,"%s", (char *)(mes->dev_name)); //set dev_name
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_LabelDevEUIWaterLeak,"%s", dest2); //set dev_eui
            
            //set water_leak_status
            if(*(mes->status) == WATER_LEAK_STATUS_NORMAL) 
            {
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusLeakingWaterLeak, LV_OBJ_FLAG_HIDDEN) == false)
                {
                    lv_obj_add_flag(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusLeakingWaterLeak, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusNormalWaterLeak, LV_OBJ_FLAG_HIDDEN) == true)
                {
                    lv_obj_clear_flag(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusNormalWaterLeak, LV_OBJ_FLAG_HIDDEN);
                }
            }
            else if(*(mes->status) == WATER_LEAK_STATUS_LEAKING) 
            {
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusNormalWaterLeak, LV_OBJ_FLAG_HIDDEN) == false)
                {
                    lv_obj_add_flag(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusNormalWaterLeak, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusLeakingWaterLeak, LV_OBJ_FLAG_HIDDEN) == true)
                {
                    lv_obj_clear_flag(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_PanelCurrentStatusLeakingWaterLeak, LV_OBJ_FLAG_HIDDEN);
                }
            }
            
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_LabelBatteryWaterLeak, "%dmV", *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256); //set bat label
            lv_slider_set_value(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_SliderBatteryWaterLeak, *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256, LV_ANIM_OFF); //set bat slider
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_LabelSignalStrengthValueWaterLeak, "Rssi: %d", mes->data_rssi); //set signal strength label
            lv_slider_set_value(arr->array[arr->size - 1].panel_obj.panel_union.water_leak.ui_SliderSignalStrengthWaterLeak, mes->data_rssi, LV_ANIM_OFF); //set signal strength slider
            arr->array[arr->size - 1].sec_arrive = get_sec(); //record arrive time
            NewPanelArrive_Animation(ui_ContainerNewMessageNotice, 1000);

            break;
        case OCCUPIED_TYPE:
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_LabelNameOccupied,"%s", (char *)(mes->dev_name)); //set dev_name
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_LabelDevEUIOccupied,"%s", dest2); //set dev_eui
            
            //set occupied_status
            if(*(mes->status) == OCCUPIED_STATUS_FREE) 
            {
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_LabelOccupiedOccupied, LV_OBJ_FLAG_HIDDEN) == false) 
                {
                    lv_obj_add_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_LabelOccupiedOccupied, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_ImageOccupiedOccupied, LV_OBJ_FLAG_HIDDEN) == false) 
                {
                    lv_obj_add_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_ImageOccupiedOccupied, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_LabelFreeOccupied, LV_OBJ_FLAG_HIDDEN) == true) 
                {
                    lv_obj_clear_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_LabelFreeOccupied, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_ImageFreeOccupied, LV_OBJ_FLAG_HIDDEN) == true) 
                {
                    lv_obj_clear_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_ImageFreeOccupied, LV_OBJ_FLAG_HIDDEN);
                }
            }
            else if(*(mes->status) == OCCUPIED_STATUS_OCCUPIED) 
            {
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_LabelFreeOccupied, LV_OBJ_FLAG_HIDDEN) == false) 
                {
                    lv_obj_add_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_LabelFreeOccupied, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_ImageFreeOccupied, LV_OBJ_FLAG_HIDDEN) == false) 
                {
                    lv_obj_add_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_ImageFreeOccupied, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_LabelOccupiedOccupied, LV_OBJ_FLAG_HIDDEN) == true) 
                {
                    lv_obj_clear_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_LabelOccupiedOccupied, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_ImageOccupiedOccupied, LV_OBJ_FLAG_HIDDEN) == true) 
                {
                    lv_obj_clear_flag(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_ImageOccupiedOccupied, LV_OBJ_FLAG_HIDDEN);
                }
            }
            
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_LabelBatteryOccupied, "%dmV", *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256); //set bat label
            lv_slider_set_value(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_SliderBatteryOccupied, *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256, LV_ANIM_OFF); //set bat slider
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_LabelSignalStrengthValueOccupied, "Rssi: %d", mes->data_rssi); //set signal strength label
            lv_slider_set_value(arr->array[arr->size - 1].panel_obj.panel_union.occupied.ui_SliderSignalStrengthOccupied, mes->data_rssi, LV_ANIM_OFF); //set signal strength slider
            arr->array[arr->size - 1].sec_arrive = get_sec(); //record arrive time
            NewPanelArrive_Animation(ui_ContainerNewMessageNotice, 1000);


            break;
        case BUTTON_TYPE:
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_LabelNameButton,"%s", (char *)(mes->dev_name)); //set dev_name
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_LabelDevEUIButton,"%s", dest2); //set dev_eui
            
            //set button_status
            if(*(mes->status) == BUTTON_STATUS_OFF) 
            {
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_LabelButtonONButton, LV_OBJ_FLAG_HIDDEN) == false) 
                {
                    lv_obj_add_flag(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_LabelButtonONButton, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_LabelButtonOFFButton, LV_OBJ_FLAG_HIDDEN) == true) 
                {
                    lv_obj_clear_flag(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_LabelButtonOFFButton, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_state(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_SwitchButtonButton, LV_STATE_CHECKED) == true) 
                {
                    lv_obj_clear_state(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_SwitchButtonButton, LV_STATE_CHECKED);
                }
            }
            else if(*(mes->status) == BUTTON_STATUS_ON) 
            {
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_LabelButtonOFFButton, LV_OBJ_FLAG_HIDDEN) == false) 
                {
                    lv_obj_add_flag(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_LabelButtonOFFButton, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_flag(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_LabelButtonONButton, LV_OBJ_FLAG_HIDDEN) == true) 
                {
                    lv_obj_clear_flag(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_LabelButtonONButton, LV_OBJ_FLAG_HIDDEN);
                }
                if(lv_obj_has_state(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_SwitchButtonButton, LV_STATE_CHECKED) == false) 
                {
                    lv_obj_add_state(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_SwitchButtonButton, LV_STATE_CHECKED);
                }
            }
            
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_LabelBatteryButton, "%dmV", *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256); //set bat label
            lv_slider_set_value(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_SliderBatteryButton, *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256, LV_ANIM_OFF); //set bat slider
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_LabelSignalStrengthValueButton, "Rssi: %d", mes->data_rssi); //set signal strength label
            lv_slider_set_value(arr->array[arr->size - 1].panel_obj.panel_union.button.ui_SliderSignalStrengthButton, mes->data_rssi, LV_ANIM_OFF); //set signal strength slider
            arr->array[arr->size - 1].sec_arrive = get_sec(); //record arrive time
            NewPanelArrive_Animation(ui_ContainerNewMessageNotice, 1000);

            break;

        case ALARM_TYPE:
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.alarm.ui_LabelNameAlarm,"%s", (char *)(mes->dev_name)); //set dev_name
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.alarm.ui_LabelDevEUIAlarm,"%s", dest2); //set dev_eui
            
            //set alarm_status
            if(*(mes->status) == ALARM_STATUS_ALARM) 
            {
                arr->array[arr->size - 1].panel_obj.panel_union.alarm.timer_alarm_en_flag = ALARM_TIMER_ENABLE;
            }
            else if(*(mes->status) == ALARM_STATUS_OFF) 
            {
                arr->array[arr->size - 1].panel_obj.panel_union.alarm.timer_alarm_en_flag = ALARM_TIMER_DISABLE;
            }
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.alarm.ui_LabelBatteryAlarm, "%dmV", *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256); //set bat label
            lv_slider_set_value(arr->array[arr->size - 1].panel_obj.panel_union.alarm.ui_SliderBatteryAlarm, *(mes->bat) + (*((mes->bat) + 1) & 0x3F)*256, LV_ANIM_OFF); //set bat slider
            lv_label_set_text_fmt(arr->array[arr->size - 1].panel_obj.panel_union.alarm.ui_LabelSignalStrengthValueAlarm, "Rssi: %d", mes->data_rssi); //set signal strength label
            lv_slider_set_value(arr->array[arr->size - 1].panel_obj.panel_union.alarm.ui_SliderSignalStrengthAlarm, mes->data_rssi, LV_ANIM_OFF); //set signal strength slider
            arr->array[arr->size - 1].sec_arrive = get_sec(); //record arrive time
            // lv_label_set_text_fmt(ui_ContainerNewMessageNotice, "sss");//Panel Alarm Added
            NewPanelArrive_Animation(ui_ContainerNewMessageNotice, 1000); // 动画的播放需要延时一定的时间，否则动画无法完整显示，若不等待这段时间则动画会被当前忙碌的LVGL定时执行任务所搁置，导致只显示出动画的最后一点便迅速结束了

            break;
    }
}

void sort_init(void)
{
    Mutex = xSemaphoreCreateMutex();
    system_begin_sec_set(get_sec());
    initArray(arr_get(), INIT_PANEL_CAPACITY);
    
    timer_lvgl = lv_timer_create(timer_callback, 1000, NULL);
    timer_alarm_lvgl = lv_timer_create(timer_alarm_callback, 200, NULL);

    // for(uint16_t i = 0;i < 6;i++) //PANEL_TYPE_NUM
	// {
	// 	lvgl_lock_get();
	// 	addElement(arr_get(), i + 1, ui_PanelContainer);
	// 	lvgl_lock_release();
	// }

    printf("sort_init ok\r\n");
}
