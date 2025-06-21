/* -------------------------------------------------------------------------- */
/* --- DEPENDENCIES --------------------------------------------------------- */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/ledc.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"

#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lv_port_fs.h"
#include "lv_demos.h"

#include "ui.h"
#include "sort.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ENC_LEN        (1)
#define DEV_NAME_LEN   (16) 
#define DEV_EUI_LEN    (8)
#define MOD_TYPE_LEN   (1)
#define BAT_LEN        (2)
#define TEMP1_LEN      (2) 
#define TEMP2_LEN      (2) 
#define HUMIDITY_LEN   (2) 
#define FPORT_LEN      (1)
#define DATA_TYPE_1_LEN (MOD_TYPE_LEN + BAT_LEN + TEMP1_LEN + TEMP2_LEN + HUMIDITY_LEN + FPORT_LEN)
#define STATUS_LEN     (1)
#define DATA_TYPE_2_LEN (MOD_TYPE_LEN + BAT_LEN + STATUS_LEN + FPORT_LEN)
#define MES_LEN_TYPE_1 (ENC_LEN + DEV_NAME_LEN + DEV_EUI_LEN + DATA_TYPE_1_LEN)
#define MES_LEN_TYPE_2 (ENC_LEN + DEV_NAME_LEN + DEV_EUI_LEN + DATA_TYPE_2_LEN)
#define RECEIVE_MAX_RAW_DATA_LEN (1024)
#define COM_UART UART_NUM_1
#define NON_ENCRYPTION (0x01)
#define DATA_FORMAT_TYPE_1 (1)
#define DATA_FORMAT_TYPE_2 (2)

#define DEBUG_LA66_UART_RECV
#define DEBUG_LA66_UART_RECV_PANEL_DATA
#define ESP_LA66_UART_BAUD (9600)

#define ACK_OK_STR "received\r\n"
#define ACK_TYPE_ERROR_STR "type error\r\n"

#define LA66_TTN_CONNECTION_ACTIVATE_STR "activate TTN\r\n"
#define LA66_ACTIVATE_SUCCEED_STR "An uplink for activation has been sent"
#define LA66_ACTIVATE_FAIL_LWAN_BUSY_STR "LWAN Busy"
#define LA66_ACTIVATE_FAIL_PARAM_ERROR_STR "Param Error"
#define LA66_ACTIVATE_FAIL_JOIN_FAILED_3_TIMES_STR "Join failed 3 times"

#define LA66_BUTTON_UPLINK_SUCCEED_STR "An uplink about button state change has been sent"
#define LA66_BUTTON_UPLINK_FAIL_LWAN_BUSY_STR "LWAN Busy"
#define LA66_BUTTON_UPLINK_FAIL_PARAM_ERROR_STR "Param Error"

// #define GET_LA66_JOIN_STATUS_STR "get join status\r\n"
// #define LA66_JOINED_STR "Joined"//\r\n
// #define LA66_NOT_JOINED_STR "Not Joined"//\r\n

#define GET_LA66_FW_TYPE_STR "fw type get\r\n"
#define LA66_FW_TYPE_CLASS_C_STR "CLASS C"
#define LA66_FW_TYPE_P2P_STR "P2P"

#define GET_LA66_CFG_STR "cfg get\r\n"
#define LA66_CFG_STR "la66 cfg"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

static const char * TAG_MAIN = "main";

static const uint8_t button_checked[] = {0x05, 0x00};
static const uint8_t button_unchecked[] = {0x05, 0x01};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

typedef struct {
	uint8_t encription[ENC_LEN];
	uint8_t dev_name[DEV_NAME_LEN + 1];
	uint8_t dev_eui[DEV_EUI_LEN];
	uint8_t mod_type[MOD_TYPE_LEN];
	uint8_t bat[BAT_LEN];
	uint8_t tem1[TEMP1_LEN];
	uint8_t tem2[TEMP2_LEN];
	uint8_t hum[HUMIDITY_LEN];

	uint8_t fport[FPORT_LEN];
} TemHum_type; // data_format_type: 1

typedef struct {
	uint8_t encription[ENC_LEN];
	uint8_t dev_name[DEV_NAME_LEN + 1];
	uint8_t dev_eui[DEV_EUI_LEN];
	uint8_t mod_type[MOD_TYPE_LEN];
	uint8_t bat[BAT_LEN];
	uint8_t status[STATUS_LEN];
	uint8_t switch_type;
	
	uint8_t fport[FPORT_LEN];
} Switch_type; // data_format_type: 2

typedef struct {
	TemHum_type TemHum;
	Switch_type Switch;
} Panel_Arr_t;

// 定义字段描述结构
// Define field description structure
typedef struct
{
    size_t len;
    uint8_t *dest;
    const char *name;//
    bool is_string;
} field_desc_t;

typedef enum {
	STATE_RECEIVE_PANEL_DATA,
	STATE_RECEIVE_CMD_RESPONSE_DATA,
	STATE_RECEIVE_BUTTON_RESPONSE_DATA
} UART_State_t;

typedef enum {
	CMD_RESP_RECV_UNKNOWN_FLAG = 0,
	CMD_RESP_RECV_P2P_FLAG = 1,
	CMD_RESP_RECV_CLASS_C_FLAG = 2,
	CMD_RESP_RECV_JOINED_STATUS_FLAG = 3,
	CMD_RESP_RECV_NOT_JOINED_STATUS_FLAG = 4,
	CMD_RESP_RECV_LA66_CFG_FLAG = 5,
	CMD_RESP_RECV_ACTIVATE_SUCCEED_FLAG = 6,
	CMD_RESP_RECV_ACTIVATE_FAIL_LWAN_BUSY_FLAG = 7,
	CMD_RESP_RECV_ACTIVATE_FAIL_PARAM_ERROR_FLAG = 8,
	CMD_RESP_RECV_ACTIVATE_FAIL_JOIN_FAILED_3_TIMES_FLAG = 9,
} UART_Cmd_Resp_t;

typedef enum {
	BUTTON_UPLINK_RESP_RECV_UNKNOWN_FLAG = 0,
	BUTTON_UPLINK_RESP_RECV_SUCCEED_FLAG = 1,
	BUTTON_UPLINK_RESP_RECV_FAIL_LWAN_BUSY_FLAG = 2,
	BUTTON_UPLINK_RESP_RECV_FAIL_PARAM_ERROR_FLAG = 3,
} UART_Button_Uplink_Resp_t;

typedef enum {
	LA66_NULL_TYPE = 0,
	LA66_P2P_FW_TYPE = 1,
	LA66_CLASS_C_FW_TYPE = 2,
	LA66_UNKNOWN_FW_TYPE = 3,
} LA66_Fw_Type_t;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static bool lvgl_tick_task_idle_flag = false;
static uint8_t la66_fport_151_data[1024] = {0};

static UART_State_t uart_state = STATE_RECEIVE_PANEL_DATA;
static UART_Cmd_Resp_t uart_cmd_resp = CMD_RESP_RECV_UNKNOWN_FLAG;
static UART_Button_Uplink_Resp_t uart_button_uplink_resp = BUTTON_UPLINK_RESP_RECV_UNKNOWN_FLAG;

static LA66_Fw_Type_t fw_type = LA66_NULL_TYPE;
// static uint8_t fw_type_exec_once = 1;

/* -------------------------------------------------------------------------- */
/* ----------- FUNCTIONS DECLARATION ---------------------------------------- */

static void Touch_IO_RST(void);
static void lvgl_hardWare_init(void);

static void uart_init(int baud);
static void bl_pwn_init(void);
static void boot_args_init(void);

static bool wait_recv_done(int8_t wait_time_sec, UART_State_t uart_state_before_recv_done);
static int power(int base, int exponent);
static int raw_data_to_Rssi(uint8_t *raw_data, int raw_data_len);
static int raw_data_to_data(uint8_t *raw_data, int raw_data_len, uint8_t *data);
static void type_init(void *panel_pointer, uint8_t data_format_type, uint8_t panel_type);


void esp_uart_recv_task(void *arg);
void esp_uart_send_task(void *arg);

static void config_store(void);
void config_store_task(void *arg);

void lv_tick_task(void *arg);

void esp_uart_button_status_switch_task(void *arg);

/* -------------------------------------------------------------------------- */
/* ------------ FUNCTIONS DEFINITION ---------------------------------------- */

static void Touch_IO_RST(void) 
{
#if(GT911==1)
	//touch reset pin     Low level reset
	gpio_reset_pin(39);
	gpio_reset_pin(40);
	gpio_pullup_en(40);
	gpio_pullup_en(39);

	gpio_set_direction(39, GPIO_MODE_OUTPUT);
	gpio_set_direction(40, GPIO_MODE_OUTPUT);
	gpio_set_level(40, 1);
	gpio_set_level(39, 1);
	ESP_LOGI(TAG_MAIN, "io39 set_high");
	vTaskDelay(pdMS_TO_TICKS(50));
	gpio_pulldown_en(39);
	gpio_pulldown_en(40);
	gpio_set_level(39, 0);
	gpio_set_level(40, 0);
	ESP_LOGI(TAG_MAIN, "io39 set_low");

	vTaskDelay(pdMS_TO_TICKS(20));
	gpio_set_level(40, 1);
	vTaskDelay(pdMS_TO_TICKS(20));
	gpio_pulldown_en(39);
	gpio_set_level(39, 0);
	//gpio_reset_pin(39);
	//gpio_set_direction(39, GPIO_MODE_INPUT);//The interrupt pin is not used, it is only used to configure the address
	
#elif(CST3240==1)

    gpio_reset_pin(39);
	gpio_reset_pin(40);
	
	gpio_set_direction( GPIO_NUM_40, GPIO_MODE_OUTPUT);//RST SET PORT OUTPUT
	gpio_set_level(40, 0);        //RST RESET IO
	vTaskDelay(pdMS_TO_TICKS(50));//DELAY 50ms 
	gpio_set_level(40, 1);        //SET RESET IO
	vTaskDelay(pdMS_TO_TICKS(10));//DELAY 10ms 	
#endif
}

// Peripheral hardware initialization
static void lvgl_hardWare_init(void) 
{
    ESP_ERROR_CHECK(bsp_i2c_init(I2C_NUM_0, 400000));
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();
    lv_port_tick_init();
}

static void uart_init(int baud) 
{
	uart_config_t uart_config = 
	{
		.baud_rate = baud,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // UART_HW_FLOWCTRL_CTS_RTS
		//.rx_flow_ctrl_thresh = 122,
		.source_clk = UART_SCLK_DEFAULT,
	};

	// Setup UART buffered IO with event queue
	const int uart_buffer_size = (1024 * 8);
	QueueHandle_t uart_queue;
	// Install UART driver using an event queue here
	ESP_ERROR_CHECK(uart_driver_install(COM_UART, uart_buffer_size,
										uart_buffer_size, 1024, &uart_queue, 0));

	// Configure UART parameters
	ESP_ERROR_CHECK(uart_param_config(COM_UART, &uart_config));

	// Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
	ESP_ERROR_CHECK(uart_set_pin(COM_UART, 17, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void bl_pwn_init(void) 
{
	ledc_timer_config_t pwn_config = 
	{
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_num = LEDC_TIMER_0,
		.duty_resolution = LEDC_TIMER_10_BIT,
		.freq_hz = 5000,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ledc_timer_config(&pwn_config);

	ledc_channel_config_t pwm_channel = 
	{
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_0,
		.timer_sel = LEDC_TIMER_0,
		.intr_type = LEDC_INTR_FADE_END,
		.gpio_num = GPIO_NUM_1, 
		.duty = 690,
		.hpoint = 0,
	};
	ledc_channel_config(&pwm_channel);
}

static void boot_args_init(void) 
{
	// Initialize NVS
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
	err = nvs_open("setting_args", NVS_READONLY, &my_handle); 
	if(err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\r\n", esp_err_to_name(err));
		printf("nvs setting_args addr %u\r\n", (unsigned int)my_handle);
    }
	else {
        printf("NVS open without error\r\n");
	}
	
	int32_t brightness = 9;
	err = nvs_get_i32(my_handle, "brightness", &brightness);
	printf("brightness: %ld\r\n", brightness);
	switch(err) {
		case ESP_OK: printf("brightness value read done\r\n"); break;
		case ESP_ERR_NVS_NOT_FOUND: printf("The brightness value is not initialized yet!\r\n"); break;
		default : printf("Error (%s) reading!\r\n", esp_err_to_name(err)); break;
	}

	uint16_t tem_unit = 0;
	err = nvs_get_u16(my_handle, "tem_unit", &tem_unit);
	printf("tem_unit: %d\r\n", tem_unit);
	switch (err) { 
		case ESP_OK: printf("tem_unit value read done\r\n"); break;
		case ESP_ERR_NVS_NOT_FOUND: printf("The tem_unit value is not initialized yet!\r\n"); break;
		default : printf("Error (%s) reading!\r\n", esp_err_to_name(err)); break;
	}
		
	uint16_t boot_screen = 0;
	err = nvs_get_u16(my_handle, "boot_screen", &boot_screen);
	printf("boot_screen: %d\r\n", boot_screen);
	switch (err) {
		case ESP_OK: printf("boot_screen value read done\r\n"); break;
		case ESP_ERR_NVS_NOT_FOUND: printf("The boot_screen value is not initialized yet!\r\n"); break;
		default : printf("Error (%s) reading!\r\n", esp_err_to_name(err)); break;
	}

	// uint16_t la66_cfg_font_index = 0;
	// err = nvs_get_u16(my_handle, "la66_cfg_font_index", &la66_cfg_font_index);
	// printf("la66_cfg_font_index: %d\r\n", la66_cfg_font_index);
	// switch (err) {
	// 	case ESP_OK: printf("la66_cfg_font_index value read done\r\n"); break;
	// 	case ESP_ERR_NVS_NOT_FOUND: printf("The la66_cfg_font_index value is not initialized yet!\r\n"); break;
	// 	default : printf("Error (%s) reading!\r\n", esp_err_to_name(err)); break;
	// }

	uint16_t fport_display = 0;
	err = nvs_get_u16(my_handle, "fport_display", &fport_display);
	printf("fport_display: %d\r\n", fport_display);
	switch (err) {
		case ESP_OK: printf("fport_display value read done\r\n"); break;
		case ESP_ERR_NVS_NOT_FOUND: printf("The fport_display value is not initialized yet!\r\n"); break;
		default : printf("Error (%s) reading!\r\n", esp_err_to_name(err)); break;
	}

	// close nvs
	nvs_close(my_handle);

	/*set brightness*/
	lvgl_lock_get();
	lv_slider_set_value(ui_SliderBrightnessAdjustment, brightness, LV_ANIM_OFF);
	lvgl_lock_release();
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, BRIGHTNESS_MIN + brightness * BRIGHTNESS_STEP);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

	/*set tem unit*/
	lvgl_lock_get();
	lv_dropdown_set_selected(ui_DropdownTemUnit, tem_unit);
	lvgl_lock_release();

	/*set boot screen*/
	if(boot_screen == 0) 
	{
		lvgl_lock_get();
		lv_disp_load_scr(ui_ScreenBoot);
		lvgl_lock_release();
		ESP_LOGD(TAG_MAIN,"Screen Boot at 0");
	}
	else if(boot_screen == 1) 
	{
		lvgl_lock_get();
		lv_disp_load_scr(ui_ScreenMain);
		lvgl_lock_release();
		ESP_LOGD(TAG_MAIN,"Screen Boot at 1");
	}
	lvgl_lock_get();
	lv_dropdown_set_selected(ui_DropdownBootAnimationSwitch, boot_screen);
	lvgl_lock_release();
	
	// /*set font size la66 cfg*/
	// lvgl_lock_get();
	// lv_dropdown_set_selected(ui_DropdownFontSizeLoRaParams, la66_cfg_font_index);
	// lv_obj_set_style_text_font(ui_TextAreaDialogLoRaParams, font_size_address_table[la66_cfg_font_index], LV_PART_MAIN | LV_STATE_DEFAULT);
	// lvgl_lock_release();

	/*set fport display*/
	lvgl_lock_get();
	lv_dropdown_set_selected(ui_DropdownFportDisplaySwitch, fport_display);
	lvgl_lock_release();
}

static bool wait_recv_done(int8_t wait_time_sec, UART_State_t uart_state_before_recv_done)
{
	// wait last recv finish
	bool exec_succeed_flag = true;
	// wait with timeout until cmd response is received
	while(uart_state == uart_state_before_recv_done) {
		vTaskDelay((1000) / portTICK_PERIOD_MS);
		if(wait_time_sec-- < 0) {
			exec_succeed_flag = false;
			uart_state = STATE_RECEIVE_PANEL_DATA;

			break;
		}
	}

	return exec_succeed_flag;
}

static int power(int base, int exponent) 
{
    int result = 1;  
    while (exponent > 0) 
	{
        if(exponent % 2 == 1) 
		{
            result *= base;  
        }  
        base *= base;  
        exponent /= 2;  
    }  
    return result;  
}

static int raw_data_to_Rssi(uint8_t *raw_data, int raw_data_len) 
{
	int data_begin_index = 0;
	int data_end_index = 0;
	int rssi = 0;
	int rssi_data_len = 0;
	for(int i = 0; i < raw_data_len; i++) 
	{
		if(*(raw_data + i) == (uint8_t)'R' && *(raw_data + (i + 1)) == (uint8_t)'s' \
		&& *(raw_data + (i + 2)) == (uint8_t)'s' && *(raw_data + (i + 3)) == (uint8_t)'i' \
		&& *(raw_data + (i + 4)) == (uint8_t)'=' && *(raw_data + (i + 5)) == (uint8_t)' ') 
		{
			data_begin_index = i + 6;
		}
	}
	for(int j = data_begin_index; j < raw_data_len; j++) 
	{
		if(*(raw_data + j) == 0x0D && *(raw_data + (j + 1)) == 0x0A) 
		{
			data_end_index = j - 1;
		}
	}
	rssi_data_len = data_end_index - data_begin_index + 1;
	if(*(raw_data + data_begin_index) == (uint8_t)'-')
	{
		rssi_data_len -= 1;
	}
	for(int k = data_end_index;k >= data_end_index - rssi_data_len + 1;k--) 
	{
		rssi += (*(raw_data + k) - 0x30) * power(10, data_end_index - k);
	}
	if(*(raw_data + data_begin_index) == (uint8_t)'-') 
	{
		rssi = 0 - rssi;
	}

	return rssi; 
}

static int raw_data_to_data(uint8_t *raw_data, int raw_data_len, uint8_t *data) 
{
	int data_begin_index = 0;
	int data_end_index = 0;
	uint8_t data_tmp1[1024] = {0};
	int data_tmp1_real_len = 0;
	uint8_t data_tmp2[1024] = {0};
	int data_tmp2_real_len = 0;
	int data_real_len = 0;
	char char_tmp1[3] = {0};
	char char_tmp2[3] = {0};
	if(strchr((char *)raw_data, ')') == NULL || strchr((char *)raw_data, 'R') == NULL) 
	{
		// 处理错误情况
		ESP_LOGD(TAG_MAIN, "left edge or right edge not found");
		return 0;
	}
	ESP_LOGI(TAG_MAIN, "raw_data len:%d", raw_data_len);
	for(int i = 0; i < raw_data_len; i++) 
	{
		if(raw_data[i] == (uint8_t)')') 
		{
			data_begin_index = i + 1;
			// ESP_LOGD(TAG_MAIN, "index begin found:%d", data_begin_index);
		}
		if((i + 3) < raw_data_len && raw_data[i] == (uint8_t)'R' && raw_data[i + 1] == (uint8_t)'s' && raw_data[i + 2] == (uint8_t)'s' && raw_data[i + 3] == (uint8_t)'i') 
		{
			data_end_index = i - 6; 
			// ESP_LOGD(TAG_MAIN, "index end found:%d", data_end_index);
		}
	}
	vTaskDelay(10 / portTICK_PERIOD_MS);
	ESP_LOGD(TAG_MAIN, "index_begin:%d  index_end:%d", (int)data_begin_index, (int)data_end_index);
	for(int j = 0; j < raw_data_len; j++) 
	{
		if(j >= data_begin_index && j <= data_end_index) 
		{
			data_tmp1[j - data_begin_index] = raw_data[j];
		}
		else if(j == data_end_index + 1) 
		{
			data_tmp1_real_len = j - data_begin_index;
		}
	}
	vTaskDelay(10 / portTICK_PERIOD_MS);
	for(int k = 0, cnt1 = 0; k < data_tmp1_real_len; k++) 
	{
		if(k % 3 == 2) 
		{
			char_tmp1[0] = (char)data_tmp1[k - 1];
			char_tmp1[1] = (char)data_tmp1[k];
			char_tmp1[2] = '\0';
			data_tmp2[cnt1] = (uint8_t)strtol(char_tmp1, NULL, 16);
			cnt1++;
			char_tmp1[0] = '\0';
			char_tmp1[1] = '\0';
			char_tmp1[2] = '\0';
			if(k == data_tmp1_real_len - 1) 
			{
				data_tmp2_real_len = cnt1;
			}
		}
	}
	vTaskDelay(10 / portTICK_PERIOD_MS);
	for(int m = 0, cnt2 = 0; m < data_tmp2_real_len; m++) 
	{
		if(m % 2 == 1) 
		{
			char_tmp2[0] = (char)data_tmp2[m - 1];
			char_tmp2[1] = (char)data_tmp2[m];
			char_tmp2[2] = '\0';
			data[cnt2] = (uint8_t)strtol(char_tmp2, NULL, 16);
			cnt2++;
			char_tmp2[0] = '\0';
			char_tmp2[1] = '\0';
			char_tmp2[2] = '\0';
			if(m == data_tmp2_real_len - 1) 
			{
				data_real_len = cnt2;
			}
		}
	}
	vTaskDelay(10 / portTICK_PERIOD_MS);
	return data_real_len;
}

static void type_init(void *panel_pointer, uint8_t data_format_type, uint8_t panel_type) 
{
	switch(data_format_type) 
	{
		case DATA_FORMAT_TYPE_1: 
		{
			TemHum_type * panel = (TemHum_type *)panel_pointer;
			memset((void *)(panel->encription), NON_ENCRYPTION, sizeof(uint8_t)*ENC_LEN);
			memset((void *)(panel->dev_name), 0, sizeof(uint8_t)*(DEV_NAME_LEN + 1));
			memset((void *)(panel->dev_eui), 0, sizeof(uint8_t)*DEV_EUI_LEN);
			memset((void *)(panel->mod_type), panel_type, sizeof(uint8_t)*MOD_TYPE_LEN);
			memset((void *)(panel->bat), 0, sizeof(uint8_t)*BAT_LEN);
			memset((void *)(panel->tem1), 0, sizeof(uint8_t)*TEMP1_LEN);
			memset((void *)(panel->tem2), 0, sizeof(uint8_t)*TEMP2_LEN);
			memset((void *)(panel->hum), 0, sizeof(uint8_t)*HUMIDITY_LEN);
			memset((void *)(panel->fport), 0, sizeof(uint8_t)*FPORT_LEN);
			
			break;
		}
		case DATA_FORMAT_TYPE_2: 
		{
			Switch_type * panel = (Switch_type *)panel_pointer;
			memset((void *)(panel->encription), NON_ENCRYPTION, sizeof(uint8_t)*ENC_LEN);
			memset((void *)(panel->dev_name), 0, sizeof(uint8_t)*(DEV_NAME_LEN + 1));
			memset((void *)(panel->dev_eui), 0, sizeof(uint8_t)*DEV_EUI_LEN);
			memset((void *)(panel->mod_type), panel_type, sizeof(uint8_t)*MOD_TYPE_LEN);
			memset((void *)(panel->bat), 0, sizeof(uint8_t)*BAT_LEN);
			memset((void *)(panel->status), 0, sizeof(uint8_t)*STATUS_LEN);
			memset((void *)(panel->fport), 0, sizeof(uint8_t)*FPORT_LEN);
			
			break;
		}
	}
}

// 统一数据处理函数
// Unified data processing function
static void process_device_data(uint8_t *data, size_t data_len, int32_t data_rssi, uint8_t device_type, Panel_Arr_t *panel_arr_p)
{
	switch(fw_type) {
		case LA66_P2P_FW_TYPE:
			ESP_LOGD(TAG_MAIN, "firmware type [P2P]");
			break;
		case LA66_CLASS_C_FW_TYPE:
			ESP_LOGD(TAG_MAIN, "firmware type [CLASS C]");
			break;
		case LA66_UNKNOWN_FW_TYPE:
			ESP_LOGD(TAG_MAIN, "firmware type [UNKNOWN]");
			break;
		default:
			ESP_LOGD(TAG_MAIN, "firmware type [UNKNOWN]");
			break;
	}
	
    // 公共字段描述符 (所有设备类型共有)
	// Common field descriptor (shared by all device types)
    field_desc_t common_fields[] = {{ENC_LEN, NULL, "encription", false},
        							{DEV_NAME_LEN, NULL, "dev_name", true},
        							{DEV_EUI_LEN, NULL, "dev_eui", false},
        							{MOD_TYPE_LEN, NULL, "mod_type", false},
        							{BAT_LEN, NULL, "bat", false}};

    const int common_field_count = sizeof(common_fields) / sizeof(common_fields[0]);
	

	field_desc_t TemHum_fields[] = {{TEMP1_LEN, panel_arr_p->TemHum.tem1, "tem1", false},
									{TEMP2_LEN, panel_arr_p->TemHum.tem2, "tem2", false},
            						{HUMIDITY_LEN, panel_arr_p->TemHum.hum, "hum", false}};

	const int temhum_field_count = sizeof(TemHum_fields) / sizeof(TemHum_fields[0]);
	

	field_desc_t Switch_fields[] = {{STATUS_LEN, panel_arr_p->Switch.status, "status", false}};

	const int switch_field_count = sizeof(Switch_fields) / sizeof(Switch_fields[0]);


	field_desc_t common_fields_2[] = {{FPORT_LEN, NULL, "fport", false}};

	const int common_field_count_2 = sizeof(common_fields_2) / sizeof(common_fields_2[0]);



    // 设备特定字段
	// Device specific fields
    field_desc_t *specific_fields = NULL;
    int specific_field_count = 0;
    void *device_panel = NULL;

    // 根据设备类型配置特定字段
	// Configure specific fields based on device type
    if (device_type == TEM_HUM_TYPE)
    {
        specific_fields = TemHum_fields;
        specific_field_count = temhum_field_count;//3
        device_panel = &panel_arr_p->TemHum;

        // 设置公共字段目标地址
		// Set public field target address
        common_fields[0].dest = panel_arr_p->TemHum.encription;
        common_fields[1].dest = panel_arr_p->TemHum.dev_name;
        common_fields[2].dest = panel_arr_p->TemHum.dev_eui;
        common_fields[3].dest = panel_arr_p->TemHum.mod_type;
        common_fields[4].dest = panel_arr_p->TemHum.bat;

		if(fw_type == LA66_CLASS_C_FW_TYPE) {
			common_fields_2[0].dest = panel_arr_p->TemHum.fport;
		}
    }
    else if (device_type > TEM_HUM_TYPE && device_type <= ALARM_TYPE)
    {
        specific_fields = Switch_fields;
        specific_field_count = switch_field_count;//1
        device_panel = &panel_arr_p->Switch;

        // 设置公共字段目标地址
		// Set public field target address
        common_fields[0].dest = panel_arr_p->Switch.encription;
        common_fields[1].dest = panel_arr_p->Switch.dev_name;
        common_fields[2].dest = panel_arr_p->Switch.dev_eui;
        common_fields[3].dest = panel_arr_p->Switch.mod_type;
        common_fields[4].dest = panel_arr_p->Switch.bat;

		if(fw_type == LA66_CLASS_C_FW_TYPE) {
			common_fields_2[0].dest = panel_arr_p->Switch.fport;
		}
    }

    // 计算总字段数
	// Calculate the total number of fields
    int total_fields = common_field_count + specific_field_count;//const 
	if(fw_type == LA66_CLASS_C_FW_TYPE) {
		total_fields += common_field_count_2;
	}
    field_desc_t all_fields[total_fields];

    // 合并字段数组
	// Merge field array
    memcpy(all_fields, common_fields, sizeof(common_fields));
    memcpy(all_fields + common_field_count, specific_fields, specific_field_count * sizeof(field_desc_t));
	if(fw_type == LA66_CLASS_C_FW_TYPE) {
		memcpy(all_fields + common_field_count + specific_field_count, common_fields_2, sizeof(common_fields_2));
	}
	
    size_t current_pos = 0; // 当前处理位置  // Current processing location

    // 处理所有字段
	// Process all fields
    for (int i = 0; i < total_fields; i++)
    {
        field_desc_t *f = &all_fields[i];

        // 边界检查
		// Bound checking
        if (current_pos + f->len > data_len)
        {
			ESP_LOGE(TAG_MAIN, "Process position %u; Bound %u", (unsigned int)(current_pos + f->len), (unsigned int)data_len);
            ESP_LOGE(TAG_MAIN, "Field %s out of range", f->name);
            break;
        }

        // 特殊处理字符串字段
		// Special handling of string fields
        if (f->is_string)
        {
            for (size_t j = 0; j < f->len; j++)
            {
                f->dest[j] = (data[current_pos + j] == 0xFF)
                                 ? 0
                                 : data[current_pos + j];
            }
            // // 确保字符串终止
			// // Ensure string termination
            // f->dest[f->len - 1] = '\0';
        }
        // 普通字段直接内存复制
		// Copy regular fields directly from memory
        else
        {
            memcpy(f->dest, data + current_pos, f->len);
        }

// 调试日志
// Debug Log
#ifdef DEBUG_LA66_UART_RECV_PANEL_DATA
        for (size_t j = 0; j < f->len; j++)
        {
            ESP_LOGD(TAG_MAIN, "%s[%d]:%02X", f->name, j, f->dest[j]);
        }
#endif

        current_pos += f->len; // 移动到下一字段  // Move to the next field

		if(fw_type == LA66_CLASS_C_FW_TYPE) {
			if (device_type > TEM_HUM_TYPE && device_type <= ALARM_TYPE)
			{
				if(strncmp(f->name, Switch_fields[0].name, strlen(Switch_fields[0].name)) == 0)
				{
					current_pos += (TEMP1_LEN + TEMP2_LEN + HUMIDITY_LEN - STATUS_LEN);
				}
			}
		}
    }

    // 组装消息
	// Assembly message
    message mes = {0};

    if (device_type == TEM_HUM_TYPE)
    {
        TemHum_type *panel = (TemHum_type *)device_panel;
        mes = (message){
            .dev_name = panel->dev_name,
            .dev_eui = panel->dev_eui,
            .mod = panel->mod_type[0],
            .tem1 = panel->tem1,
            .tem2 = panel->tem2,
            .hum = panel->hum,
            .bat = panel->bat,
			.fport = panel->fport[0],// .fport = panel->fport,
            .data_rssi = data_rssi};
    }
    else if (device_type > TEM_HUM_TYPE && device_type <= ALARM_TYPE)
    {
        Switch_type *panel = (Switch_type *)device_panel;
        mes = (message){
            .dev_name = panel->dev_name,
            .dev_eui = panel->dev_eui,
            .mod = panel->mod_type[0],
            .status = panel->status,
            .bat = panel->bat,
			.fport = panel->fport[0],// .fport = panel->fport,
            .data_rssi = data_rssi};
    }

    // 处理加密状态
	// Processing encryption status
    if (*((uint8_t *)device_panel) == NON_ENCRYPTION)
    { // 检查加密字段的第一个字节  // Check the first byte of the encrypted field
        lvgl_lock_get();
        panel_update(&mes, arr_get(), ui_PanelContainer);
        lvgl_lock_release();

        // 使用静态任务名
		// Use static task names
        static const char *TASK_NAME = "uart_send_ok";
        xTaskCreatePinnedToCore(esp_uart_send_task, TASK_NAME, 3072, (void *)ACK_OK_STR, 6, NULL, 1);
    }
}


void esp_uart_recv_task(void *arg) 
{
	uint8_t raw_data[RECEIVE_MAX_RAW_DATA_LEN] = {0};
	uint8_t data[MES_LEN_TYPE_1] = {0};
	Panel_Arr_t panel_arr = {0};
	
	int data_rssi = 0;
	int data_len = 0;

	int raw_size = 0;

	uint16_t packet_min_len = MES_LEN_TYPE_2 - FPORT_LEN;
	uint16_t packet_max_len = MES_LEN_TYPE_1;

	while (1) 
	{
		raw_size = uart_read_bytes(COM_UART, raw_data, sizeof(raw_data), 1500 / portTICK_PERIOD_MS);

		if(raw_size > 0) {
			ESP_LOGI(TAG_MAIN, "raw_data size:%d", raw_size);
			ESP_LOGI(TAG_MAIN, "raw_data(string):%s", raw_data); 
		}
		
		if(raw_size > 0 && !(raw_size == 1 && *raw_data == 0)) 
		{
			switch(uart_state)
			{
				case STATE_RECEIVE_PANEL_DATA:
					memset(data, 0, sizeof(data));
					data_rssi = raw_data_to_Rssi(raw_data, raw_size);
					data_len = raw_data_to_data(raw_data, raw_size, data);
					ESP_LOGI(TAG_MAIN, "data_len:%d", data_len);
					
					type_init(&panel_arr.TemHum, DATA_FORMAT_TYPE_1, NULL_TYPE);
					type_init(&panel_arr.Switch, DATA_FORMAT_TYPE_2, NULL_TYPE);

#if defined(DEBUG_LA66_UART_RECV)
					for(uint16_t i = 0;i < raw_size;i++) {
						ESP_LOGD(TAG_MAIN, "raw_data[%d]: %02X", i, raw_data[i]);
					}
#endif

					// 主处理逻辑
					if (data_len >= packet_min_len && data_len <= packet_max_len)
					{
						const size_t type_offset = ENC_LEN + DEV_NAME_LEN + DEV_EUI_LEN + MOD_TYPE_LEN - 1;
						const uint8_t device_type = data[type_offset];

						if (device_type == TEM_HUM_TYPE || (device_type > TEM_HUM_TYPE && device_type <= ALARM_TYPE))
						{
							process_device_data(data, data_len, data_rssi, device_type, &panel_arr);
						}
						else if (device_type == 0 || device_type > ALARM_TYPE)
						{
							static const char *TASK_NAME = "uart_send_type_err";
							xTaskCreatePinnedToCore(esp_uart_send_task, TASK_NAME, 3072, (void *)ACK_TYPE_ERROR_STR, 6, NULL, 1);
						}
					}

					if(raw_data[0] == 151 && raw_data[raw_size - 1] == 151)
					{
						memset(la66_fport_151_data, 0, sizeof(la66_fport_151_data));
						memcpy(la66_fport_151_data, raw_data + 1, raw_size - 2 > sizeof(la66_fport_151_data) ? sizeof(la66_fport_151_data) : raw_size - 2);
						ESP_LOGI(TAG_MAIN, "data from fport 151 is stored");
					}

					break;
				case STATE_RECEIVE_CMD_RESPONSE_DATA:
					if(strncmp((char *)raw_data, LA66_ACTIVATE_SUCCEED_STR, raw_size) == 0) {
						lvgl_lock_get();
						lv_label_set_text(ui_LabelMessageNoticeLoRaParams, "Activation successful"); 
						LoRaParams_Page_Notice_Message_Animation(ui_ContainerMessageNoticeLoRaParams, 1000, 2500);
						lv_dropdown_set_selected(ui_DropdownLoRaWANNetworkStatusLoRaParams, 1);
						lvgl_lock_release();
						uart_cmd_resp = CMD_RESP_RECV_ACTIVATE_SUCCEED_FLAG;
					}
					else if(strncmp((char *)raw_data, LA66_ACTIVATE_FAIL_LWAN_BUSY_STR, raw_size) == 0) {
						lvgl_lock_get();
						lv_label_set_text(ui_LabelMessageNoticeLoRaParams, "Network busy, please try again later");
						LoRaParams_Page_Notice_Message_Animation(ui_ContainerMessageNoticeLoRaParams, 1000, 2500);
						lv_dropdown_set_selected(ui_DropdownLoRaWANNetworkStatusLoRaParams, 0);
						lvgl_lock_release();
						uart_cmd_resp = CMD_RESP_RECV_ACTIVATE_FAIL_LWAN_BUSY_FLAG;
					}
					else if(strncmp((char *)raw_data, LA66_ACTIVATE_FAIL_PARAM_ERROR_STR, raw_size) == 0) {
						lvgl_lock_get();
						lv_label_set_text(ui_LabelMessageNoticeLoRaParams, "Parameter error, please try again later");
						LoRaParams_Page_Notice_Message_Animation(ui_ContainerMessageNoticeLoRaParams, 1000, 2500);
						lv_dropdown_set_selected(ui_DropdownLoRaWANNetworkStatusLoRaParams, 0);
						lvgl_lock_release();
						uart_cmd_resp = CMD_RESP_RECV_ACTIVATE_FAIL_PARAM_ERROR_FLAG;
					}
					else if(strncmp((char *)raw_data, LA66_ACTIVATE_FAIL_JOIN_FAILED_3_TIMES_STR, raw_size) == 0) {
						lvgl_lock_get();
						lv_label_set_text(ui_LabelMessageNoticeLoRaParams, "Join failed, please try again later");
						LoRaParams_Page_Notice_Message_Animation(ui_ContainerMessageNoticeLoRaParams, 1000, 2500);
						lv_dropdown_set_selected(ui_DropdownLoRaWANNetworkStatusLoRaParams, 0);
						lvgl_lock_release();
						uart_cmd_resp = CMD_RESP_RECV_ACTIVATE_FAIL_JOIN_FAILED_3_TIMES_FLAG;
					}
					else if(strncmp((char *)raw_data, LA66_FW_TYPE_CLASS_C_STR, strlen(LA66_FW_TYPE_CLASS_C_STR)) == 0) {
						char ver[16] = {'\0'};
						strncpy(ver, (char *)raw_data + strlen(LA66_FW_TYPE_CLASS_C_STR), raw_size - strlen(LA66_FW_TYPE_CLASS_C_STR));
						lvgl_lock_get();
						lv_label_set_text(ui_LabelMessageNoticeLoRaParams, "Successful acquisition of firmware information");
						LoRaParams_Page_Notice_Message_Animation(ui_ContainerMessageNoticeLoRaParams, 1000, 1500);
						lv_dropdown_set_selected(ui_DropdownCurrentFwLoRaParams, 2);
						lv_label_set_text_fmt(ui_LabelFwVersionLoRaParams, "%s", ver);//Version: 
						lv_obj_set_x(ui_LabelFwVersionLoRaParams, 0 - 10);
						lv_obj_add_flag(ui_PanelLoRaWANNetworkStatusHideLoRaParams, LV_OBJ_FLAG_HIDDEN);
						lv_obj_add_flag(ui_ButtonLoRaWANNetworkActivateLoRaParams, LV_OBJ_FLAG_CLICKABLE);
						// packet_min_len = MES_LEN_TYPE_2;
						lvgl_lock_release();
						ESP_LOGD(TAG_MAIN, "recv fw info: %s", (char *)raw_data);
						uart_cmd_resp = CMD_RESP_RECV_CLASS_C_FLAG;
					}
					else if(strncmp((char *)raw_data, LA66_FW_TYPE_P2P_STR, strlen(LA66_FW_TYPE_P2P_STR)) == 0) {
						char ver[16] = {'\0'};
						strncpy(ver, (char *)raw_data + strlen(LA66_FW_TYPE_P2P_STR), raw_size - strlen(LA66_FW_TYPE_P2P_STR));
						lvgl_lock_get();
						lv_label_set_text(ui_LabelMessageNoticeLoRaParams, "Successful acquisition of firmware information");
						LoRaParams_Page_Notice_Message_Animation(ui_ContainerMessageNoticeLoRaParams, 1000, 1500);
						lv_dropdown_set_selected(ui_DropdownCurrentFwLoRaParams, 1);
						lv_label_set_text_fmt(ui_LabelFwVersionLoRaParams, "%s", ver);//Version: 
						lv_obj_set_x(ui_LabelFwVersionLoRaParams, -50);
						lv_obj_clear_flag(ui_PanelLoRaWANNetworkStatusHideLoRaParams, LV_OBJ_FLAG_HIDDEN);
						lv_obj_clear_flag(ui_ButtonLoRaWANNetworkActivateLoRaParams, LV_OBJ_FLAG_CLICKABLE);
						// packet_min_len = MES_LEN_TYPE_2 - FPORT_LEN;
						lvgl_lock_release();
						ESP_LOGD(TAG_MAIN, "recv fw info: %s", (char *)raw_data);
						uart_cmd_resp = CMD_RESP_RECV_P2P_FLAG;
					}
					else if(strncmp((char *)raw_data, LA66_CFG_STR, strlen(LA66_CFG_STR)) == 0) { 
						lvgl_lock_get();
						lv_label_set_text(ui_LabelMessageNoticeLoRaParams, "LA66 module configuration information obtained successfully");
						LoRaParams_Page_Notice_Message_Animation(ui_ContainerMessageNoticeLoRaParams, 1000, 1500);
						lv_textarea_set_text(ui_TextAreaDialogLoRaParams, (char *)(raw_data + strlen(LA66_CFG_STR)));
						lvgl_lock_release();
						uart_cmd_resp = CMD_RESP_RECV_LA66_CFG_FLAG;
					}
					// else if(strncmp((char *)raw_data, LA66_JOINED_STR, raw_size) == 0) {
					// 	uart_cmd_resp = CMD_RESP_RECV_JOINED_STATUS_FLAG;
					// }
					// else if(strncmp((char *)raw_data, LA66_NOT_JOINED_STR, raw_size) == 0) {
					// 	uart_cmd_resp = CMD_RESP_RECV_NOT_JOINED_STATUS_FLAG;
					// }
					else {
						lvgl_lock_get();
						lv_label_set_text(ui_LabelMessageNoticeLoRaParams, "Received unknown message");
						LoRaParams_Page_Notice_Message_Animation(ui_ContainerMessageNoticeLoRaParams, 1000, 1500);
						lvgl_lock_release();
						uart_cmd_resp = CMD_RESP_RECV_UNKNOWN_FLAG;
					}

					uart_state = STATE_RECEIVE_PANEL_DATA;
					
					break;
				case STATE_RECEIVE_BUTTON_RESPONSE_DATA:
					if(strncmp((char *)raw_data, LA66_BUTTON_UPLINK_SUCCEED_STR, raw_size) == 0) {
						lvgl_lock_get();
						lv_label_set_text(ui_LabelMessageNoticeMain, "Button panel status transition message successfully uploaded");
						NewPanelArrive_Animation(ui_ContainerMessageNoticeMain, 1000);
						// lv_dropdown_set_selected(ui_DropdownLoRaWANNetworkStatusLoRaParams, 1);
						lvgl_lock_release();
						uart_button_uplink_resp = BUTTON_UPLINK_RESP_RECV_SUCCEED_FLAG;
					}
					else if(strncmp((char *)raw_data, LA66_BUTTON_UPLINK_FAIL_LWAN_BUSY_STR, raw_size) == 0) {
						lvgl_lock_get();
						lv_label_set_text(ui_LabelMessageNoticeMain, "Network busy, please try again later");
						NewPanelArrive_Animation(ui_ContainerMessageNoticeMain, 1000);
						// lv_dropdown_set_selected(ui_DropdownLoRaWANNetworkStatusLoRaParams, 0);
						lvgl_lock_release();
						uart_button_uplink_resp = BUTTON_UPLINK_RESP_RECV_FAIL_LWAN_BUSY_FLAG;
					}
					else if(strncmp((char *)raw_data, LA66_BUTTON_UPLINK_FAIL_PARAM_ERROR_STR, raw_size) == 0) {
						lvgl_lock_get();
						lv_label_set_text(ui_LabelMessageNoticeMain, "Parameter error, please try again later");
						NewPanelArrive_Animation(ui_ContainerMessageNoticeMain, 1000);
						// lv_dropdown_set_selected(ui_DropdownLoRaWANNetworkStatusLoRaParams, 0);
						lvgl_lock_release();
						uart_button_uplink_resp = BUTTON_UPLINK_RESP_RECV_FAIL_PARAM_ERROR_FLAG;
					}
					else {
						lvgl_lock_get();
						lv_label_set_text(ui_LabelMessageNoticeMain, "Received unknown message");
						NewPanelArrive_Animation(ui_ContainerMessageNoticeMain, 1000);
						lvgl_lock_release();
						uart_button_uplink_resp = BUTTON_UPLINK_RESP_RECV_UNKNOWN_FLAG;
					}

					uart_state = STATE_RECEIVE_PANEL_DATA;
					
					break;
				default: 
					break;
			}


		}

		// clear buffer
		uart_flush(COM_UART); 

		memset(raw_data, 0, sizeof(raw_data));
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}

void esp_uart_send_task(void *arg) 
{
	char * str_ack = (char *)arg;

	ESP_ERROR_CHECK(uart_wait_tx_done(COM_UART, 3000));//portMAX_DELAY
	
	uart_write_bytes(COM_UART, (const char*)str_ack, strlen(str_ack));
	ESP_LOGI(TAG_MAIN,"esp to la66 uart tx done");

	ESP_ERROR_CHECK(uart_wait_tx_done(COM_UART, 3000));//portMAX_DELAY

	vTaskDelay(50 / portTICK_PERIOD_MS);
	vTaskDelete(NULL);
}

void esp_uart_button_status_switch_task(void *arg) 
{
	DynamicArray * arr_temp = arr_get();
	char * str_temp = NULL;
	char str_send[16 + 2 + 2] = {'\0'};
	uint8_t str_send_len = sizeof(str_send) / sizeof(str_send[0]);

	while(1)
	{
		lvgl_lock_get();
		
		if(button_uplink_num_get() > 0) {
			for(uint16_t i = 0;i < arr_temp->size;i++) {
				if(arr_temp->array[i].panel_obj.panel_type == BUTTON_TYPE) {
					if(arr_temp->array[i].panel_obj.panel_union.button.button_press_flag == true) {
						str_temp = lv_label_get_text(arr_temp->array[i].panel_obj.panel_union.button.ui_LabelNameButton);
						ESP_LOGI(TAG_MAIN, "str_name:%s, sizeof(str_name):%d bytes", str_temp, strlen(str_temp));//strlen(str_temp) <= 16
						memset(str_send, '\0', sizeof(str_send));
						strncpy(str_send, str_temp, strlen(str_temp));

						if(arr_temp->array[i].panel_obj.panel_union.button.button_status == true) {
							str_send[strlen(str_temp)] = button_checked[0];
							str_send[strlen(str_temp) + 1] = button_checked[1] + 0x01;
						}
						else if(arr_temp->array[i].panel_obj.panel_union.button.button_status == false) {
							str_send[strlen(str_temp)] = button_unchecked[0];
							str_send[strlen(str_temp) + 1] = button_unchecked[1] + 0x01;
						}

						str_send[strlen(str_temp) + 2] = '\r';
						str_send[strlen(str_temp) + 3] = '\n';
						printf("str_send(strlen(str_temp) + 4: %d):", strlen(str_temp) + 4);
						for(uint8_t i = 0;i < str_send_len;i++) {
							printf("%d ", str_send[i]);
						}
						printf("\r\n");
						uart_state = STATE_RECEIVE_BUTTON_RESPONSE_DATA;
						ESP_ERROR_CHECK(uart_wait_tx_done(COM_UART, portMAX_DELAY));
						uart_write_bytes(COM_UART, (const void*)str_send, strlen(str_temp) + 4);
						ESP_ERROR_CHECK(uart_wait_tx_done(COM_UART, portMAX_DELAY));

						arr_temp->array[i].panel_obj.panel_union.button.button_press_flag = false;

						bool exec_succeed_flag = wait_recv_done(8, STATE_RECEIVE_BUTTON_RESPONSE_DATA);

						if(exec_succeed_flag == true) {
							if(uart_button_uplink_resp == BUTTON_UPLINK_RESP_RECV_SUCCEED_FLAG) {
								
							}
						}

					}
				}
			}

			button_uplink_num_set(0);
		}
		
		lvgl_lock_release();
		
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

static void config_store(void) 
{
	int32_t brightness = 1;
	uint16_t tem_unit = 1;
	uint16_t boot_screen = 0;
	uint16_t la66_cfg_font_index = 0;
	uint16_t fport_display = 0;

	/*get brightness Config*/
	lvgl_lock_get();
	brightness = lv_slider_get_value(ui_SliderBrightnessAdjustment);
	lvgl_lock_release();

	/*get tem uint Config*/
	lvgl_lock_get();
	tem_unit = lv_dropdown_get_selected(ui_DropdownTemUnit);
	lvgl_lock_release();

	/*get Boot Animation Config*/
	lvgl_lock_get();
	boot_screen = lv_dropdown_get_selected(ui_DropdownBootAnimationSwitch);
	lvgl_lock_release();

	// /*get Font Size LoRa Params Config*/
	// lvgl_lock_get();
	// la66_cfg_font_index = lv_dropdown_get_selected(ui_DropdownFontSizeLoRaParams);
	// lvgl_lock_release();

	/*get Fport Display Config*/
	lvgl_lock_get();
	fport_display = lv_dropdown_get_selected(ui_DropdownFportDisplaySwitch);
	lvgl_lock_release();

	nvs_store_complete_flag_set(false);
	while(lvgl_tick_task_idle_flag == false) 
	{
		vTaskDelay((1) / portTICK_PERIOD_MS);
	}
	config_save(brightness, tem_unit, boot_screen, la66_cfg_font_index, fport_display);
	nvs_store_complete_flag_set(true);
}

void config_store_task(void *arg) 
{
	while(1)
	{
		if(save_button_press_flag_get() == true) 
		{
			save_button_press_flag_set(false);
			ESP_LOGD(TAG_MAIN, "config save task execute!");
			config_store();
		}
		vTaskDelay((40) / portTICK_PERIOD_MS);
	}
}

void send_cmd_to_la66_task(void *arg)
{
	while(1)
	{
		if(activate_str_send_button_press_flag_get() == true)
		{
			activate_str_send_button_press_flag_set(false);
			uart_state = STATE_RECEIVE_CMD_RESPONSE_DATA;
			ESP_LOGD(TAG_MAIN, "send LoRaWAN network activation request task execute!");
			static const char *TASK_NAME = "uart_send_activate_ttn";
			xTaskCreatePinnedToCore(esp_uart_send_task, TASK_NAME, 2048, (void *)LA66_TTN_CONNECTION_ACTIVATE_STR, 6, NULL, 1);
			
			lvgl_lock_get();
			lv_dropdown_set_selected(ui_DropdownLoRaWANNetworkStatusLoRaParams, 2);
			lvgl_lock_release();
		}
		else if(fw_detect_str_send_button_press_flag_get() == true) 
		{
			fw_detect_str_send_button_press_flag_set(false);
			uart_state = STATE_RECEIVE_CMD_RESPONSE_DATA;
			ESP_LOGD(TAG_MAIN, "send firmware type get request task execute!");
			static const char *TASK_NAME = "uart_send_fw_type_get";
			xTaskCreatePinnedToCore(esp_uart_send_task, TASK_NAME, 2048, (void *)GET_LA66_FW_TYPE_STR, 6, NULL, 1);
			lvgl_lock_get();
			lv_dropdown_set_selected(ui_DropdownCurrentFwLoRaParams, 3);
			lv_label_set_text(ui_LabelFwVersionLoRaParams, "");
			lvgl_lock_release();
			
			bool exec_succeed_flag = wait_recv_done(8, STATE_RECEIVE_CMD_RESPONSE_DATA);
			
			if(exec_succeed_flag == false) {
				lvgl_lock_get();
				lv_dropdown_set_selected(ui_DropdownCurrentFwLoRaParams, 0);
				lvgl_lock_release();
			}

			if (exec_succeed_flag == true) {
			    if(uart_cmd_resp == CMD_RESP_RECV_P2P_FLAG || uart_cmd_resp == CMD_RESP_RECV_CLASS_C_FLAG) {
					uart_state = STATE_RECEIVE_CMD_RESPONSE_DATA;
					ESP_LOGD(TAG_MAIN, "send la66 config get request task execute!");
					static const char *TASK_NAME = "uart_send_cfg_get";
					xTaskCreatePinnedToCore(esp_uart_send_task, TASK_NAME, 2048, (void *)GET_LA66_CFG_STR, 6, NULL, 1);
				}
				else {
					lvgl_lock_get();
					lv_dropdown_set_selected(ui_DropdownCurrentFwLoRaParams, 0);
					lvgl_lock_release();
				}

				// if(fw_type_exec_once == 1) {
					if(uart_cmd_resp == CMD_RESP_RECV_P2P_FLAG) {
						fw_type = LA66_P2P_FW_TYPE;
					}
					else if(uart_cmd_resp == CMD_RESP_RECV_CLASS_C_FLAG) {
						fw_type = LA66_CLASS_C_FW_TYPE;
					}
					else {
						fw_type = LA66_UNKNOWN_FW_TYPE;
					}
				// 	fw_type_exec_once = 0;
				// }
			}
		}
		// else if(send_data_update_button_press_flag_get() == true) 
		// {
		// 	send_data_update_button_press_flag_set(false);
		// 	uart_state = STATE_RECEIVE_CMD_RESPONSE_DATA;
		// 	ESP_LOGD(TAG_MAIN, "send data update request task execute!");
		// 	xTaskCreatePinnedToCore(esp_uart_send_task, "esp_uart_send_task", 2048, (void *)GET_LA66_CFG_STR, 6, NULL, 1);
		// }
		vTaskDelay((40) / portTICK_PERIOD_MS);
	}
}

void exec_at_startup_task(void *arg) 
{
	vTaskDelay((5000) / portTICK_PERIOD_MS);

	fw_detect_str_send_button_press_flag_set(true);
	
	int8_t timeout_cnt = 12;
	while(fw_type == LA66_NULL_TYPE) {
		vTaskDelay((1000) / portTICK_PERIOD_MS);
		if(timeout_cnt-- < 0) break;
	}

	vTaskDelay((5000) / portTICK_PERIOD_MS);// wait for la66 cfg response info to be received completely
	
	if(fw_type == LA66_CLASS_C_FW_TYPE)
		activate_str_send_button_press_flag_set(true);
		
	vTaskDelete(NULL);
}

// LVGL clock task
void lv_tick_task(void *arg) 
{
	vTaskDelay((400) / portTICK_PERIOD_MS);
	esp_lcd_rgb_panel_restart(panel_handle_for_avoid_screen_drift);  // avoid boot screen drift
    while(1) 
	{
        vTaskDelay((40) / portTICK_PERIOD_MS);

		lvgl_tick_task_idle_flag = false;

		lvgl_lock_get();
        lv_task_handler();
		lvgl_lock_release();

		lvgl_tick_task_idle_flag = true;
	}
}

void app_main(void) 
{
    Touch_IO_RST();
    lvgl_hardWare_init();
	uart_init(ESP_LA66_UART_BAUD);
	bl_pwn_init();
	ui_init();
	sort_init();
	boot_args_init();

	ESP_LOGI(TAG_MAIN, "all init finished");

	xTaskCreatePinnedToCore(esp_uart_recv_task, "esp_uart_recv_task", 1024 * 24, NULL, 6, NULL, 0);
	xTaskCreatePinnedToCore(lv_tick_task, "lv_tick_task", 1024 * 24, NULL, 7, NULL, 0);
	xTaskCreatePinnedToCore(config_store_task, "config_store_task", 1024 * 4, NULL, 6, NULL, 1);
	xTaskCreatePinnedToCore(esp_uart_button_status_switch_task, "esp_uart_button_status_switch_task", 1024 * 4, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(send_cmd_to_la66_task, "send_cmd_to_la66_task", 1024 * 5, NULL, 6, NULL, 0);
	xTaskCreatePinnedToCore(exec_at_startup_task, "exec_at_startup_task", 1024, NULL, 6, NULL, 0);
}
