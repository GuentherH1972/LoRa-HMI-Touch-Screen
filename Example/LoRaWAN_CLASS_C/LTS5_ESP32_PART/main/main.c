/* -------------------------------------------------------------------------- */
/* --- DEPENDENCIES --------------------------------------------------------- */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "driver/uart.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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
#define STATUS_LEN     (1)
#define MES_LEN_TYPE_1 (ENC_LEN + DEV_NAME_LEN + DEV_EUI_LEN + MOD_TYPE_LEN + BAT_LEN + TEMP1_LEN + TEMP2_LEN + HUMIDITY_LEN)
#define MES_LEN_TYPE_2 (ENC_LEN + DEV_NAME_LEN + DEV_EUI_LEN + MOD_TYPE_LEN + BAT_LEN + STATUS_LEN)
#define RECEIVE_MAX_RAW_DATA_LEN (1024)
#define COM_UART UART_NUM_1
#define NON_ENCRYPTION (0x01)
#define DATA_FORMAT_TYPE_1 (1)
#define DATA_FORMAT_TYPE_2 (2)

#define PANEL_TYPE_NUM      ALARM_TYPE
#define DEBUG_UART
#define ESP_LA66_UART_BAUD (9600)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

static const char * TAG_MAIN = "main";
static const char ack_ok[] = "received\r\n";
static const char ack_type_error[] = "type error\r\n";

static uint8_t button_checked[] = {0x05, 0x00};
static uint8_t button_unchecked[] = {0x05, 0x01};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

typedef struct {
	uint8_t encription[ENC_LEN];
	uint8_t dev_name[DEV_NAME_LEN + 1];
	uint8_t dev_eui[DEV_EUI_LEN];
	uint8_t mod_type[MOD_TYPE_LEN];
	uint8_t bat[BAT_LEN];
	uint8_t temp1[TEMP1_LEN];
	uint8_t temp2[TEMP2_LEN];
	uint8_t humidity[HUMIDITY_LEN];
} TemHum_type; // data_format_type: 1

typedef struct {
	uint8_t encription[ENC_LEN];
	uint8_t dev_name[DEV_NAME_LEN + 1];
	uint8_t dev_eui[DEV_EUI_LEN];
	uint8_t mod_type[MOD_TYPE_LEN];
	uint8_t bat[BAT_LEN];
	uint8_t status[STATUS_LEN];
	uint8_t switch_type;
} Switch_type; // data_format_type: 2

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static bool lvgl_tick_task_idle_flag = false;
static uint8_t la66_fport_151_data[1024] = {0};

/* -------------------------------------------------------------------------- */
/* ----------- FUNCTIONS DECLARATION ---------------------------------------- */

static void Touch_IO_RST(void);
static void lvgl_hardWare_init(void);

static void uart_init(int baud);
static void bl_pwn_init(void);
static void boot_args_init(void);

static int power(int base, int exponent);
static int raw_data_to_Rssi(uint8_t *raw_data, int raw_data_len);
static int raw_data_to_data(uint8_t *raw_data, int raw_data_len, uint8_t *data);
static void type_init(void *panel_pointer, uint8_t data_format_type, uint8_t panel_type);


void esp_uart_task(void *arg);
void esp_uart_ack_task(void *arg);

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
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
	else
	{
        printf("Done\n");
	}
	
	int32_t brightness = 9;
	err = nvs_get_i32(my_handle, "brightness", &brightness);
	printf("brightness: %ld\r\n", brightness);
	switch(err) 
	{
		case ESP_OK:
			printf("Done\n");
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			printf("The value is not initialized yet!\n");
			break;
		default :
			printf("Error (%s) reading!\n", esp_err_to_name(err));
	}

	uint16_t tem_unit = 0;
	err = nvs_get_u16(my_handle, "tem_unit", &tem_unit);
	printf("tem_unit: %d\r\n", tem_unit);
	switch (err) 
	{
		case ESP_OK:
			printf("Done\n");
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			printf("The value is not initialized yet!\n");
			break;
		default :
			printf("Error (%s) reading!\n", esp_err_to_name(err));
	}
		
	uint16_t boot_screen = 0;
	err = nvs_get_u16(my_handle, "boot_screen", &boot_screen);
	printf("boot_screen: %d\r\n", boot_screen);
	switch (err) 
	{
		case ESP_OK:
			printf("Done\n");
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			printf("The value is not initialized yet!\n");
			break;
		default :
			printf("Error (%s) reading!\n", esp_err_to_name(err));
	}

	// Close
	nvs_close(my_handle);

	/*set brightness*/
	lvgl_lock_get();
	lv_slider_set_value(ui_SliderBrightnessAdjustment, brightness, LV_ANIM_OFF);
	lvgl_lock_release();
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, BRIGHTNESS_start + brightness * BRIGHTNESS_STEP);
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
		ESP_LOGE(TAG_MAIN,"Screen Boot at 0");
	}
	else if(boot_screen == 1) 
	{
		lvgl_lock_get();
		lv_disp_load_scr(ui_ScreenMain);
		lvgl_lock_release();
		ESP_LOGE(TAG_MAIN,"Screen Boot at 1");
	}
	lvgl_lock_get();
	lv_dropdown_set_selected(ui_DropdownBootAnimationSwitch, boot_screen);
	lvgl_lock_release();
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
		ESP_LOGI(TAG_MAIN, "left edge or right edge not found");
		return 0;
	}
	ESP_LOGI(TAG_MAIN, "raw_data len:%d", raw_data_len);
	for(int i = 0; i < raw_data_len; i++) 
	{
		if(raw_data[i] == (uint8_t)')') 
		{
			data_begin_index = i + 1;
			ESP_LOGI(TAG_MAIN, "index begin found:%d", data_begin_index);
		}
		if((i + 3) < raw_data_len && raw_data[i] == (uint8_t)'R' && raw_data[i + 1] == (uint8_t)'s' && raw_data[i + 2] == (uint8_t)'s' && raw_data[i + 3] == (uint8_t)'i') 
		{
			data_end_index = i - 6; 
			ESP_LOGI(TAG_MAIN, "index end found:%d", data_end_index);
		}
	}
	vTaskDelay(10 / portTICK_PERIOD_MS);
	ESP_LOGI(TAG_MAIN, "index_begin:%d  index_end:%d", (int)data_begin_index, (int)data_end_index);
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
			memset((void *)(panel->temp1), 0, sizeof(uint8_t)*TEMP1_LEN);
			memset((void *)(panel->temp2), 0, sizeof(uint8_t)*TEMP2_LEN);
			memset((void *)(panel->humidity), 0, sizeof(uint8_t)*HUMIDITY_LEN);
			
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
			
			break;
		}
	}
}

void esp_uart_task(void *arg) 
{
	while (1) 
	{
		uint8_t raw_data[RECEIVE_MAX_RAW_DATA_LEN] = {0};
		uint8_t data[MES_LEN_TYPE_1] = {0};
		memset(data, 0, MES_LEN_TYPE_1);
		int data_rssi = 0;
		int data_len = 0;
		int left = 0;
		int right = 0 + ENC_LEN;

		TemHum_type TemHum_panel;
		Switch_type Switch_panel;
		type_init(&TemHum_panel, DATA_FORMAT_TYPE_1, NULL_TYPE);
		type_init(&Switch_panel, DATA_FORMAT_TYPE_2, NULL_TYPE);


		int raw_size = uart_read_bytes(COM_UART, raw_data, RECEIVE_MAX_RAW_DATA_LEN, 1500 / portTICK_PERIOD_MS); // 接收数据,等待超时时间为10ms

		if(raw_size > 0) 
		{
			// #if defined(DEBUG_UART)
			ESP_LOGI(TAG_MAIN, "raw_data size:%d", raw_size);
			ESP_LOGI(TAG_MAIN, "raw_data(string):%s", raw_data); 

			for(uint16_t i = 0;i < raw_size;i++) {
				ESP_LOGI(TAG_MAIN, "raw_data[%d]: %02X", i, raw_data[i]);
			}
			// #endif
			data_len = raw_data_to_data(raw_data, raw_size, data);

			// #if defined(DEBUG_UART)
			ESP_LOGI(TAG_MAIN, "data_len:%d", data_len);
			// #endif

			data_rssi = raw_data_to_Rssi(raw_data, raw_size);

			// data length check 
			if(data_len >= MES_LEN_TYPE_2 && data_len <= MES_LEN_TYPE_1) // in fact, all panel data from la66 is 34 bytes length
			{
				if(data[ENC_LEN + DEV_NAME_LEN + DEV_EUI_LEN + MOD_TYPE_LEN - 1] == TEM_HUM_TYPE) 
				{
					uint8_t flags[8] = {0};
					memset((void *)flags, 1, sizeof(flags));
					for(int i = 0; i < data_len; i++) 
					{
						if(i >= left && i < right && flags[0] == 1) 
						{
							TemHum_panel.encription[i - left] = data[i];
							ESP_LOGI(TAG_MAIN, "encription[%d]:%02X", i - left, TemHum_panel.encription[i - left]);
							if(i == right - 1) 
							{
								left += ENC_LEN;
								right += DEV_NAME_LEN;
								flags[0] = 0;
							}
						}
						else if(i >= left && i < right && flags[1] == 1) 
						{
							if(data[i] == 0xFF) 
							{
								TemHum_panel.dev_name[i - left] = 0;
							}
							else 
							{
								TemHum_panel.dev_name[i - left] = data[i];
							}
							ESP_LOGI(TAG_MAIN, "dev_name[%d]:%02X", i - left, TemHum_panel.dev_name[i - left]);
							if(i == right - 1) 
							{
								left += DEV_NAME_LEN;
								right += DEV_EUI_LEN;
								flags[1] = 0;
							}
						}
						else if(i >= left && i < right && flags[2] == 1) 
						{
							TemHum_panel.dev_eui[i - left] = data[i];
							ESP_LOGI(TAG_MAIN, "dev_eui[%d]:%02X", i - left, TemHum_panel.dev_eui[i - left]);
							if(i == right - 1) 
							{
								left += DEV_EUI_LEN;
								right += MOD_TYPE_LEN;
								flags[2] = 0;
							}
						}
						else if(i >= left && i < right && flags[3] == 1) 
						{
							TemHum_panel.mod_type[i - left] = data[i];
							ESP_LOGI(TAG_MAIN, "mod_type[%d]:%02X", i - left, TemHum_panel.mod_type[i - left]);
							if(i == right - 1) 
							{
								left += MOD_TYPE_LEN;
								right += BAT_LEN;
								flags[3] = 0;
							}
						}
						else if(i >= left && i < right && flags[4] == 1) 
						{
							TemHum_panel.bat[i - left] = data[i];
							ESP_LOGI(TAG_MAIN, "bat[%d]:%02X", i - left, TemHum_panel.bat[i - left]);
							if(i == right - 1) 
							{
								left += BAT_LEN;
								right += TEMP1_LEN;
								flags[4] = 0;
							}
						}
						else if(i >= left && i < right && flags[5] == 1) 
						{
							TemHum_panel.temp1[i - left] = data[i];
							ESP_LOGI(TAG_MAIN, "temp1[%d]:%02X", i - left, TemHum_panel.temp1[i - left]);
							if(i == right - 1) 
							{
								left += TEMP1_LEN;
								right += TEMP2_LEN;
								flags[5] = 0;
							}
						}
						else if(i >= left && i < right && flags[6] == 1) 
						{
							TemHum_panel.temp2[i - left] = data[i];
							ESP_LOGI(TAG_MAIN, "temp2[%d]:%02X", i - left, TemHum_panel.temp2[i - left]);
							if(i == right - 1) 
							{
								left += TEMP2_LEN;
								right += HUMIDITY_LEN;
								flags[6] = 0;
							}
						}
						else if(i >= left && i < right && flags[7] == 1) 
						{
							TemHum_panel.humidity[i - left] = data[i];
							ESP_LOGI(TAG_MAIN, "humidity[%d]:%02X", i - left, TemHum_panel.humidity[i - left]);
							if(i == right - 1) 
							{
								left = 0;
								right = 0 + ENC_LEN;
								flags[7] = 0;
							}
						}
					}

					message mes;
					mes.dev_name = TemHum_panel.dev_name;
					mes.dev_eui = TemHum_panel.dev_eui;
					mes.mod = TemHum_panel.mod_type[0];
					mes.temp1 = TemHum_panel.temp1;
					mes.temp2 = TemHum_panel.temp2;
					mes.humidity = TemHum_panel.humidity;
					mes.bat = TemHum_panel.bat;
					mes.data_rssi = data_rssi;

					if(TemHum_panel.encription[0] == NON_ENCRYPTION) 
					{
						lvgl_lock_get();
						panel_update(&mes, arr_get(), ui_PanelContainer);
						lvgl_lock_release();
						xTaskCreatePinnedToCore(esp_uart_ack_task, "esp_uart_ack_task", 3072, (void *)ack_ok, 6, NULL, 1);
					}
				}
				else if(data[ENC_LEN + DEV_NAME_LEN + DEV_EUI_LEN + MOD_TYPE_LEN - 1] > TEM_HUM_TYPE && data[ENC_LEN + DEV_NAME_LEN + DEV_EUI_LEN + MOD_TYPE_LEN - 1] <= ALARM_TYPE) 
				{
					uint8_t flags[6] = {0};
					memset((void *)flags, 1, sizeof(flags));
					for(int i = 0; i < data_len; i++) 
					{ // LA66发来的都是9个字节的数据（不足的部分会补零，以形成一个完整的9个字节的数据包，当然前面还有name, deui, encription，加上这些才是完整的34字节数据包），1.开关类型是5个字节的数据或者2.LoRaWAN随便下发的非5个字节但小于9个字节的数据是未规定的数据 都小于9个字节
						// ESP_LOGI(TAG_MAIN, "i value:%d", i);  // 上一行写i<data_len也是可以的，在这个循环中，当接受完5个字节的数据包后，循环在空转，不执行任何动作，因为不满足flags标志位为1的条件了

						if(i >= left && i < right && flags[0] == 1) 
						{
							Switch_panel.encription[i - left] = data[i];
							ESP_LOGI(TAG_MAIN, "encription[%d]:%02X", i - left, Switch_panel.encription[i - left]);
							if(i == right - 1) 
							{
								left += ENC_LEN;
								right += DEV_NAME_LEN;
								flags[0] = 0;
							}
						}
						else if(i >= left && i < right && flags[1] == 1) 
						{
							if(data[i] == 0xFF) 
							{
								Switch_panel.dev_name[i - left] = 0;
							}
							else 
							{
								Switch_panel.dev_name[i - left] = data[i];
							}
							ESP_LOGI(TAG_MAIN, "dev_name[%d]:%02X", i - left, Switch_panel.dev_name[i - left]);
							if(i == right - 1) 
							{
								left += DEV_NAME_LEN;
								right += DEV_EUI_LEN;
								flags[1] = 0;
							}
						}
						else if(i >= left && i < right && flags[2] == 1) 
						{
							Switch_panel.dev_eui[i - left] = data[i];
							ESP_LOGI(TAG_MAIN, "dev_eui[%d]:%02X", i - left, Switch_panel.dev_eui[i - left]);
							if(i == right - 1) 
							{
								left += DEV_EUI_LEN;
								right += MOD_TYPE_LEN;
								flags[2] = 0;
							}
						}
						else if(i >= left && i < right && flags[3] == 1) 
						{
							Switch_panel.mod_type[i - left] = data[i];
							ESP_LOGI(TAG_MAIN, "mod_type[%d]:%02X", i - left, Switch_panel.mod_type[i - left]);
							if(i == right - 1) 
							{
								left += MOD_TYPE_LEN;
								right += BAT_LEN;
								flags[3] = 0;
							}
						}
						else if(i >= left && i < right && flags[4] == 1) 
						{
							Switch_panel.bat[i - left] = data[i];
							ESP_LOGI(TAG_MAIN, "bat[%d]:%02X", i - left, Switch_panel.bat[i - left]);
							if(i == right - 1) 
							{
								left += BAT_LEN;
								right += STATUS_LEN;
								flags[4] = 0;
							}
						}
						else if(i >= left && i < right && flags[5] == 1) 
						{
							Switch_panel.status[i - left] = data[i];
							ESP_LOGI(TAG_MAIN, "status[%d]:%02X", i - left, Switch_panel.status[i - left]);
							if(i == right - 1) 
							{
								left = 0;
								right = 0 + ENC_LEN;
								flags[5] = 0;
							}
						}
					}

					message mes;
					mes.dev_name = Switch_panel.dev_name;
					mes.dev_eui = Switch_panel.dev_eui;
					mes.mod = Switch_panel.mod_type[0];
					mes.status = Switch_panel.status;
					mes.bat = Switch_panel.bat;
					mes.data_rssi = data_rssi;

					if(Switch_panel.encription[0] == NON_ENCRYPTION) 
					{
						lvgl_lock_get();
						panel_update(&mes, arr_get(), ui_PanelContainer);
						lvgl_lock_release();
						xTaskCreatePinnedToCore(esp_uart_ack_task, "esp_uart_ack_task", 3072, (void *)ack_ok, 6, NULL, 1);//3 5
					}
				}
				else if(data[ENC_LEN + DEV_NAME_LEN + DEV_EUI_LEN + MOD_TYPE_LEN - 1] == 0 || data[ENC_LEN + DEV_NAME_LEN + DEV_EUI_LEN + MOD_TYPE_LEN - 1] > ALARM_TYPE)
				{
					xTaskCreatePinnedToCore(esp_uart_ack_task, "esp_uart_ack_task", 3072, (void *)ack_type_error, 6, NULL, 1); // panel type in la66 panel information is error. it means panel type is unknown, rather than panel type is mismatched with data length in fact. 
				}
			}

			if(raw_data[0] == 151 && raw_data[raw_size - 1] == 151)
			{
				memset(la66_fport_151_data, 0, sizeof(la66_fport_151_data));
				memcpy(la66_fport_151_data, raw_data + 1, raw_size - 2 > sizeof(la66_fport_151_data) ? sizeof(la66_fport_151_data) : raw_size - 2);
				ESP_LOGI(TAG_MAIN, "data from fport 151 is stored");
			}
			
			uart_flush(COM_UART); // Clear buffer
		}
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}


void esp_uart_ack_task(void *arg) 
{
	char * str_ack = (char *)arg;

	ESP_ERROR_CHECK(uart_wait_tx_done(COM_UART, portMAX_DELAY));
	
	uart_write_bytes(COM_UART, (const char*)str_ack, strlen(str_ack));
	ESP_LOGI(TAG_MAIN,"esp ack la66 panel data tx done");

	ESP_ERROR_CHECK(uart_wait_tx_done(COM_UART, portMAX_DELAY));

	vTaskDelay(50 / portTICK_PERIOD_MS);
	vTaskDelete(NULL);
}

void esp_uart_button_status_switch_task(void *arg) 
{
	while(1)
	{
		lvgl_lock_get();
		DynamicArray * arr_temp = arr_get();
		if(button_uplink_num_get() > 0) {
			for(uint16_t i = 0;i < arr_temp->size;i++) {
				if(arr_temp->array[i].panel_obj.panel_type == BUTTON_TYPE) {
					if(arr_temp->array[i].panel_obj.panel_union.button.button_press_flag == true) {
						if(arr_temp->array[i].panel_obj.panel_union.button.button_status == true) {
							char * str_temp = lv_label_get_text(arr_temp->array[i].panel_obj.panel_union.button.ui_LabelNameButton);
							ESP_LOGI(TAG_MAIN, "str_name:%s, sizeof(str_name):%d bytes", str_temp, strlen(str_temp));
							char str_send[16+2+2] = {'\0'};
							strncpy(str_send, str_temp, strlen(str_temp));
							str_send[strlen(str_temp)] = button_checked[0];
							str_send[strlen(str_temp) + 1] = button_checked[1] + 0x01;
							str_send[strlen(str_temp) + 2] = '\r';
							str_send[strlen(str_temp) + 3] = '\n';
							printf("str_send(strlen(str_temp) + 4: %d):", strlen(str_temp) + 4);
							for(uint8_t i = 0;i<20;i++) {
								printf("%d ", str_send[i]);
							}
							printf("\r\n");
							ESP_ERROR_CHECK(uart_wait_tx_done(COM_UART, portMAX_DELAY));
							uart_write_bytes(COM_UART, (const void*)str_send, strlen(str_temp) + 4);
							ESP_ERROR_CHECK(uart_wait_tx_done(COM_UART, portMAX_DELAY));
						}
						else if(arr_temp->array[i].panel_obj.panel_union.button.button_status == false) {
							char * str_temp = lv_label_get_text(arr_temp->array[i].panel_obj.panel_union.button.ui_LabelNameButton);
							ESP_LOGI(TAG_MAIN, "str_name:%s, sizeof(str_name):%d bytes", str_temp, strlen(str_temp));
							char str_send[16+2+2] = {'\0'};
							strncpy(str_send, str_temp, strlen(str_temp));
							str_send[strlen(str_temp)] = button_unchecked[0];
							str_send[strlen(str_temp) + 1] = button_unchecked[1] + 0x01;
							str_send[strlen(str_temp) + 2] = '\r';
							str_send[strlen(str_temp) + 3] = '\n';
							printf("str_send(strlen(str_temp) + 4: %d):", strlen(str_temp) + 4);
							for(uint8_t i = 0;i<20;i++) {
								printf("%d ", str_send[i]);
							}
							printf("\r\n");
							ESP_ERROR_CHECK(uart_wait_tx_done(COM_UART, portMAX_DELAY));
							uart_write_bytes(COM_UART, (const void*)str_send, strlen(str_temp) + 4);
							ESP_ERROR_CHECK(uart_wait_tx_done(COM_UART, portMAX_DELAY));
						}
						arr_temp->array[i].panel_obj.panel_union.button.button_press_flag = false;
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

	nvs_store_complete_flag_set(false);
	while(lvgl_tick_task_idle_flag == false) 
	{
		vTaskDelay((1) / portTICK_PERIOD_MS);
	}
	config_save(brightness, tem_unit, boot_screen);
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

	ESP_LOGI(TAG_MAIN, "init ok");

	xTaskCreatePinnedToCore(esp_uart_task, "esp_uart_task", 4096*6, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(lv_tick_task, "lv_tick_task", 1024*48, NULL, 7, NULL, 0);//6 //有的时候会跳秒，即直接加两秒，猜想可能是lvgl定时任务的优先级没有设置成最高优先级
	xTaskCreatePinnedToCore(config_store_task, "config_store_task", 1024*4, NULL, 6, NULL, 1);//0
	xTaskCreatePinnedToCore(esp_uart_button_status_switch_task, "esp_uart_button_status_switch_task", 1024*6, NULL, 5, NULL, 0);
}
