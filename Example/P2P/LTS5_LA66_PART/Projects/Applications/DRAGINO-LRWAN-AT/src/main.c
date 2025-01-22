#include <stdio.h>
#include <string.h>
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "log.h"
#include "tremo_uart.h"
#include "tremo_lpuart.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"
#include "tremo_iwdg.h"
#include "tremo_delay.h"
#include "tremo_pwr.h"
#include "rtc-board.h"
#include "lora_app.h"
#include "bsp.h"
#include "command.h"
#include "lora_config.h"
#include "flash_eraseprogram.h"
#include "tremo_rtc.h"
#include "tremo_system.h"
#include "tremo_adc.h"
#include "sx126x.h"
#include "tremo_flash.h"

typedef struct
{
	/*point to the LoRa App data buffer*/
	uint8_t *Buff;
	/*LoRa App data buffer size*/
	uint8_t BuffSize;
} lora_AppData_t;

log_level_t g_log_level = LL_ERR | LL_WARN | LL_DEBUG;
__IO uint8_t EnterLowPowerStopModeStatus = 0, EnterLowPowerStopMode_error_times = 0;

int16_t rssi_value;
int8_t snr_value;
static lora_AppData_t AppData;
uint8_t ack_reply = 0;
bool data_check_flag = 0;
bool is_time_to_IWDG_Refresh = 0;
bool sending_flag = 0;
bool uplink_data_status = 0;
bool rx_waiting_flag = 0;
bool ack_data_status = 0;
uint32_t uplinkcount = 0;
uint32_t downlinkcount = 0;
uint8_t sendDataBuff[255];
uint8_t sendBufferSize = 0;
uint8_t *receiveDataBuff;
uint8_t receiveBufferSize = 0;
static uint8_t txDataBuff[255];
static uint8_t rxDataBuff[255];
static uint8_t rxrelayBuff[255];
static uint8_t txBufferSize = 0;
static uint8_t rxBufferSize = 0;
static uint8_t rxrelayBufferSize = 0;
uint8_t ack_flags = 0;
uint8_t retransmission_flags = 0;
uint8_t retransmission_temp = 0;

extern bool print_isdone(void);
extern uint8_t printf_dma_busy;
extern uint16_t printf_dma_idx_w;
extern uint16_t printf_dma_idx_r;

extern SysTime_t LastTxdoneTime;
extern uint8_t tx_groupmod_value;
extern uint8_t rx_groupmod_value;
extern uint32_t tx_signal_freqence;
extern uint32_t rx_signal_freqence;
extern uint8_t tx_bandwidth_value;
extern uint8_t rx_bandwidth_value;
extern uint8_t tx_spreading_value;
extern uint8_t rx_spreading_value;
extern uint8_t txp_value;
extern uint8_t tx_codingrate_value;
extern uint8_t rx_codingrate_value;
extern bool tx_header_value;
extern bool rx_header_value;
extern bool tx_crc_value;
extern bool rx_crc_value;
extern bool tx_IQ_value;
extern bool rx_IQ_value;
extern uint16_t tx_preamble_value;
extern uint16_t rx_preamble_value;
extern uint8_t syncword_value;
extern uint16_t rxmode_timeout;
extern uint8_t rxmode_ack;
extern bool rx_form_value;
extern uint32_t retun_wait_time;
extern bool debug_flags;

static void StartIWDGRefresh(void);
TimerEvent_t IWDGRefreshTimer;
static void OnIWDGRefreshTimeoutEvent(void);

static void LoraStartTx(void);
TimerEvent_t TxTimer;
static void OnTxTimerEvent(void);

static void AckStartTX(void);
TimerEvent_t AckTimer; // TDC
static void OnAckEvent(void);

void board_init();
void uart_log_init(uint32_t baudrate);
static void Send_TX(void);
static void Send_RX_message(uint8_t type);
static void RxData(lora_AppData_t *AppData);

static RadioEvents_t RadioEvents;
void lora_test_init(void);
static void test_OnTxDone(void);
static void test_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
static void test_OnTxTimeout(void);
static void test_OnRxTimeout(void);
static void test_OnRxError(void);

void uart_log_init(uint32_t baudrate)
{
	lpuart_init_t lpuart_init_cofig;
	uart_config_t uart_config;

	// set iomux
	gpio_set_iomux(GPIOD, GPIO_PIN_12, 2); // LPUART_RX:GP60
	gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);  // UART0_TX:GP17

	// lpuart init
	lpuart_deinit(LPUART);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_LPUART, true);
	lpuart_init_cofig.baudrate = baudrate;
	lpuart_init_cofig.data_width = LPUART_DATA_8BIT;
	lpuart_init_cofig.parity = LPUART_PARITY_NONE;
	lpuart_init_cofig.stop_bits = LPUART_STOP_1BIT;
	lpuart_init_cofig.low_level_wakeup = false;
	lpuart_init_cofig.start_wakeup = false;
	lpuart_init_cofig.rx_done_wakeup = true;
	lpuart_init(LPUART, &lpuart_init_cofig);

	lpuart_config_interrupt(LPUART, LPUART_CR1_RX_DONE_INT, ENABLE);
	lpuart_config_tx(LPUART, false);
	lpuart_config_rx(LPUART, true);

	NVIC_SetPriority(LPUART_IRQn, 2);
	NVIC_EnableIRQ(LPUART_IRQn);

	// uart init
	uart_config_init(&uart_config);
	uart_config.fifo_mode = ENABLE;
	uart_config.mode = UART_MODE_TX;
	uart_config.baudrate = baudrate;
	uart_init(CONFIG_DEBUG_UART, &uart_config);

	uart_cmd(CONFIG_DEBUG_UART, ENABLE);
}

void uart_to_esp32_init(uint32_t baudrate) {
	uart_config_t uart_config;

	// set iomux
	gpio_set_iomux(GPIOB, GPIO_PIN_8, 1);  // UART2_RX:GP24
	gpio_set_iomux(GPIOB, GPIO_PIN_9, 1);  // UART2_TX:GP25
		
	// uart init
	uart_config_init(&uart_config);
	uart_config.baudrate = baudrate;
	uart_config.data_width = UART_DATA_WIDTH_8;		
	uart_config.parity = UART_PARITY_NO;
	uart_config.stop_bits = UART_STOP_BITS_1;	
	uart_config.mode = UART_MODE_TXRX;
	uart_config.flow_control = UART_FLOW_CONTROL_DISABLED;
	uart_config.fifo_mode = DISABLE;

	uart_init(UART2, &uart_config);

	// uart_config_interrupt(UART2, UART_INTERRUPT_RX_DONE, true); 
	// NVIC_SetPriority(UART2_IRQn, 2);
	// NVIC_EnableIRQ(UART2_IRQn);

	uart_cmd(UART2, ENABLE);
}

// uint8_t esp_pc5_flag = 0;

void board_init()
{
	rcc_enable_oscillator(RCC_OSC_RCO32K, true);
	rcc_set_iwdg_clk_source(RCC_IWDG_CLK_SOURCE_RCO32K);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_IWDG, true);
	delay_ms(100);
	iwdg_init(true);

	iwdg_set_prescaler(IWDG_PRESCALER_256);
	iwdg_set_reload(0xFFF);
	iwdg_start();
	rcc_enable_oscillator(RCC_OSC_XO32K, true);

	rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);

	// gpio_init(GPIOC, GPIO_PIN_5, GPIO_MODE_INPUT_PULL_UP);
	// gpio_config_interrupt(GPIOC, GPIO_PIN_5, GPIO_INTR_FALLING_EDGE);
	// NVIC_SetPriority(GPIO_IRQn, 2);
	// NVIC_EnableIRQ(GPIO_IRQn);
	
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_LPUART, true);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART2, true);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_PWR, true);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_RTC, true);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_SAC, true);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_LORA, true);
#ifdef PRINT_BY_DMA
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_SYSCFG, true);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_DMA0, true);
#endif

	delay_ms(100);

	int rst_src = 0;

	rst_src = RCC->RST_SR;

	if (!(rst_src & 0x24))
	{
		RtcInit();
		*((unsigned int *)(0x2000F000)) = 0x00;
		*((unsigned int *)(0x2000F001)) = 0x00;
		*((unsigned int *)(0x2000F002)) = 0x00;
		*((unsigned int *)(0x2000F003)) = 0x00;

		*((unsigned int *)(0x2000F004)) = 0x00;
		*((unsigned int *)(0x2000F005)) = 0x00;

		for (int i = 0; i < 3328; i++)
		{
			*((unsigned int *)(SRAM_SENSOR_DATA_STORE_ACK_START_ADDR + i)) = 0xEE;
		}
	}
	else
	{
		rtc_calendar_cmd(ENABLE);
		NVIC_EnableIRQ(RTC_IRQn);

		RCC->RST_SR |= 0x24;
	}
}

void *_sbrk(int nbytes)
{
	// variables from linker script
	extern char _heap_bottom[];
	extern char _heap_top[];

	static char *heap_ptr = _heap_bottom;

	if ((unsigned int)(heap_ptr + nbytes) <= (unsigned int)_heap_top)
	{
		void *base = heap_ptr;
		heap_ptr += nbytes;
		return base;
	}
	else
	{
		return (void *)(-1);
	}
}

int main(void)
{
	// Target board initialization
	board_init();
	uart_log_init(9600);
	uart_to_esp32_init(9600);
	linkwan_at_init();
	Data_init();
	lora_test_init();
	StartIWDGRefresh();
	AckStartTX();
	LoraStartTx();

	if (debug_flags == 1)
	{
		LOG_PRINTF(LL_DEBUG, "dragino_6601_ota\r\n");
	}

	while (1)
	{
		/* Handle UART commands */
		linkwan_at_process();

		if ((uplink_data_status == 1) && (sending_flag == 0))
		{
			sending_flag = 1;
			if (ack_data_status == 1)
			{
				LOG_PRINTF(LL_DEBUG, "Send ACK\r\n");
			}
			Radio.SetChannel(tx_signal_freqence);
			Radio.SetTxConfig(MODEM_LORA, txp_value, 0, tx_bandwidth_value, tx_spreading_value, tx_codingrate_value, tx_preamble_value,
							  tx_header_value, tx_crc_value, 0, 0, tx_IQ_value, 3000);
			if (debug_flags == 0)
			{
				LOG_PRINTF(LL_DEBUG, "\r\n**************************\n\r");
			}
			else
			{
				LOG_PRINTF(LL_DEBUG, "\r\n***** UpLinkCounter= %lu *****\n\r", uplinkcount);
			}
			uplinkcount++;
			LOG_PRINTF(LL_DEBUG, "TX on freq %.3f Hz at SF %d\r\n", (float)(tx_signal_freqence / 1000000.0), tx_spreading_value);
			if (ack_data_status == 1)
			{
				Send_RX_message(rxmode_ack);
				ack_data_status = 0;
			}
			else
			{
				Send_TX();
			}
			uplink_data_status = 0;
		}

		if (rx_waiting_flag == 1)
		{
			Radio.SetChannel(rx_signal_freqence);
			Radio.SetRxConfig(MODEM_LORA, rx_bandwidth_value, rx_spreading_value, rx_codingrate_value, 0, rx_preamble_value,
							  0, rx_header_value, 0, rx_crc_value, 0, 0, rx_IQ_value, true);
			if (rxmode_timeout != 0)
			{
				LOG_PRINTF(LL_DEBUG, "RX on freq %.3f Hz at SF %d\r\n", (float)(rx_signal_freqence / 1000000.0), rx_spreading_value);
				if (rxmode_timeout == 65535)
				{
					if (debug_flags == 1)
					{
						LOG_PRINTF(LL_DEBUG, "Rx window is receiving\r\n");
					}
					Radio.Rx(0);
				}
				else
				{
					if (debug_flags == 1)
					{
						LOG_PRINTF(LL_DEBUG, "Rx window opens for %d seconds\r\n", rxmode_timeout);
					}
					Radio.Rx(rxmode_timeout * 1000);
				}
			}
			else
			{
				if (debug_flags == 1)
				{
					LOG_PRINTF(LL_DEBUG, "Rx window is close\r\n");
				}
				Radio.Sleep();
			}
			sending_flag = 0;
			rx_waiting_flag = 0;
		}

		if (is_time_to_IWDG_Refresh == 1)
		{
			is_time_to_IWDG_Refresh = 0;
			iwdg_reload();

			if (EnterLowPowerStopModeStatus == 0)
			{
				EnterLowPowerStopMode_error_times++;
				if (EnterLowPowerStopMode_error_times >= 2)
				{
					EnterLowPowerStopMode_error_times = 0;
					printf_dma_idx_w = printf_dma_idx_r;
					printf_dma_busy = 0;
				}
			}
		}

		if (print_isdone())
		{
			EnterLowPowerStopModeStatus = 1;
			TimerLowPowerHandler();
		}
		else
			EnterLowPowerStopModeStatus = 0;

		Radio.IrqProcess();
	}
}

static void Send_TX(void)
{
	uint8_t i = 0;

	if (tx_groupmod_value != 0)
	{
		txDataBuff[i++] = tx_groupmod_value;
	}

	for (uint8_t k = 0; k < sendBufferSize; k++)
	{
		txDataBuff[i++] = sendDataBuff[k];
	}

	txBufferSize = i;

	if (ack_flags >= 1)
	{
		TimerSetValue(&AckTimer, 5000);
		TimerStart(&AckTimer);
	}

	Radio.Send(txDataBuff, txBufferSize);
}

static void Send_RX_message(uint8_t type)
{
	uint8_t i = 0;

	if (type == 1)
	{
		if (tx_groupmod_value != 0)
		{
			rxrelayBuff[i++] = tx_groupmod_value;
		}

		for (uint8_t k = 0; k < rxBufferSize; k++)
		{
			rxrelayBuff[i++] = rxDataBuff[k];
		}
	}
	else if (type == 2)
	{
		rxrelayBuff[i++] = 0x00;
		rxrelayBuff[i++] = 0xff;
	}

	rxrelayBufferSize = i;

	Radio.Send(rxrelayBuff, rxrelayBufferSize);
}

static void RxData(lora_AppData_t *AppData)
{
	if (ack_flags == 1)
	{
		uint8_t comp_len = 0;
		for (uint8_t j = 0; j < AppData->BuffSize; j++)
		{
			if (txDataBuff[j] == AppData->Buff[j])
			{
				comp_len++;
			}
		}

		if (comp_len == txBufferSize)
		{
			TimerStop(&AckTimer);
			retransmission_temp = 0;
			ack_flags = 0;
			LOG_PRINTF(LL_DEBUG, "Receive ACK\r\n");
		}
	}
	else if (ack_flags == 2)
	{
		if ((AppData->Buff[0] == 0x00) && (AppData->Buff[1] == 0xff) && (AppData->BuffSize = 2))
		{
			TimerStop(&AckTimer);
			retransmission_temp = 0;
			ack_flags = 0;
			LOG_PRINTF(LL_DEBUG, "Receive ACK\r\n");
		}
	}

	if (rxmode_ack >= 1)
	{
		if (retun_wait_time != 0)
		{
			TimerSetValue(&TxTimer, retun_wait_time);
			TimerStart(&TxTimer);
		}
		else
		{
			uplink_data_status = 1;
			ack_data_status = 1;
		}
	}
}

static void OnTxTimerEvent(void)
{
	TimerStop(&TxTimer);
	uplink_data_status = 1;
	ack_data_status = 1;
}

static void LoraStartTx(void)
{
	TimerInit(&TxTimer, OnTxTimerEvent);
}

static void OnAckEvent(void)
{
	retransmission_temp++;
	if (retransmission_temp > retransmission_flags)
	{
		TimerStop(&AckTimer);
		retransmission_temp = 0;
		ack_flags = 0;
	}
	else
	{
		TimerSetValue(&AckTimer, 5000);
		TimerStart(&AckTimer);
		uplinkcount--;
		uplink_data_status = 1;
	}
}

static void AckStartTX(void)
{
	TimerInit(&AckTimer, OnAckEvent);
}

static void OnIWDGRefreshTimeoutEvent(void)
{
	TimerSetValue(&IWDGRefreshTimer, 18000);

	TimerStart(&IWDGRefreshTimer);

	is_time_to_IWDG_Refresh = 1;
}

static void StartIWDGRefresh(void)
{
	/* send everytime timer elapses */
	TimerInit(&IWDGRefreshTimer, OnIWDGRefreshTimeoutEvent);
	TimerSetValue(&IWDGRefreshTimer, 18000);
	TimerStart(&IWDGRefreshTimer);
}

void lora_test_init(void)
{
	// Radio initialization
	RadioEvents.TxDone = test_OnTxDone;
	RadioEvents.RxDone = test_OnRxDone;
	RadioEvents.TxTimeout = test_OnTxTimeout;
	RadioEvents.RxTimeout = test_OnRxTimeout;
	RadioEvents.RxError = test_OnRxError;

	Radio.Init(&RadioEvents);
	Radio.SetPublicNetwork(syncword_value);
	Radio.SetMaxPayloadLength(MODEM_LORA, 255);

	Radio.SetChannel(rx_signal_freqence);
	Radio.SetRxConfig(MODEM_LORA, rx_bandwidth_value, rx_spreading_value, rx_codingrate_value, 0, rx_preamble_value,
					  0, rx_header_value, 0, rx_crc_value, 0, 0, rx_IQ_value, true);
	if (rxmode_timeout == 65535)
	{
		if (debug_flags == 1)
		{
			LOG_PRINTF(LL_DEBUG, "\r\n***** RX window always open *****\n\r");
			LOG_PRINTF(LL_DEBUG, "RX on freq %.3f Hz at SF %d\r\n", (float)(rx_signal_freqence / 1000000.0), rx_spreading_value);
			LOG_PRINTF(LL_DEBUG, "Rx window is receiving\r\n");
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "\r\n**************************\n\r");
			LOG_PRINTF(LL_DEBUG, "RX on freq %.3f Hz at SF %d\r\n", (float)(rx_signal_freqence / 1000000.0), rx_spreading_value);
		}
		Radio.Rx(0);
		printf_dma_busy = 0;
	}
	else if (rxmode_timeout == 0)
	{
		if (debug_flags == 1)
		{
			LOG_PRINTF(LL_DEBUG, "\r\n***** RX window always close *****\n\r");
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "\r\n**************************\n\r");
		}
		Radio.Sleep();
		printf_dma_busy = 0;
	}
	else
	{
		if (debug_flags == 1)
		{
			LOG_PRINTF(LL_DEBUG, "\r\n***** RX window is open *****\n\r");
			LOG_PRINTF(LL_DEBUG, "RX on freq %.3f Hz at SF %d\r\n", (float)(rx_signal_freqence / 1000000.0), rx_spreading_value);
			LOG_PRINTF(LL_DEBUG, "Rx window opens for %d seconds\r\n", rxmode_timeout);
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "\r\n**************************\n\r");
			LOG_PRINTF(LL_DEBUG, "RX on freq %.3f Hz at SF %d\r\n", (float)(rx_signal_freqence / 1000000.0), rx_spreading_value);
		}
		Radio.Rx(rxmode_timeout * 1000);
		printf_dma_busy = 0;
	}
}

static void test_OnTxDone(void)
{
	Radio.Sleep();
	rx_waiting_flag = 1;
}

static void test_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	if ((rx_groupmod_value == 0) || (rx_groupmod_value == payload[0]))
	{
		rxBufferSize = 0;
		LOG_PRINTF(LL_DEBUG, "\r\nrxDone\r\nData: ");
		char str_temp1[] = "\r\nrxDone\r\nData: ";
		for(uint8_t i = 0;i < strlen(str_temp1);i++) {
			uart_send_data(UART2, *(str_temp1 + i));
		}
		
		if (rx_form_value == 0)
		{
			LOG_PRINTF(LL_DEBUG, "(HEX:) ");
			char str_temp2[] = "(HEX:) ";
			for(uint8_t i = 0;i < strlen(str_temp2);i++) {
				uart_send_data(UART2, *(str_temp2 + i));
			}
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "(String: ) ");
			char str_temp3[] = "(String: ) ";
			for(uint8_t i = 0;i < strlen(str_temp3);i++) {
				uart_send_data(UART2, *(str_temp3 + i));
			}
		}

		for (uint16_t i = 0; i < size; i++)
		{
			if (rx_form_value == 0)
			{
				LOG_PRINTF(LL_DEBUG, "%02x ", payload[i]);
				char str_temp6[10] = {'\0'};
				snprintf(str_temp6, 10, "%02x ", payload[i]);
				for(uint8_t i = 0;i < strlen(str_temp6);i++) {
					uart_send_data(UART2, *(str_temp6 + i));
				}
				delay_ms(6);
			}
			else
			{
				printf("%c", payload[i]);
				uart_send_data(UART2, payload[i]);
				delay_ms(6);
			}
			rxDataBuff[rxBufferSize++] = payload[i];
		}

		LOG_PRINTF(LL_DEBUG, "\r\n\r\n");
		char str_temp5[] = "\r\n\r\n";
		for(uint8_t i = 0;i < strlen(str_temp5);i++) {
			uart_send_data(UART2, *(str_temp5 + i));
		}

		receiveDataBuff = rxDataBuff;
		AppData.Buff = rxDataBuff;
		receiveBufferSize = rxBufferSize;
		AppData.BuffSize = rxBufferSize;

		RxData(&AppData);
		downlinkcount++;
		rssi_value = rssi;
		snr_value = snr;
		printf_dma_busy = 0;
	}
	else
	{
		data_check_flag = 1;
	}
}

static void test_OnTxTimeout(void)
{
	LOG_PRINTF(LL_DEBUG, "txtimeout\n\r");
	Radio.Sleep();
	rx_waiting_flag = 1;
}

static void test_OnRxTimeout(void)
{
	LOG_PRINTF(LL_DEBUG, "rxtimeout\n\r");
}

static void test_OnRxError(void)
{
	LOG_PRINTF(LL_DEBUG, "rxError\n\r");
}

#ifdef USE_FULL_ASSERT
void assert_failed(void *file, uint32_t line)
{
	(void)file;
	(void)line;

	while (1)
	{
	}
}
#endif
