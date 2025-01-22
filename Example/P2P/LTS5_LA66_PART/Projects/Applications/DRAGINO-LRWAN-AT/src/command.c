/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */
#include <stdlib.h>
#include <string.h>
#include "log.h"
#include "tremo_flash.h"
#include "tremo_delay.h"
#include "tremo_uart.h"
#include "tremo_system.h"

#include "command.h"
#include "lora_app.h"
#include "timer.h"
#include "version.h"
#include "bsp.h"
#include "flash_eraseprogram.h"
#include "tremo_system.h"
#include "tremo_gpio.h"
#include "lora_config.h"
#include "radio.h"
#include "tremo_rcc.h"
#include "sx126x-board.h"

#define ARGC_LIMIT 16
#define ATCMD_SIZE (242 * 2 + 18)
#define PORT_LEN 4

#define QUERY_CMD 0x01
#define EXECUTE_CMD 0x02
#define DESC_CMD 0x03
#define SET_CMD 0x04

uint8_t write_config_in_flash_status = 0;
uint8_t parse_flag = 0;
bool atrecve_flag = 0;
bool debug_flags = 0;
uint8_t atcmd[ATCMD_SIZE];
uint16_t atcmd_index = 0;
volatile bool g_atcmd_processing = false;
extern uint32_t retun_wait_time;
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
extern __IO uint32_t write_address;
extern bool sending_flag;
extern bool uplink_data_status;
extern bool rx_form_value;
extern uint8_t sendDataBuff[255];
extern uint8_t sendBufferSize;
extern uint8_t ack_flags;
extern uint8_t retransmission_flags;
extern uint32_t uplinkcount;
extern uint32_t downlinkcount;
extern int16_t rssi_value;
extern int8_t snr_value;
extern uint8_t *receiveDataBuff;
extern uint8_t receiveBufferSize;
extern void lora_test_init(void);

typedef struct
{
	char *cmd;
	int (*fn)(int opt, int argc, char *argv[]);
} at_cmd_t;

// AT functions
static int at_reset_func(int opt, int argc, char *argv[]);
static int at_debug_func(int opt, int argc, char *argv[]);
static int at_fdr_func(int opt, int argc, char *argv[]);
static int at_cfg_func(int opt, int argc, char *argv[]);
static int at_fcu_func(int opt, int argc, char *argv[]);
static int at_fcd_func(int opt, int argc, char *argv[]);
static int at_frequency_func(int opt, int argc, char *argv[]);
static int at_group_func(int opt, int argc, char *argv[]);
static int at_BandWidth_func(int opt, int argc, char *argv[]);
static int at_Spreading_Factor_func(int opt, int argc, char *argv[]);
static int at_Power_Range_func(int opt, int argc, char *argv[]);
static int at_CRC_func(int opt, int argc, char *argv[]);
static int at_Header_func(int opt, int argc, char *argv[]);
static int at_Coding_Rate_func(int opt, int argc, char *argv[]);
static int at_InvertIQ_func(int opt, int argc, char *argv[]);
static int at_Preamble_Length_func(int opt, int argc, char *argv[]);
static int at_syncword_func(int opt, int argc, char *argv[]);
static int at_rxmod_func(int opt, int argc, char *argv[]);
static int at_send_func(int opt, int argc, char *argv[]);
static int at_recv_func(int opt, int argc, char *argv[]);
static int at_rxdaform_func(int opt, int argc, char *argv[]);
static int at_waittime_func(int opt, int argc, char *argv[]);

static at_cmd_t g_at_table[] = {
	{AT_RESET, at_reset_func},
	{AT_DEBUG, at_debug_func},
	{AT_FDR, at_fdr_func},
	{AT_CFG, at_cfg_func},
	{AT_FCU, at_fcu_func},
	{AT_FCD, at_fcd_func},
	{AT_FRE, at_frequency_func},
	{AT_GROUPMOD, at_group_func},
	{AT_BW, at_BandWidth_func},
	{AT_SF, at_Spreading_Factor_func},
	{AT_POWER, at_Power_Range_func},
	{AT_CRC, at_CRC_func},
	{AT_HEADER, at_Header_func},
	{AT_CR, at_Coding_Rate_func},
	{AT_IQ, at_InvertIQ_func},
	{AT_PREAMBLE, at_Preamble_Length_func},
	{AT_SYNCWORD, at_syncword_func},
	{AT_RXMOD, at_rxmod_func},
	{AT_SEND, at_send_func},
	{AT_RECV, at_recv_func},
	{AT_RXDAFORM, at_rxdaform_func},
	{AT_WAITTIME, at_waittime_func},
};

#define AT_TABLE_SIZE (sizeof(g_at_table) / sizeof(at_cmd_t))

static int stringbin_format(const char *hex, uint8_t *bin, uint16_t bin_length)
{
	uint16_t hex_length = strlen(hex);
	const char *hex_end = hex + hex_length;
	uint8_t *cur = bin;

	while (hex < hex_end)
	{

		*cur++ = *hex;

		hex++;
	}
	return hex_length;
}

static int hex2bin_format2(const char *hex, uint8_t *bin, uint16_t bin_length)
{
	uint16_t hex_length = strlen(hex);
	const char *hex_end = hex + hex_length;
	uint8_t *cur = bin;
	uint8_t num_chars = hex_length & 1;
	uint8_t byte = 0;

	if ((hex_length + 1) / 2 > bin_length)
	{
		return -1;
	}

	while (hex < hex_end)
	{
		if ('A' <= *hex && *hex <= 'F')
		{
			byte |= 10 + (*hex - 'A');
		}
		else if ('a' <= *hex && *hex <= 'f')
		{
			byte |= 10 + (*hex - 'a');
		}
		else if ('0' <= *hex && *hex <= '9')
		{
			byte |= *hex - '0';
		}
		else
		{
			return -1;
		}
		hex++;
		num_chars++;

		if (num_chars >= 2)
		{
			num_chars = 0;
			*cur++ = byte;
			byte = 0;
		}
		else
		{
			byte <<= 4;
		}
	}
	return cur - bin;
}

static int at_reset_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	switch (opt)
	{
	case EXECUTE_CMD:
	{
		ret = LWAN_SUCCESS;
		system_reset();
		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Trig a reset of the MCU\r\n");
		break;
	}

	default:
		break;
	}

	return ret;
}

static int at_debug_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	switch (opt)
	{
	case EXECUTE_CMD:
	{
		ret = LWAN_SUCCESS;
		if (debug_flags == 1)
		{
			debug_flags = 0;
			snprintf((char *)atcmd, ATCMD_SIZE, "Exit Debug mode\r\n");
		}
		else
		{
			debug_flags = 1;
			snprintf((char *)atcmd, ATCMD_SIZE, "Enter Debug mode\r\n");
		}

		break;
	}
	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Set more info output\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_fdr_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	switch (opt)
	{
	case EXECUTE_CMD:
	{
		ret = LWAN_SUCCESS;
		uint8_t status[128] = {0};
		memset(status, 0x00, 128);

		flash_erase_page(FLASH_USER_START_ADDR_CONFIG);
		if (flash_program_bytes(FLASH_USER_START_ADDR_CONFIG, status, 128) == ERRNO_FLASH_SEC_ERROR)
		{
			snprintf((char *)atcmd, ATCMD_SIZE, "FDR error\r\n");
		}

		delay_ms(100);
		system_reset();
		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Reset Parameters to Factory Default\r\n");
		break;
	}

	default:
		break;
	}

	return ret;
}

static int at_cfg_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	switch (opt)
	{
	case EXECUTE_CMD:
	{
		if (sending_flag == 1)
		{
			return LWAN_BUSY_ERROR;
		}

		ret = LWAN_SUCCESS;
		atcmd[0] = '\0';

		for (uint8_t num = 0; num < AT_TABLE_SIZE; num++)
		{
			if (g_at_table[num].fn(QUERY_CMD, 0, 0) == LWAN_SUCCESS)
			{
				LOG_PRINTF(LL_DEBUG, "AT%s=", g_at_table[num].cmd);
				linkwan_serial_output(atcmd, strlen((const char *)atcmd));
				delay_ms(20);
			}
			atcmd_index = 0;
			memset(atcmd, 0xff, ATCMD_SIZE);
		}
		snprintf((char *)atcmd, ATCMD_SIZE, "\r\n");

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Print all configurations\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_fcu_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint32_t value = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%lu\r\n", uplinkcount);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value = strtol((const char *)argv[0], NULL, 0);
		uplinkcount = value;
		ret = LWAN_SUCCESS;
		atcmd[0] = '\0';

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Frame Counter Uplink\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_fcd_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint32_t value = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%lu\r\n", downlinkcount);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value = strtol((const char *)argv[0], NULL, 0);
		downlinkcount = value;
		ret = LWAN_SUCCESS;
		atcmd[0] = '\0';

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Frame Counter Downlink\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_frequency_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint32_t value1 = 0, value2 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%.3f,%.3f\r\n", tx_signal_freqence / 1000000.0, rx_signal_freqence / 1000000.0);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);
		value2 = strtol((const char *)argv[1], NULL, 0);

		if ((value1 > 100000) && (value2 > 100000))
		{
			tx_signal_freqence = value1 * 1000;
			rx_signal_freqence = value2 * 1000;
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Example: AT+FRE=868.100,868.100\r\n");
			return LWAN_PARAM_ERROR;
		}
		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set TX and RX frequency\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_group_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint16_t value1 = 0, value2 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", tx_groupmod_value, rx_groupmod_value);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);
		value2 = strtol((const char *)argv[1], NULL, 0);

		if ((value1 <= 255) && (value2 <= 255))
		{
			tx_groupmod_value = value1;
			rx_groupmod_value = value2;
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Range( 0 ~ 255 ), Example: AT+GROUP=0,0\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set TX and RX group\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_BandWidth_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint8_t value1 = 0, value2 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", tx_bandwidth_value, rx_bandwidth_value);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);
		value2 = strtol((const char *)argv[1], NULL, 0);

		if ((value1 <= 9) && (value2 <= 9))
		{
			tx_bandwidth_value = value1;
			rx_bandwidth_value = value2;
			LOG_PRINTF(LL_DEBUG, "Attention:Take effect after ATZ\r\n");
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Range( 0 ~ 9 ), Example: AT+BW=0,0\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set TX and RX BandWidth\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_Spreading_Factor_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint8_t value1 = 0, value2 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", tx_spreading_value, rx_spreading_value);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);
		value2 = strtol((const char *)argv[1], NULL, 0);

		if (((value1 >= 5) && (value1 <= 12)) && ((value2 >= 5) && (value2 <= 12)))
		{
			tx_spreading_value = value1;
			rx_spreading_value = value2;
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Range ( 5 ~ 12), Example: AT+SF=12,12\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set TX and RX Spreading Factor\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_Power_Range_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint8_t value1 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", txp_value);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);

		if (value1 <= 22)
		{
			txp_value = value1;
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Range ( 0 ~ 22), Example: AT+POWER=14\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set Tx Power Range\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_CRC_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint8_t value1 = 0, value2 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", tx_crc_value, rx_crc_value);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);
		value2 = strtol((const char *)argv[1], NULL, 0);

		if ((value1 <= 1) && (value2 <= 1))
		{
			tx_crc_value = value1;
			rx_crc_value = value2;
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Range ( 0 , 1), Example: AT+CRC=1,1\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set TX and RX CRC Type\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_Header_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint8_t value1 = 0, value2 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", tx_header_value, rx_header_value);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);
		value2 = strtol((const char *)argv[1], NULL, 0);

		if ((value1 <= 1) && (value2 <= 1))
		{
			tx_header_value = value1;
			rx_header_value = value2;
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Range ( 0 , 1), Example: AT+HEADER=0,0\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set TX and RX Header Type\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_Coding_Rate_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint8_t value1 = 0, value2 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", tx_codingrate_value, rx_codingrate_value);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);
		value2 = strtol((const char *)argv[1], NULL, 0);

		if (((value1 >= 1) && (value1 <= 4)) && ((value2 >= 1) && (value2 <= 4)))
		{
			tx_codingrate_value = value1;
			rx_codingrate_value = value2;
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Range ( 1 ~ 4), Example: AT+CR=1,1\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set TX and RX Header Type\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_InvertIQ_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint8_t value1 = 0, value2 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", tx_IQ_value, rx_IQ_value);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);
		value2 = strtol((const char *)argv[1], NULL, 0);

		if ((value1 <= 1) && (value2 <= 1))
		{
			tx_IQ_value = value1;
			rx_IQ_value = value2;
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Range ( 0 , 1), Example: AT+IQ=0,0\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set TX and RX InvertIQ\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_Preamble_Length_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint32_t value1 = 0, value2 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", tx_preamble_value, rx_preamble_value);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);
		value2 = strtol((const char *)argv[1], NULL, 0);

		if ((value1 <= 65535) && (value2 <= 65535))
		{
			tx_preamble_value = value1;
			rx_preamble_value = value2;
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Range ( 0 ~ 65535 ), Example: AT+PREAMBLE=8,8\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set TX and RX Preamble Length\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_syncword_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint8_t value1 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", syncword_value);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);

		if (value1 <= 1)
		{
			syncword_value = value1;
			LOG_PRINTF(LL_DEBUG, "Attention:Take effect after ATZ\r\n");
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Range ( 0 ,1 ), Example: AT+SYNCWORD=1\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set sync word\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_rxmod_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint32_t value1 = 0;
	uint8_t value2 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", rxmode_timeout, rxmode_ack);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);
		value2 = strtol((const char *)argv[1], NULL, 0);

		if (((value1 <= 65535)) && ((value2 <= 2)))
		{
			rxmode_timeout = value1;
			rxmode_ack = value2;
			LOG_PRINTF(LL_DEBUG, "Attention:Take effect after ATZ\r\n");
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Example: AT+RXMOD=65535,0\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set Rx Timeout and Reply mode\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_send_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;

	switch (opt)
	{

	case SET_CMD:
	{
		if (argc < 1)
			break;

		uint16_t length = 0, len = 0;
		uint8_t payload[255];
		uint8_t data_type = 0, ack_type = 0, retran_type = 0;

		data_type = strtol((const char *)argv[0], NULL, 0);
		ack_type = strtol((const char *)argv[2], NULL, 0);
		retran_type = strtol((const char *)argv[3], NULL, 0);
		length = strlen((const char *)argv[1]) / 2;

		if (data_type == 1)
		{
			len = stringbin_format((const char *)argv[1], payload, length);
		}
		else
		{
			len = hex2bin_format2((const char *)argv[1], payload, length);
		}

		if ((len > 0 && len < 255) && (ack_type <= 2) && (retran_type <= 8))
		{
			for (uint8_t i = 0; i < len; i++)
			{
				sendDataBuff[i] = payload[i];
			}
			sendBufferSize = len;
			ack_flags = ack_type;
			retransmission_flags = retran_type;
			uplink_data_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Example: AT+SEND=1,hello world,0,3\r\n");
			ret = LWAN_PARAM_ERROR;
		}
		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Send text data or hex along with the application port and confirm status\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_recv_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint8_t value = 0;

	switch (opt)
	{
	case SET_CMD:
	{
		if (argc < 1)
			break;

		value = strtol((const char *)argv[0], NULL, 0);

		if (value <= 1)
		{
			for (uint8_t i = 0; i < receiveBufferSize; i++)
			{
				if (value == 0)
				{
					LOG_PRINTF(LL_DEBUG, "%02x ", receiveDataBuff[i]);
					delay_ms(6);
				}
				else
				{
					LOG_PRINTF(LL_DEBUG, "%c", receiveDataBuff[i]);
					delay_ms(6);
				}
			}
			if (receiveBufferSize != 0)
			{
				LOG_PRINTF(LL_DEBUG, ",%d,%d\r\n", rssi_value, snr_value);
			}
			else
			{
				LOG_PRINTF(LL_DEBUG, "NULL\r\n");
			}
			receiveBufferSize = 0;
			rssi_value = 0;
			snr_value = 0;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Example: AT+RECV=0\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Print last receive message, RSSI and SNR\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_rxdaform_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint8_t value1 = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", rx_form_value);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		value1 = strtol((const char *)argv[0], NULL, 0);

		if (value1 <= 1)
		{
			rx_form_value = value1;
			write_config_in_flash_status = 1;
			ret = LWAN_SUCCESS;
			atcmd[0] = '\0';
		}
		else
		{
			LOG_PRINTF(LL_DEBUG, "Error Parameter, Range ( 0 , 1), Example: AT+RXDAFORM=0\r\n");
			return LWAN_PARAM_ERROR;
		}

		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set Format received by RX\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

static int at_waittime_func(int opt, int argc, char *argv[])
{
	int ret = LWAN_PARAM_ERROR;
	uint32_t time = 0;

	switch (opt)
	{
	case QUERY_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)retun_wait_time);

		break;
	}

	case SET_CMD:
	{
		if (argc < 1)
			break;

		time = strtol((const char *)argv[0], NULL, 0);

		retun_wait_time = time;
		ret = LWAN_SUCCESS;
		write_config_in_flash_status = 1;
		atcmd[0] = '\0';
		break;
	}

	case DESC_CMD:
	{
		ret = LWAN_SUCCESS;
		snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the takes time to return ACK in ms\r\n");
		break;
	}
	default:
		break;
	}

	return ret;
}

// this can be in intrpt context
void linkwan_serial_input(uint8_t cmd)
{
	if (g_atcmd_processing)
		return;

	if ((cmd >= '0' && cmd <= '9') || (cmd >= 'a' && cmd <= 'z') ||
		(cmd >= 'A' && cmd <= 'Z') || cmd == '?' || cmd == '+' ||
		cmd == ':' || cmd == '=' || cmd == ' ' || cmd == ',' || cmd == '-')
	{
		if (atcmd_index >= ATCMD_SIZE)
		{
			memset(atcmd, 0xff, ATCMD_SIZE);
			atcmd_index = 0;
			return;
		}
		atcmd[atcmd_index++] = cmd;
	}
	else if (cmd == '\r' || cmd == '\n')
	{
		if (atcmd_index >= ATCMD_SIZE)
		{
			memset(atcmd, 0xff, ATCMD_SIZE);
			atcmd_index = 0;
			return;
		}
		atcmd[atcmd_index] = '\0';
	}
}

int linkwan_serial_output(uint8_t *buffer, int len)
{
	LOG_PRINTF(LL_DEBUG, "%s", buffer);

	return 0;
}

void linkwan_at_process(void)
{
	char *ptr = NULL;
	int argc = 0;
	int index = 0;
	char *argv[ARGC_LIMIT];
	int ret = LWAN_ERROR;
	uint8_t *rxcmd = atcmd + 2;
	int16_t rxcmd_index = atcmd_index - 2;

	if ((atcmd[0] == 'A') && (atcmd[1] == 'T') && (atcmd[2] == '\0'))
	{
		LOG_PRINTF(LL_DEBUG, "\r\nOK\r\n");
		atcmd_index = 0;
		memset(atcmd, 0xff, ATCMD_SIZE);
		return;
	}

	if (atcmd_index <= 2 && atcmd[atcmd_index] == '\0')
	{

		atcmd_index = 0;
		memset(atcmd, 0xff, ATCMD_SIZE);
		return;
	}

	if (rxcmd_index <= 0 || rxcmd[rxcmd_index] != '\0')
	{
		return;
	}

	g_atcmd_processing = true;

	if (atcmd[0] != 'A' || atcmd[1] != 'T')
		goto at_end;

	if ((atcmd[0] == 'A') && (atcmd[1] == 'T') && (atcmd[2] == '?') && (atcmd[3] == '\0')) // AT?
	{
		for (uint8_t num = 0; num < AT_TABLE_SIZE; num++)
		{
			if (g_at_table[num].fn(DESC_CMD, 0, 0) == LWAN_SUCCESS)
			{
				LOG_PRINTF(LL_DEBUG, "AT%s: ", g_at_table[num].cmd);
				linkwan_serial_output(atcmd, strlen((const char *)atcmd));
			}
			delay_ms(50);
			atcmd_index = 0;
			memset(atcmd, 0xff, ATCMD_SIZE);
		}
		snprintf((char *)atcmd, ATCMD_SIZE, "\r\n");
		ret = LWAN_SUCCESS;
	}
	else
	{
		for (index = 0; index < AT_TABLE_SIZE; index++)
		{
			int cmd_len = strlen(g_at_table[index].cmd);
			if (!strncmp((const char *)rxcmd, g_at_table[index].cmd, cmd_len))
			{
				ptr = (char *)rxcmd + cmd_len;
				break;
			}
		}
		if (index >= AT_TABLE_SIZE || !g_at_table[index].fn)
			goto at_end;

		if (ptr[0] == '\0')
		{
			ret = g_at_table[index].fn(EXECUTE_CMD, argc, argv);
		}
		else if (ptr[0] == ' ')
		{
			argv[argc++] = ptr;
			ret = g_at_table[index].fn(EXECUTE_CMD, argc, argv);
		}
		else if ((ptr[0] == '?') && (ptr[1] == '\0'))
		{
			ret = g_at_table[index].fn(DESC_CMD, argc, argv);
		}
		else if ((ptr[0] == '=') && (ptr[1] == '?') && (ptr[2] == '\0'))
		{
			ret = g_at_table[index].fn(QUERY_CMD, argc, argv);
		}
		else if (ptr[0] == '=')
		{
			ptr += 1;

			char *str = strtok((char *)ptr, ",");
			while (str)
			{
				argv[argc++] = str;
				str = strtok((char *)NULL, ",");
			}
			ret = g_at_table[index].fn(SET_CMD, argc, argv);

			if ((ret == LWAN_SUCCESS) && (write_config_in_flash_status == 1))
			{
				Flash_Store_Config();
				write_config_in_flash_status = 0;
			}
		}
		else
		{
			ret = LWAN_ERROR;
		}
	}

at_end:

	if (LWAN_ERROR == ret)
		snprintf((char *)atcmd, ATCMD_SIZE, "\r\n%s\r\n", AT_ERROR);
	else if (LWAN_PARAM_ERROR == ret)
		snprintf((char *)atcmd, ATCMD_SIZE, "\r\n%s\r\n", AT_PARAM_ERROR);
	else if (LWAN_BUSY_ERROR == ret)
		snprintf((char *)atcmd, ATCMD_SIZE, "\r\n%s\r\n", AT_BUSY_ERROR);
	else if (LWAN_NO_NET_JOINED == ret)
		snprintf((char *)atcmd, ATCMD_SIZE, "\r\n%s\r\n", AT_NO_NET_JOINED);

	linkwan_serial_output(atcmd, strlen((const char *)atcmd));

	if (ret == LWAN_SUCCESS)
	{
		LOG_PRINTF(LL_DEBUG, "\r\nOK\r\n");
	}

	atcmd_index = 0;
	memset(atcmd, 0xff, ATCMD_SIZE);
	g_atcmd_processing = false;
	return;
}

void linkwan_at_init(void)
{
	atcmd_index = 0;
	memset(atcmd, 0xff, ATCMD_SIZE);
}