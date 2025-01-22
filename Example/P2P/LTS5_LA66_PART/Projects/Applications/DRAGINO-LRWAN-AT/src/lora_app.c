/******************************************************************************
 * @file    lora.c
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   lora API to drive the lora state Machine
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "timer.h"
#include "lora_app.h"
#include "flash_eraseprogram.h"
#include "version.h"
#include "log.h"
#include "tremo_system.h"
#include "tremo_flash.h"
#include "tremo_delay.h"

uint32_t retun_wait_time = 0;
uint8_t product_id_read_in_flash = 0;
uint8_t fre_band_read_in_flash = 0;
uint8_t firmware_ver_read_in_flash = 0;

uint8_t tx_groupmod_value;
uint8_t rx_groupmod_value;
uint32_t tx_signal_freqence;
uint32_t rx_signal_freqence;
uint8_t tx_bandwidth_value;
uint8_t rx_bandwidth_value;
uint8_t tx_spreading_value;
uint8_t rx_spreading_value;
uint8_t txp_value;
uint8_t tx_codingrate_value;
uint8_t rx_codingrate_value;
bool tx_header_value;
bool rx_header_value;
bool tx_crc_value;
bool rx_crc_value;
bool tx_IQ_value;
bool rx_IQ_value;
bool rx_form_value;
uint16_t tx_preamble_value;
uint16_t rx_preamble_value;
uint8_t syncword_value;
uint16_t rxmode_timeout;
uint8_t rxmode_ack;

extern bool print_isdone(void);

void Data_init(void)
{
	uint8_t status = (*((uint8_t *)FLASH_USER_START_ADDR_CONFIG));
	if ((status == 0x00) || (status == 0xff) || (status == 0x11))
	{
		retun_wait_time = 1000;
		tx_groupmod_value = 0;// 21 89
		rx_groupmod_value = 0;// 21 89
		tx_signal_freqence = 868700000;
		rx_signal_freqence = 868700000;
		tx_bandwidth_value = 0;
		rx_bandwidth_value = 0;
		tx_spreading_value = 12;
		rx_spreading_value = 12;
		txp_value = 14;
		tx_codingrate_value = 1;
		rx_codingrate_value = 1;
		tx_header_value = 0;
		rx_header_value = 0;
		tx_crc_value = 1;
		rx_crc_value = 1;
		tx_IQ_value = 0;
		rx_IQ_value = 0;
		tx_preamble_value = 8;
		rx_preamble_value = 8;
		syncword_value = 1;
		rxmode_timeout = 65535;
		rxmode_ack = 0;//  1 2
		Flash_Store_Config();
	}

	Flash_Read_Config();
	Printf_message();
}

void Printf_message(void)
{
	char *string1[2] = {"OFF", "ON"};
	char *string2[2] = {"explicit header", "implicit header"};
	char *string3[10] = {"125", "250", "500", "62.5", "41.67",
						 "31.25", "20.83", "15.63", "10.42", "7.81"};
	float txfreq, rxfreq;
	txfreq = tx_signal_freqence / 1000000.0;
	rxfreq = rx_signal_freqence / 1000000.0;

	LOG_PRINTF(LL_DEBUG, "\n\r");
	LOG_PRINTF(LL_DEBUG, "LA66 P2P Firmware " AT_VERSION_STRING "\n\r");
	delay_ms(20);
	LOG_PRINTF(LL_DEBUG, "Group: %d , %d\n\r", tx_groupmod_value, rx_groupmod_value);
	delay_ms(20);

	if (syncword_value == 1)
		LOG_PRINTF(LL_DEBUG, "Syncword: 0x3444 for Public Network\n\r");
	else
		LOG_PRINTF(LL_DEBUG, "Syncword: 0x1424 for Private Network\n\r");
	delay_ms(20);

	if ((tx_signal_freqence >= 450000000) && (tx_signal_freqence <= 999000000) && (rx_signal_freqence >= 450000000) && (rx_signal_freqence <= 999000000))
	{
		LOG_PRINTF(LL_DEBUG, "Frequency: %.3f MHZ , %.3f MHZ\n\r", txfreq, rxfreq);
		delay_ms(20);
	}

	if ((tx_bandwidth_value <= 9) && (rx_bandwidth_value <= 9))
	{
		LOG_PRINTF(LL_DEBUG, "BandWidth: %s kHZ , %s kHZ\n\r", string3[tx_bandwidth_value], string3[rx_bandwidth_value]);
		delay_ms(20);
	}
	LOG_PRINTF(LL_DEBUG, "Spreading Factor: %d , %d\n\r", tx_spreading_value, rx_spreading_value);
	delay_ms(20);
	LOG_PRINTF(LL_DEBUG, "Output Power: %d dBm\n\r", txp_value);
	delay_ms(20);
	LOG_PRINTF(LL_DEBUG, "Coding Rate: 4/%d , 4/%d\n\r", tx_codingrate_value + 4, rx_codingrate_value + 4);
	delay_ms(20);
	LOG_PRINTF(LL_DEBUG, "Header Type: %s , %s\n\r", string2[tx_header_value], string2[rx_header_value]);
	delay_ms(20);
	LOG_PRINTF(LL_DEBUG, "CRC: %s , %s\n\r", string1[tx_crc_value], string1[rx_crc_value]);
	delay_ms(20);
	LOG_PRINTF(LL_DEBUG, "Invert IQ: %d , %d\n\r", tx_IQ_value, rx_IQ_value);
	delay_ms(20);
	LOG_PRINTF(LL_DEBUG, "Preamble: %d , %d\n\r", tx_preamble_value, rx_preamble_value);
	delay_ms(20);
	LOG_PRINTF(LL_DEBUG, "RX Mode: %d , %d\n\r", rxmode_timeout, rxmode_ack);
	delay_ms(20);
}

void Flash_Store_Config(void)
{
	uint8_t store_config_in_flash[128];

	while (print_isdone() == 0)
		;

	store_config_in_flash[0] = 0x22; // FDR_status

	store_config_in_flash[1] = tx_groupmod_value & 0xFF;
	store_config_in_flash[2] = rx_groupmod_value & 0xFF;

	store_config_in_flash[3] = tx_signal_freqence >> 24 & 0xFF;
	store_config_in_flash[4] = tx_signal_freqence >> 16 & 0xFF;
	store_config_in_flash[5] = tx_signal_freqence >> 8 & 0xFF;
	store_config_in_flash[6] = tx_signal_freqence & 0xFF;
	store_config_in_flash[7] = rx_signal_freqence >> 24 & 0xFF;
	store_config_in_flash[8] = rx_signal_freqence >> 16 & 0xFF;
	store_config_in_flash[9] = rx_signal_freqence >> 8 & 0xFF;
	store_config_in_flash[10] = rx_signal_freqence & 0xFF;

	store_config_in_flash[11] = tx_bandwidth_value & 0xFF;
	store_config_in_flash[12] = rx_bandwidth_value & 0xFF;

	store_config_in_flash[13] = tx_spreading_value & 0xFF;
	store_config_in_flash[14] = rx_spreading_value & 0xFF;

	store_config_in_flash[15] = txp_value & 0xFF;

	store_config_in_flash[16] = tx_codingrate_value & 0xFF;
	store_config_in_flash[17] = rx_codingrate_value & 0xFF;

	store_config_in_flash[18] = tx_header_value & 0xFF;
	store_config_in_flash[19] = rx_header_value & 0xFF;

	store_config_in_flash[20] = tx_crc_value & 0xFF;
	store_config_in_flash[21] = rx_crc_value & 0xFF;

	store_config_in_flash[22] = tx_IQ_value & 0xFF;
	store_config_in_flash[23] = rx_IQ_value & 0xFF;

	store_config_in_flash[24] = tx_preamble_value >> 8 & 0xFF;
	store_config_in_flash[25] = tx_preamble_value & 0xFF;
	store_config_in_flash[26] = rx_preamble_value >> 8 & 0xFF;
	store_config_in_flash[27] = rx_preamble_value & 0xFF;

	store_config_in_flash[28] = syncword_value & 0xFF;

	store_config_in_flash[29] = rxmode_timeout >> 8 & 0xFF;
	store_config_in_flash[30] = rxmode_timeout & 0xFF;
	store_config_in_flash[31] = rxmode_ack & 0xFF;

	store_config_in_flash[32] = rx_form_value;

	store_config_in_flash[33] = retun_wait_time >> 24 & 0xFF;
	store_config_in_flash[34] = retun_wait_time >> 16 & 0xFF;
	store_config_in_flash[35] = retun_wait_time >> 8 & 0xFF;
	store_config_in_flash[36] = retun_wait_time & 0xFF;

	flash_erase_page(FLASH_USER_START_ADDR_CONFIG);
	if (flash_program_bytes(FLASH_USER_START_ADDR_CONFIG, store_config_in_flash, 128) == ERRNO_FLASH_SEC_ERROR)
	{
		LOG_PRINTF(LL_DEBUG, "write config error\r\n");
	}
}

void Flash_Read_Config(void)
{
	uint8_t read_config_in_flash[128];

	while (print_isdone() == 0)
		;

	for (int i = 0; i < 128; i++)
	{
		read_config_in_flash[i] = (*((uint8_t *)(FLASH_USER_START_ADDR_CONFIG + i)));
	}

	tx_groupmod_value = read_config_in_flash[1];
	rx_groupmod_value = read_config_in_flash[2];

	tx_signal_freqence = read_config_in_flash[3] << 24 | read_config_in_flash[4] << 16 | read_config_in_flash[5] << 8 | read_config_in_flash[6];
	rx_signal_freqence = read_config_in_flash[7] << 24 | read_config_in_flash[8] << 16 | read_config_in_flash[9] << 8 | read_config_in_flash[10];

	tx_bandwidth_value = read_config_in_flash[11];
	rx_bandwidth_value = read_config_in_flash[12];

	tx_spreading_value = read_config_in_flash[13];
	rx_spreading_value = read_config_in_flash[14];

	txp_value = read_config_in_flash[15];

	tx_codingrate_value = read_config_in_flash[16];
	rx_codingrate_value = read_config_in_flash[17];

	tx_header_value = read_config_in_flash[18];
	rx_header_value = read_config_in_flash[19];

	tx_crc_value = read_config_in_flash[20];
	rx_crc_value = read_config_in_flash[21];

	tx_IQ_value = read_config_in_flash[22];
	rx_IQ_value = read_config_in_flash[23];

	tx_preamble_value = read_config_in_flash[24] << 8 | read_config_in_flash[25];
	rx_preamble_value = read_config_in_flash[26] << 8 | read_config_in_flash[27];

	syncword_value = read_config_in_flash[28];

	rxmode_timeout = read_config_in_flash[29] << 8 | read_config_in_flash[30];
	rxmode_ack = read_config_in_flash[31];

	rx_form_value = read_config_in_flash[32];

	retun_wait_time = read_config_in_flash[33] << 24 | read_config_in_flash[34] << 16 | read_config_in_flash[35] << 8 | read_config_in_flash[36];
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
