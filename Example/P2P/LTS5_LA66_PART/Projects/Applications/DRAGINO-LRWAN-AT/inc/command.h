/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef LINKWAN_AT_H
#define LINKWAN_AT_H

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#define AT_CMD "AT"

#define AT_ERROR         "AT_ERROR"
#define AT_PARAM_ERROR   "AT_PARAM_ERROR"
#define AT_BUSY_ERROR    "AT_BUSY_ERROR"
#define AT_NO_NET_JOINED "AT_NO_NET_JOINED"

#define LWAN_SUCCESS  0
#define LWAN_ERROR   -1    
#define LWAN_PARAM_ERROR   -2    
#define LWAN_BUSY_ERROR   -3
#define LWAN_NO_NET_JOINED   -4

//// mandatory
#define AT_RESET      "Z"
#define AT_DEBUG      "+DEBUG"
#define AT_FDR        "+FDR"
#define AT_CFG        "+CFG"
#define AT_FCU        "+FCU"
#define AT_FCD        "+FCD"
#define AT_FRE        "+FRE"
#define AT_GROUPMOD   "+GROUPMOD"
#define AT_BW         "+BW"
#define AT_SF         "+SF"
#define AT_POWER      "+POWER"
#define AT_CRC				"+CRC"
#define AT_HEADER			"+HEADER"
#define AT_CR				  "+CR"
#define AT_IQ					"+IQ"
#define AT_PREAMBLE   "+PREAMBLE"
#define AT_SYNCWORD		"+SYNCWORD"
#define AT_RXMOD			"+RXMOD"
#define AT_SEND				"+SEND"
#define AT_RECV				"+RECV"
#define AT_RXDAFORM	  "+RXDAFORM"
#define AT_WAITTIME	  "+WAITTIME"

void linkwan_at_init(void);
void linkwan_at_process(void);
void linkwan_serial_input(uint8_t cmd);
int linkwan_serial_output(uint8_t *buffer, int len);
void linkwan_at_prompt_print();
#endif
