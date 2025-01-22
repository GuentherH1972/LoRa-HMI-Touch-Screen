/**
  ******************************************************************************
  * @file    lora_app.h
  * @author  MCD Application Team
  * @brief   Header of application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LORA_APP_H__
#define __LORA_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>	
#include "Commissioning.h"

void Data_init(void);
void Printf_message (void);
void Flash_Store_Config(void);
void Flash_Read_Config(void);

#ifdef __cplusplus
}
#endif

#endif /*__LORA_APP_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
