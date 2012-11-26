/**
  ******************************************************************************
  * @file    audiocodec_cs43l22.h
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    29-June-2012
  * @brief   This file contains all the functions prototypes for the CS43L22 codec
  *          firmware driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIOCODEC_CS43L22_H
#define __AUDIOCODEC_CS43L22_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Codec output DEVICE */
#define OutputDevice_SPEAKER          1
#define OutputDevice_HEADPHONE        2
#define OutputDevice_BOTH             3
#define OutputDevice_AUTO             4

/* Volume Levels values */
#define DEFAULT_VOLMIN                0x00
#define DEFAULT_VOLMAX                0xFF
#define DEFAULT_VOLSTEP               0x04

/* Codec POWER DOWN modes */
#define CodecPowerDown_HW             1
#define CodecPowerDown_SW             2

/* MUTE commands */
#define MUTE_ON                       1
#define MUTE_OFF                      0

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint32_t CODEC_Config(uint16_t AudioOutput, uint16_t I2S_Standard, uint16_t I2S_MCLKOutput, uint8_t Volume);
uint32_t CODEC_ControlVolume(uint8_t Volume);
void CODEC_Mute(uint32_t Command);
void CODEC_Reset(void);
void CODEC_PowerDown(uint32_t CodecPowerDown_Mode);

/* Low Layer Codec Function --------------------------------------------------*/
uint32_t CODEC_WriteRegister(uint32_t RegisterAddr, uint32_t RegisterValue);
uint32_t CODEC_ReadRegister(uint32_t RegisterAddr);
void CODEC_Delay(__IO uint32_t nCount);

#endif /* __AUDIOCODEC_CS43L22_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
