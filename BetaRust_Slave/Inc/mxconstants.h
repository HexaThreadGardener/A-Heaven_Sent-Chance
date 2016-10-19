/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define L0_Pin GPIO_PIN_0
#define L0_GPIO_Port GPIOC
#define L1_Pin GPIO_PIN_1
#define L1_GPIO_Port GPIOC
#define R0_Pin GPIO_PIN_2
#define R0_GPIO_Port GPIOC
#define R1_Pin GPIO_PIN_3
#define R1_GPIO_Port GPIOC
#define PWM_L_Pin GPIO_PIN_0
#define PWM_L_GPIO_Port GPIOA
#define PWM_R_Pin GPIO_PIN_1
#define PWM_R_GPIO_Port GPIOA
#define Communcate_Tx_Pin GPIO_PIN_2
#define Communcate_Tx_GPIO_Port GPIOA
#define Communicate_Rx_Pin GPIO_PIN_3
#define Communicate_Rx_GPIO_Port GPIOA
#define Outside_OUT_Pin GPIO_PIN_4
#define Outside_OUT_GPIO_Port GPIOA
#define Speed_EXTI4_Pin GPIO_PIN_4
#define Speed_EXTI4_GPIO_Port GPIOC
#define Info_EXTI5_Pin GPIO_PIN_5
#define Info_EXTI5_GPIO_Port GPIOC
#define HP_OUT_Pin GPIO_PIN_0
#define HP_OUT_GPIO_Port GPIOB
#define AIR_OUT_Pin GPIO_PIN_1
#define AIR_OUT_GPIO_Port GPIOB
#define BW_OUT_Pin GPIO_PIN_2
#define BW_OUT_GPIO_Port GPIOB
#define Temp_Tx_Pin GPIO_PIN_9
#define Temp_Tx_GPIO_Port GPIOA
#define Temp_Rx_Pin GPIO_PIN_10
#define Temp_Rx_GPIO_Port GPIOA
#define Debug_Tx_Pin GPIO_PIN_10
#define Debug_Tx_GPIO_Port GPIOC
#define Debug_Rx_Pin GPIO_PIN_11
#define Debug_Rx_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOC
#define AimExist_OUT_Pin GPIO_PIN_2
#define AimExist_OUT_GPIO_Port GPIOD
#define AimColor_OUT_Pin GPIO_PIN_3
#define AimColor_OUT_GPIO_Port GPIOB
#define H_Hurt_OUT_Pin GPIO_PIN_4
#define H_Hurt_OUT_GPIO_Port GPIOB
#define L_Hurt_OUT_Pin GPIO_PIN_5
#define L_Hurt_OUT_GPIO_Port GPIOB
#define A_Hurt_OUT_Pin GPIO_PIN_6
#define A_Hurt_OUT_GPIO_Port GPIOB
#define A_Cure_OUT_Pin GPIO_PIN_7
#define A_Cure_OUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
