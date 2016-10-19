/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdlib.h"
/* USER CODE BEGIN Includes */
#define NUM 18
#define RAW 18
//#undef TEMP
#define TEMP
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t raw_data[RAW];
uint8_t info_buffer[NUM];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Analyze(void);
void GetVector(uint8_t*);
void CheckVector(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==Speed_EXTI4_Pin){
		//get_speed set_speed
		uint8_t buffer[4];
		HAL_SPI_Receive(&hspi1,buffer,4,10);
		if(buffer[0]==0xAB&&buffer[3]==0xCD){//check
			//set speed
			if(buffer[1]>128){
				buffer[1]-=128;
				HAL_GPIO_WritePin(GPIOC,L1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,L0_Pin,GPIO_PIN_RESET);
			}
			else if(buffer[1]<128){
				buffer[1]=127-buffer[1];
				HAL_GPIO_WritePin(GPIOC,L1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,L0_Pin,GPIO_PIN_SET);
			}
			else{
				buffer[1]=100;
				HAL_GPIO_WritePin(GPIOC,L1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,L0_Pin,GPIO_PIN_SET);
			}
			if(buffer[2]>128){
				buffer[2]-=128;
				HAL_GPIO_WritePin(GPIOC,R1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,R0_Pin,GPIO_PIN_RESET);
			}
			else if(buffer[2]<128){
				buffer[2]=127-buffer[2];
				HAL_GPIO_WritePin(GPIOC,R1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,R0_Pin,GPIO_PIN_SET);
			}
			else{
				buffer[2]=100;
				HAL_GPIO_WritePin(GPIOC,R1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,R0_Pin,GPIO_PIN_SET);
			}
			TIM_OC_InitTypeDef L_sConfigOC;
			L_sConfigOC.OCMode = TIM_OCMODE_PWM1;
			L_sConfigOC.Pulse = (buffer[1]*(htim2.Init.Period+1))>>7;
			L_sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			L_sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			HAL_TIM_PWM_ConfigChannel(&htim2, &L_sConfigOC, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			TIM_OC_InitTypeDef R_sConfigOC;
			R_sConfigOC.OCMode = TIM_OCMODE_PWM1;
			R_sConfigOC.Pulse = (buffer[2]*(htim2.Init.Period+1))>>7;
			R_sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			R_sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			HAL_TIM_PWM_ConfigChannel(&htim2, &R_sConfigOC, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		}
	}
	else if(GPIO_Pin==Info_EXTI5_Pin){
		HAL_Delay(1);
		HAL_SPI_Transmit(&hspi1,info_buffer,NUM,10);
	}
}
void Analyze(){
	if(raw_data[16]==0x0D&&raw_data[17]==0x0A){
			//Analyze the data
			HAL_GPIO_WritePin(GPIOB,AimColor_OUT_Pin,(GPIO_PinState)(raw_data[1]&1));
			raw_data[1]>>=1;
			HAL_GPIO_WritePin(GPIOD,AimExist_OUT_Pin,(GPIO_PinState)(raw_data[1]&1));
			raw_data[1]>>=1;
			HAL_GPIO_WritePin(GPIOA,Outside_OUT_Pin,(GPIO_PinState)(raw_data[1]&1));
			raw_data[1]>>=1;
			HAL_GPIO_WritePin(GPIOB,A_Hurt_OUT_Pin,(GPIO_PinState)(raw_data[1]&1));
			raw_data[1]>>=1;
			HAL_GPIO_WritePin(GPIOB,A_Cure_OUT_Pin,(GPIO_PinState)(raw_data[1]&1));
			raw_data[1]>>=1;
			HAL_GPIO_WritePin(GPIOB,H_Hurt_OUT_Pin,(GPIO_PinState)(raw_data[1]&1));
			raw_data[1]>>=1;
			HAL_GPIO_WritePin(GPIOB,L_Hurt_OUT_Pin,(GPIO_PinState)(raw_data[1]&1));
			raw_data[1]>>=1;
			switch(raw_data[15]>>6){
				case 0:
					HAL_GPIO_WritePin(GPIOB,BW_OUT_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,AIR_OUT_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,HP_OUT_Pin,GPIO_PIN_RESET);
					break;
				case 1:
					HAL_GPIO_WritePin(GPIOB,BW_OUT_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,AIR_OUT_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,HP_OUT_Pin,GPIO_PIN_RESET);
					break;
				case 2:
					HAL_GPIO_WritePin(GPIOB,BW_OUT_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,AIR_OUT_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,HP_OUT_Pin,GPIO_PIN_SET);
					break;
				case 3:
					HAL_GPIO_WritePin(GPIOB,BW_OUT_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,AIR_OUT_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,HP_OUT_Pin,GPIO_PIN_RESET);
					break;
			}
			info_buffer[0]=0xAB;
			for(int i=0;i<12;i++)
				info_buffer[1+i]=raw_data[2+i];
			info_buffer[14]=0xCD;
			GetVector(info_buffer+15);
			info_buffer[17]=0xEF;
			CheckVector();
	}
}
void GetVector(uint8_t* buffer){
	//get the vector
}
void CheckVector(){
	//check the right vector
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
