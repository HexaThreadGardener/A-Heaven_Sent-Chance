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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#undef TRUE
#undef FALSE
#define TRUE GPIO_PIN_SET
#define FALSE GPIO_PIN_RESET
#undef BOOL
#define BOOL GPIO_PinState

#define _READ_(X) HAL_GPIO_ReadPin(GPIOB,X)
#define A_CURE _READ_(A_Cure_IN_Pin)
#define A_HURT _READ_(A_Hunt_IN_Pin)
#define L_HURT _READ_(L_Hunt_IN_Pin)
#define H_HURT _READ_(H_Hunt_IN_Pin)
#define COLOR  _READ_(AimColor_IN_Pin)
#define EXIST  _READ_(AimExist_IN_Pin)
#define OUT    _READ_(Outside_IN_Pin)
#define HP     _READ_(HP_IN_Pin)
#define BW     _READ_(BW_IN_Pin)
#define AIR    _READ_(AR_IN_Pin)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t _speed[4]={0xAB,0,0,0xCD};
uint8_t _buffer[17];
typedef struct _Point{
	signed int x,y;
}Point;
typedef struct _Player{
	Point pos;
	uint8_t hp;
}Player;
typedef struct _Status{
	Player me;
	Player opponent;
	Point aim;
	Point plane;
	Point tools;
	Point vec;
	uint8_t valid;
}Status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SetSpeed(signed int,signed int);
Status GetStatus(void);
void HAL_SPI_RxCpltCallBack(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallBack(SPI_HandleTypeDef *hspi);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
A_CURE;
A_HURT;
L_HURT;
H_HURT;
COLOR;
EXIST;
OUT  ;
HP   ; 
BW   ;
AIR   ; 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
		Status now = GetStatus();
		
		// test
		uint8_t data[100];
		data[0] = now.me.pos.x;	// 11
		data[1] = now.me.pos.y;	// 22
		data[2] = now.me.hp;  // 99
		data[3] = now.opponent.pos.x;	// 33 
		data[4] = now.opponent.pos.y;	// 44
    data[5] = now.aim.x;	// 55
		data[6] = now.aim.y;	// 66
		data[7] = now.plane.x; // 77
		data[8] = now.plane.y; // 88
		data[9] = now.tools.x;	// BB
		data[10] = now.tools.y;	// CC
		data[11] = now.valid;//01
		data[12] = now.vec.x;//00
		data[13] = now.vec.y;//00
		data[14] = data[15] = 0;
		
		HAL_UART_Transmit(&huart3, data, 14, 10);
		
		HAL_Delay(500);

		
		// HAL_UART_Transmit(&huart3,....);
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
void SetSpeed(signed int l,signed int r){
	HAL_GPIO_WritePin(GPIOA,SPEED_Pin,GPIO_PIN_SET);
	_speed[1]=l+128;_speed[2]=r+128;
	HAL_SPI_Transmit_DMA(&hspi1,_speed,4);
	HAL_GPIO_WritePin(GPIOA,SPEED_Pin,GPIO_PIN_RESET);
}
Status GetStatus(void){
	HAL_GPIO_WritePin(GPIOA,INFO_Pin,GPIO_PIN_SET);
	HAL_SPI_Receive_DMA(&hspi1,_buffer,17);
	Status now;
	if(_buffer[0]==0xAB&&_buffer[13]==0xCD&&_buffer[16]==0xEF){
		HAL_GPIO_TogglePin(GPIOD,LED_Pin);
		now.me.pos.x=_buffer[1];
		now.me.pos.y=_buffer[2];
		now.opponent.pos.x=_buffer[3];
		now.opponent.pos.y=_buffer[4];
		now.aim.x=_buffer[5];
		now.aim.y=_buffer[6];
		now.plane.x=_buffer[7];
		now.plane.y=_buffer[8];
		now.me.hp=_buffer[9];
		now.opponent.hp=_buffer[10];
		now.tools.x=_buffer[11];
		now.tools.y=_buffer[12];
		now.vec.x=_buffer[14];
		now.vec.y=_buffer[15];
		now.valid=1;
	}
	else{
		now.valid=0;
	}
	HAL_GPIO_WritePin(GPIOA,INFO_Pin,GPIO_PIN_RESET);
	return now;
}
void HAL_SPI_RxCpltCallBack(SPI_HandleTypeDef *hspi){
	hspi->State = HAL_SPI_STATE_READY;
}
void HAL_SPI_TxCpltCallBack(SPI_HandleTypeDef *hspi){
	hspi->State = HAL_SPI_STATE_READY;
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
