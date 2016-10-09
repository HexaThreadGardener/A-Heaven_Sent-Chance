#include "user.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"
void set_speed(signed char L,signed char R){
	if(L>0){
		HAL_GPIO_WritePin(GPIOD,L1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,L0_Pin,GPIO_PIN_RESET);
	}
	else if(L<0){
		L=-1-L;
		HAL_GPIO_WritePin(GPIOD,L1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,L0_Pin,GPIO_PIN_SET);
	}
	else{
		L=100;
		HAL_GPIO_WritePin(GPIOD,L1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,L0_Pin,GPIO_PIN_SET);
	}
	if(R>0){
		HAL_GPIO_WritePin(GPIOB,R1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,R0_Pin,GPIO_PIN_RESET);
	}
	else if(R<0){
		R=-1-R;
		HAL_GPIO_WritePin(GPIOB,R1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,R0_Pin,GPIO_PIN_SET);
	}
	else{
		R=100;
		HAL_GPIO_WritePin(GPIOB,R1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,R0_Pin,GPIO_PIN_SET);
	}
	TIM_OC_InitTypeDef L_sConfigOC;
	L_sConfigOC.OCMode = TIM_OCMODE_PWM1;
	L_sConfigOC.Pulse = (L*(htim4.Init.Period+1))>>7;
	L_sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	L_sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim4, &L_sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	TIM_OC_InitTypeDef R_sConfigOC;
	R_sConfigOC.OCMode = TIM_OCMODE_PWM1;
	R_sConfigOC.Pulse = (R*(htim4.Init.Period+1))>>7;
	R_sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	R_sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim4, &R_sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}
