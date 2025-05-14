/*
 * pwm.c
 *
 *  Created on: May 14, 2025
 *      Author: motii
 */

#include "pwm.h"

void motor1(int pwm)
{
	if(pwm > 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);
		HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
	}
	else if(pwm < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, abs(pwm));
		HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
	}
}
void motor2(int pwm)
{
	if(pwm > 0)
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
	}
	else if(pwm < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, abs(pwm));
		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
	}
}
void motor3(int pwm)
{
	if(pwm > 0)
	{
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, pwm);
		HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_RESET);
	}
	else if(pwm < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, abs(pwm));
		HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_SET);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
		HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_RESET);
	}
}
