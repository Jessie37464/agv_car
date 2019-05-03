#include "stm32f1xx_hal.h"
#include "main.h"
#include "motor.h"

void PWM_LEFT(int input)
{
	int pwm1;
	pwm1=(148-input);
	TIM3->CCR1=pwm1;
}
void PWM_LEFT2(int input)
{
	int pwm2;
	pwm2=(148-input);
	TIM3->CCR4=pwm2;
}
void PWM_RIGHT(int input)
{
	int pwm3;
	pwm3=(148-input);
	TIM3->CCR2=pwm3;
}

void PWM_RIGHT2(int input)
{
	int pwm4;
	pwm4=(148-input);
	TIM3->CCR3=pwm4;
}

void go_ahead()
{
HAL_GPIO_WritePin(GPIOB,right1_Pin,GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB,right2_Pin,GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB,left1_Pin,GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB,left2_Pin,GPIO_PIN_RESET);
}

void go_back()
{
HAL_GPIO_WritePin(GPIOB,right1_Pin,GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB,right2_Pin,GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB,left1_Pin,GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB,left2_Pin,GPIO_PIN_SET);
}

