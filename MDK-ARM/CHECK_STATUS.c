#include "CHECK_STATUS.h"
#include "CAN_BLDC.h"
#include "main.h"

extern BLDC_Measure_TypeDef BLDC_Motor[6];
extern TIM_HandleTypeDef htim12;
fp32 FLUCTUATION_BOUND =  0.1;

HAL_StatusTypeDef CHECK_MOTOR_STATUS(BLDC_Measure_TypeDef* MOTORS_TO_CHECK, fp32 DESIRED_POSITION)
{
	//蜂鸣器挂载在APB1 84MHz 蜂鸣器额定频率2700hz
	//84000000 / 42000 = 4000hz 
	//重载值与分频值决定pwm频率，比较值决定占空比
	//OPEN BUZZER
	fp32 LOWER_BOUND = (fp32)(DESIRED_POSITION - FLUCTUATION_BOUND);
	fp32 UPPER_BOUND = (fp32)(DESIRED_POSITION + FLUCTUATION_BOUND);
	if((LOWER_BOUND < MOTORS_TO_CHECK->Position) && (MOTORS_TO_CHECK->Position < UPPER_BOUND))
		return HAL_OK;
	else 
		return HAL_ERROR;
}

void CHECK_READY(void)
{
	__HAL_TIM_PRESCALER(&htim12, 3);
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 21000);
	HAL_Delay(250);
	__HAL_TIM_PRESCALER(&htim12, 2);
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 21000);
	HAL_Delay(250);
	__HAL_TIM_PRESCALER(&htim12, 1);
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 21000);
	HAL_Delay(250);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
}

void CHECK_FAILED(uint8_t motor_id)
{
	for(int i = 0; i < motor_id + 1; i++)
	{
		__HAL_TIM_PRESCALER(&htim12, 2);
	  __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 21000);
	  HAL_Delay(250);
	  __HAL_TIM_PRESCALER(&htim12, 2);
	  __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 0);
		HAL_Delay(250);
	}
	
	//用来隔开电机编号
	__HAL_TIM_PRESCALER(&htim12, 1);
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 21000);
	HAL_Delay(250);
	__HAL_TIM_PRESCALER(&htim12, 1);
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 0);
  HAL_Delay(250);
	
}

void init_buzzer(void)
{
	HAL_TIM_Base_Start(&htim12);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	__HAL_TIM_PRESCALER(&htim12, 1); // 2000Hz
}
