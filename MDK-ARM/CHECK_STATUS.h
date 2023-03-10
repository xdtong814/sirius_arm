#ifndef CHECK_STATUS_H_
#define CHECK_STATUS_H_

#include "main.h"
#include "CAN_BLDC.h"
#include "struct_typedef.h"

HAL_StatusTypeDef CHECK_MOTOR_STATUS(BLDC_Measure_TypeDef* MOTORS_TO_CHECK, fp32 DESIRED_POSITION);
void CHECK_READY(void);
void CHECK_FAILED(uint8_t motor_id);
void init_buzzer(void);

#endif

