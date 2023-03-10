#ifndef CUBICBEZIER_H_
#define CUBICBEZIER_H_

#include "struct_typedef.h"
#include "main.h"

#define DELTA_TIME 0.05

typedef struct _Bezier_TypeDef
{
	fp32 Position_start;
	fp32 Position_des;
	fp32 Position_output;

} Bezier_Control_TypeDef;

void Bezier_Position(CAN_HandleTypeDef *hcan, uint8_t* BLDC_tx_message_data, Bezier_Control_TypeDef* Bezier_Struct, void(*CAN_Mesg)(void));

#endif
