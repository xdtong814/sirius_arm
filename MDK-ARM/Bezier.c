#include "Bezier.h"
#include "bsp_can.h"
#include "CAN_BLDC.h"

#define PI 3.1415926

int32_t Time_Count;
fp32 Time;

void Bezier_Position(CAN_HandleTypeDef *hcan, uint8_t* BLDC_tx_message_data, Bezier_Control_TypeDef* Bezier_Struct, void(*CAN_Mesg)(void))
{
	//Bezier_Struct->Position_des = local_rc_ctrl->rc.ch[3];
	Time_Count = 0;
	fp32 Bezier_Diff = Bezier_Struct->Position_des - Bezier_Struct->Position_start;
	Time = 0;
	
	fp32 Bezier_Output;
	fp32 Bezier_Temp;
	
	// 将目标位置与当前位置切片10次发送，每次间隔30ms
	while(Time != 1)
	{
		Time = Time_Count * DELTA_TIME;
	  Bezier_Temp = Time * Time * Time + 3.0f * (Time * Time * (1 - Time));
	  Bezier_Output = Bezier_Struct->Position_start + Bezier_Temp *  Bezier_Diff;
		Bezier_Struct->Position_output = Bezier_Output;
		CAN_BLDC_cmd(hcan, BLDC_tx_message_data, Bezier_Struct->Position_output , 0, 100, 1.5, 0, CAN_Mesg);
		Time_Count++;
		HAL_Delay(30);
	}

}

