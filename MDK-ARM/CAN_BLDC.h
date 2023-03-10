#ifndef CAN_BLDC_H_
#define CAN_BLDC_H_

#include "struct_typedef.h"
#include "main.h"
#include "bsp_delay.h"

#define MOTOR_LEG1_ABAD 0x01
#define MOTOR_LEG1_HIP 0x02
#define MOTOR_LEG1_KNEE 0x03
#define MOTOR_LEG2_ABAD 0x04
#define MOTOR_LEG2_HIP 0x05
#define MOTOR_LEG2_KNEE 0x06

#define REST_MODE 0
#define MOTOR_MODE 1
#define ZERO_POSITION 2

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
 
#define KI_MIN -48.0f
#define KI_MAX 48.0f

typedef struct _BLDC_TypeDef
{
	int32_t ID;
	fp32 Position;
	fp32 Velocity;
	fp32 Current;
} BLDC_Measure_TypeDef;

void CAN_SetMsg_Leg1_ABAD(void);
void CAN_SetMsg_Leg1_HIP(void);
void CAN_SetMsg_Leg1_KNEE(void);
void CAN_SetMsg_Leg2_ABAD(void);
void CAN_SetMsg_Leg2_HIP(void);
void CAN_SetMsg_Leg2_KNEE(void);
void Receive_BLDC_Data(BLDC_Measure_TypeDef* mot,uint8_t* data);

void CAN_BLDC_cmd(CAN_HandleTypeDef *hcan ,uint8_t* BLDC_tx_message_data, float p_des, float v_des, float kp, float kd, float t_ff, void(*CAN_Mesg)(void));
void SET_MOTOR_MODE(CAN_HandleTypeDef *hcan, uint8_t M_MODE, void(*CAN_Mesg)(void));
void INIT_MOTORS(uint8_t leg);
void CAN_BLDC_SEND_COMMAND(void);
void BLDC_MOTORS_INIT(void);

void OPEN_POWER(void);
void CLOSE_POWER(void);

extern void Disable_Leg(uint8_t leg);

void ENABLE_LEGS(void);
#endif
