#include "BLDC_Motor.h"
#include "main.h"
#include "bsp_can.h"
#include "string.h"
#include "bsp_spi.h"

CAN_TxHeaderTypeDef  BLDC_tx_message;
BLDC_Measure_TypeDef BLDC_Motor[6];
uint8_t BLDC_tx_message_data[8];
void (*CAN_SETMESSAGES[6])(void);
uint16_t LED_PINS[6] = {GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8};
uint8_t enable = 0;

#define USE_SPI 1

//设置位置

const SPI_RX_DATA_T* local_SPI_RX_DATA;
const SPI_TX_DATA_T* local_SPI_TX_DATA;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void BLDC_MOTORS_DATA_INIT(void)
{
	memset(&BLDC_tx_message, 0, sizeof(BLDC_tx_message));
	memset(BLDC_Motor, 0, sizeof(BLDC_Motor));
	memset(BLDC_tx_message_data, 0, sizeof(BLDC_tx_message_data));

	CAN_SETMESSAGES[0] = CAN_SetMsg_Leg1_ABAD;
	CAN_SETMESSAGES[1] = CAN_SetMsg_Leg1_HIP;
	CAN_SETMESSAGES[2] = CAN_SetMsg_Leg1_KNEE;
	CAN_SETMESSAGES[3] = CAN_SetMsg_Leg2_ABAD;
	CAN_SETMESSAGES[4] = CAN_SetMsg_Leg2_HIP;
	CAN_SETMESSAGES[5] = CAN_SetMsg_Leg2_KNEE;
	
	local_SPI_RX_DATA = get_SPI_RX_DATA();
	local_SPI_TX_DATA = get_SPI_TX_DATA();
}


void CAN_SetMsg_Leg1_ABAD(void)
{
	BLDC_tx_message.StdId = MOTOR_LEG1_ABAD;
  BLDC_tx_message.IDE = CAN_ID_STD;
  BLDC_tx_message.RTR = CAN_RTR_DATA;
  BLDC_tx_message.DLC = 0x08;
}

void CAN_SetMsg_Leg1_HIP(void)
{
	BLDC_tx_message.StdId = MOTOR_LEG1_HIP;
  BLDC_tx_message.IDE = CAN_ID_STD;
  BLDC_tx_message.RTR = CAN_RTR_DATA;
  BLDC_tx_message.DLC = 0x08;
	//BLDC_tx_message.TransmitGlobalTime = DISABLE;
}
 
void CAN_SetMsg_Leg1_KNEE(void)
{
	BLDC_tx_message.StdId = MOTOR_LEG1_KNEE;
  BLDC_tx_message.IDE = CAN_ID_STD;
  BLDC_tx_message.RTR = CAN_RTR_DATA;
  BLDC_tx_message.DLC = 0x08;
	//BLDC_tx_message.TransmitGlobalTime = DISABLE;
}

void CAN_SetMsg_Leg2_ABAD(void)
{
	BLDC_tx_message.StdId = MOTOR_LEG2_ABAD;
  BLDC_tx_message.IDE = CAN_ID_STD;
  BLDC_tx_message.RTR = CAN_RTR_DATA;
  BLDC_tx_message.DLC = 0x08;
	//BLDC_tx_message.TransmitGlobalTime = DISABLE;
}

void CAN_SetMsg_Leg2_HIP(void)
{
	BLDC_tx_message.StdId = MOTOR_LEG2_HIP;
  BLDC_tx_message.IDE = CAN_ID_STD;
  BLDC_tx_message.RTR = CAN_RTR_DATA;
  BLDC_tx_message.DLC = 0x08;
	//BLDC_tx_message.TransmitGlobalTime = DISABLE;
}
 
void CAN_SetMsg_Leg2_KNEE(void)
{
	BLDC_tx_message.StdId = MOTOR_LEG2_KNEE;
  BLDC_tx_message.IDE = CAN_ID_STD;
  BLDC_tx_message.RTR = CAN_RTR_DATA;
  BLDC_tx_message.DLC = 0x08;
	//BLDC_tx_message.TransmitGlobalTime = DISABLE;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data_1[6];
	static uint8_t id_1;
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data_1);
	id_1 = rx_data_1[0];
  Receive_BLDC_Data(&BLDC_Motor[id_1 - 1], rx_data_1);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[6];
	static uint8_t id_2;
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
	id_2 = rx_data[0];
  Receive_BLDC_Data(&BLDC_Motor[id_2 + 2], rx_data);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void Receive_BLDC_Data(BLDC_Measure_TypeDef* mot,uint8_t* data)
{
	
	int8_t id =  data[0];
	int32_t p_int = ( data[1]<<8)| data[2];
  int32_t v_int = ( data[3]<<4)|( data[4]>>4);
  int32_t i_int = (( data[4]&0xF)<<8)| data[5];
  /// convert ints to floats ///
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float i = uint_to_float(i_int, -KI_MAX, KI_MAX, 12);
	
	mot->ID = id;
	mot->Position = p;
	mot->Velocity = v;
	mot->Current = i;
}

void CAN_BLDC_cmd(CAN_HandleTypeDef *hcan ,uint8_t* BLDC_tx_message_data, float p_des, float v_des, float kp, float kd, float t_ff, void(*CAN_Mesg)(void))
{
	uint32_t send_mail_box;
  p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);                    
  v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
  kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
  kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
  t_ff = fminf(fmaxf(KI_MIN, t_ff), KI_MAX);
  /// convert floats to unsigned ints ///
  int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);            
  int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  int t_int = float_to_uint(t_ff, KI_MIN, KI_MAX, 12);
  /// pack ints into the can buffer ///
  BLDC_tx_message_data[0] = p_int>>8;                                       
  BLDC_tx_message_data[1] = p_int&0xFF;
  BLDC_tx_message_data[2] = v_int>>4;
  BLDC_tx_message_data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
  BLDC_tx_message_data[4] = kp_int&0xFF;
  BLDC_tx_message_data[5] = kd_int>>4;
  BLDC_tx_message_data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
  BLDC_tx_message_data[7] = t_int&0xff;
	CAN_Mesg();
	HAL_CAN_AddTxMessage(hcan, &BLDC_tx_message, BLDC_tx_message_data, &send_mail_box);
}

//TODO
void CAN_BLDC_SEND_COMMAND()
{

	SPI_RUN();
	//if(enable)
	//{
	CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, local_SPI_RX_DATA->q_des_abad[0], 
																							 local_SPI_RX_DATA->qd_des_abad[0], 
																							 local_SPI_RX_DATA->kp_abad[0],
																							 local_SPI_RX_DATA->kd_abad[0],
																							 local_SPI_RX_DATA->tau_abad_ff[0],
																							 CAN_SETMESSAGES[0]);									
	delay_us(20);
	CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, local_SPI_RX_DATA->q_des_hip[0],  
																							 local_SPI_RX_DATA->qd_des_hip[0],
																							 local_SPI_RX_DATA->kp_hip[0],
																							 local_SPI_RX_DATA->kd_hip[0], 
																							 local_SPI_RX_DATA->tau_hip_ff[0], 
																							 CAN_SETMESSAGES[1]);	
	delay_us(20);
	CAN_BLDC_cmd(&hcan1, BLDC_tx_message_data, local_SPI_RX_DATA->q_des_knee[0], 
																							 local_SPI_RX_DATA->qd_des_knee[0], 
																							 local_SPI_RX_DATA->kp_knee[0],
																							 local_SPI_RX_DATA->kd_knee[0],  
																							 local_SPI_RX_DATA->tau_knee_ff[0], 
																							 CAN_SETMESSAGES[2]);			
	delay_us(20);
	CAN_BLDC_cmd(&hcan2, BLDC_tx_message_data, local_SPI_RX_DATA->q_des_abad[1], 
																							 local_SPI_RX_DATA->qd_des_abad[1], 
																							 local_SPI_RX_DATA->kp_abad[1],
																							 local_SPI_RX_DATA->kd_abad[1], 
																							 local_SPI_RX_DATA->tau_abad_ff[1],
																							 CAN_SETMESSAGES[3]);			
	delay_us(20);
	
	CAN_BLDC_cmd(&hcan2, BLDC_tx_message_data, local_SPI_RX_DATA->q_des_hip[1],  
																							 local_SPI_RX_DATA->qd_des_hip[1], 
																							 local_SPI_RX_DATA->kp_hip[1],
																							 local_SPI_RX_DATA->kd_hip[1],  
																							 local_SPI_RX_DATA->tau_hip_ff[1],
																							 CAN_SETMESSAGES[4]);			
	delay_us(20);
	CAN_BLDC_cmd(&hcan2, BLDC_tx_message_data, local_SPI_RX_DATA->q_des_knee[1], 
																							 local_SPI_RX_DATA->qd_des_knee[1], 
																							 local_SPI_RX_DATA->kp_knee[1],
																							 local_SPI_RX_DATA->kd_knee[1], 
																							 local_SPI_RX_DATA->tau_knee_ff[1],
																							 CAN_SETMESSAGES[5]);																						 
		
}

void SET_MOTOR_MODE(CAN_HandleTypeDef *hcan, uint8_t M_MODE, void(*CAN_Mesg)(void))
{
	uint32_t send_mail_box;
	switch(M_MODE)
	{
		case REST_MODE:
		BLDC_tx_message_data[0]=0XFF; 
		BLDC_tx_message_data[1]=0XFF; 
		BLDC_tx_message_data[2]=0XFF;
		BLDC_tx_message_data[3]=0XFF;
		BLDC_tx_message_data[4]=0XFF;
		BLDC_tx_message_data[5]=0XFF;
		BLDC_tx_message_data[6]=0XFF;
		BLDC_tx_message_data[7]=0XFD;
		CAN_Mesg();
		HAL_CAN_AddTxMessage(hcan, &BLDC_tx_message, BLDC_tx_message_data, &send_mail_box);
		break;
		
		case MOTOR_MODE:
		BLDC_tx_message_data[0]=0XFF; 
		BLDC_tx_message_data[1]=0XFF; 
		BLDC_tx_message_data[2]=0XFF;
		BLDC_tx_message_data[3]=0XFF;
		BLDC_tx_message_data[4]=0XFF;
		BLDC_tx_message_data[5]=0XFF;
		BLDC_tx_message_data[6]=0XFF;
		BLDC_tx_message_data[7]=0XFC;
		CAN_Mesg();
		HAL_CAN_AddTxMessage(hcan, &BLDC_tx_message, BLDC_tx_message_data, &send_mail_box);
		break;
	
		case ZERO_POSITION:
		BLDC_tx_message_data[0] = 0xFF;
		BLDC_tx_message_data[1] = 0xFF;
		BLDC_tx_message_data[2] = 0xFF;
		BLDC_tx_message_data[3] = 0xFF;
		BLDC_tx_message_data[4] = 0xFF;
		BLDC_tx_message_data[5] = 0xFF;
		BLDC_tx_message_data[6] = 0xFF;
		BLDC_tx_message_data[7] = 0xFE;  
		CAN_Mesg();
		HAL_CAN_AddTxMessage(hcan, &BLDC_tx_message, BLDC_tx_message_data, &send_mail_box);
		break;
	};
}

void ENABLE_LEGS()
{
	//近端板子delay_ms(1)
	//远端
	delay_ms(1);
	SET_MOTOR_MODE(&hcan1, MOTOR_MODE, CAN_SETMESSAGES[0]);
	delay_ms(1);
	SET_MOTOR_MODE(&hcan1, MOTOR_MODE, CAN_SETMESSAGES[1]);
	delay_ms(1);
	SET_MOTOR_MODE(&hcan1, MOTOR_MODE, CAN_SETMESSAGES[2]);
	delay_ms(1);
	SET_MOTOR_MODE(&hcan2, MOTOR_MODE, CAN_SETMESSAGES[3]);
	delay_ms(1);
	SET_MOTOR_MODE(&hcan2, MOTOR_MODE, CAN_SETMESSAGES[4]);
	delay_ms(1);
	SET_MOTOR_MODE(&hcan2, MOTOR_MODE, CAN_SETMESSAGES[5]);
	delay_ms(1);
}

void DISABLE_LEGS()
{
	  delay_ms(1);
		SET_MOTOR_MODE(&hcan1, REST_MODE, CAN_SETMESSAGES[0]);
	  delay_ms(1);
		SET_MOTOR_MODE(&hcan1, REST_MODE, CAN_SETMESSAGES[1]);
	  delay_ms(1);
		SET_MOTOR_MODE(&hcan1, REST_MODE, CAN_SETMESSAGES[2]);
	  delay_ms(1);
		SET_MOTOR_MODE(&hcan2, REST_MODE, CAN_SETMESSAGES[3]);
	  delay_ms(1);	
	  SET_MOTOR_MODE(&hcan2, REST_MODE, CAN_SETMESSAGES[4]);
	  delay_ms(1);
		SET_MOTOR_MODE(&hcan2, REST_MODE, CAN_SETMESSAGES[5]);
	  delay_ms(1);
}

void OPEN_POWER()
{
	HAL_GPIO_WritePin(GPIOH,  GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);
}
void CLOSE_POWER()
{
	HAL_GPIO_WritePin(GPIOH,  GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
}

