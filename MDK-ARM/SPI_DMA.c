#include "SPI_DMA.h"
#include "stdio.h"
#include "stdlib.h"
#include "CAN_BLDC.h"
#include "string.h"

extern SPI_HandleTypeDef hspi4;
extern DMA_HandleTypeDef hdma_spi4_rx;
extern DMA_HandleTypeDef hdma_spi4_tx;
extern uint8_t enable;
extern BLDC_Measure_TypeDef BLDC_Motor[6];

uint32_t check_temp_test;
int spi_count = 0;

SPI_RX_COMMAND_T SPI_RX_COMMAND;
SPI_RX_DATA_T SPI_RX_DATA;
SPI_TX_COMMAND_T SPI_TX_COMMAND;
SPI_TX_DATA_T SPI_TX_DATA;

uint8_t TX_BUFFER[SPI_MESSAGE_TX_LENGTH];
uint8_t RX_BUFFER[SPI_MESSAGE_RX_LENGTH];

int32_t flag_leg[2];

void uint32touint8(uint32_t* vector_32, uint8_t* vector_8, uint16_t lengthof32)
{
	for(int i = 0; i < lengthof32; i++)
	{
		vector_8[4 * i] = vector_32[i];
		vector_8[4 * i + 1] = vector_32[i] >> 8;
		vector_8[4 * i + 2] = vector_32[i] >> 16;
		vector_8[4 * i + 3] = vector_32[i] >> 24;
	}
}
	
void uint8touint32(uint32_t* vector_32, uint8_t* vector_8, uint16_t lengthof32)
{
	for(int i = 0; i < lengthof32; i++)
	{
		vector_32[i] = (vector_8[4 * i + 3] << 24) + (vector_8[4 * i + 2] << 16) + (vector_8[4 * i + 1] << 8) + vector_8[4 * i];
	}
}

uint32_t data_checksum(uint32_t* data_to_check, uint32_t check_length)
{
	uint32_t t = 0;
	for(int i = 0; i < check_length; i++)
	{
		t = t ^ data_to_check[i];
	}
	
	return t;
}

uint8_t SPI_COMMAND_2_DATA(SPI_RX_COMMAND_T* sPI_RX_COMMAND, SPI_RX_DATA_T* sPI_RX_DATA)
{
	uint32_t check_temp = data_checksum((uint32_t*)sPI_RX_COMMAND, SPI_MESSAGE_RX_CHECKLENGTH);
	if(check_temp == sPI_RX_COMMAND->checksum)
	{	
		for(int i = 0; i < 2; i++)
		{
			sPI_RX_DATA->q_des_abad[i] = sPI_RX_COMMAND->q_des_abad[i];
			sPI_RX_DATA->q_des_hip[i] = sPI_RX_COMMAND->q_des_hip[i];
			sPI_RX_DATA->q_des_knee[i] = sPI_RX_COMMAND->q_des_knee[i];
		
			sPI_RX_DATA->qd_des_abad[i] = sPI_RX_COMMAND->qd_des_abad[i];
			sPI_RX_DATA->qd_des_hip[i] = sPI_RX_COMMAND->qd_des_hip[i];
			sPI_RX_DATA->qd_des_knee[i] = sPI_RX_COMMAND->qd_des_knee[i];
		
			sPI_RX_DATA->kp_abad[i] = sPI_RX_COMMAND->kp_abad[i];
			sPI_RX_DATA->kp_hip[i] = sPI_RX_COMMAND->kp_hip[i];
			sPI_RX_DATA->kp_knee[i] = sPI_RX_COMMAND->kp_knee[i];
		
			sPI_RX_DATA->kd_abad[i] = sPI_RX_COMMAND->kd_abad[i];
			sPI_RX_DATA->kd_hip[i] = sPI_RX_COMMAND->kd_hip[i];
			sPI_RX_DATA->kd_knee[i] = sPI_RX_COMMAND->kd_knee[i];
		
			sPI_RX_DATA->tau_abad_ff[i] =  sPI_RX_COMMAND-> tau_abad_ff[i];
			sPI_RX_DATA->tau_hip_ff[i] =  sPI_RX_COMMAND-> tau_hip_ff[i];
			sPI_RX_DATA->tau_knee_ff[i] =  sPI_RX_COMMAND-> tau_knee_ff[i];
		
			sPI_RX_DATA->flags[i] = sPI_RX_COMMAND->flags[i];
		}
	}		
	
	
	return 0;
}

uint8_t SPI_DATA_2_COMMAND(SPI_TX_COMMAND_T* sPI_TX_COMMAND, SPI_TX_DATA_T* sPI_TX_DATA)
{
	for(int i = 0; i < 2; i++)
	{
	  sPI_TX_COMMAND->q_abad[i] = sPI_TX_DATA->q_abad[i];
		sPI_TX_COMMAND->q_hip[i] = sPI_TX_DATA->q_hip[i];
		sPI_TX_COMMAND->q_knee[i] = sPI_TX_DATA->q_knee[i];
	  
		sPI_TX_COMMAND->qd_abad[i] = sPI_TX_DATA->qd_abad[i];
		sPI_TX_COMMAND->qd_hip[i] = sPI_TX_DATA->qd_hip[i];
		sPI_TX_COMMAND->qd_knee[i] = sPI_TX_DATA->qd_knee[i];\
		
		sPI_TX_COMMAND->flags[i] = sPI_TX_DATA->flags[i];
	}
	
	sPI_TX_COMMAND->checksum = data_checksum((uint32_t*)sPI_TX_COMMAND,SPI_MESSAGE_TX_CHECKLENGTH);
	
	return 0;
	
}

void SPI_REFRESH_DATA()
{
	for(int i = 0; i< 2; i++)
	{
	  SPI_TX_DATA.q_abad[i] = BLDC_Motor[3*i].Position;
		SPI_TX_DATA.q_hip[i] = BLDC_Motor[3*i + 1].Position;
		SPI_TX_DATA.q_knee[i] = BLDC_Motor[3*i + 2].Position;
		
		SPI_TX_DATA.qd_abad[i] = BLDC_Motor[3*i].Velocity;
		SPI_TX_DATA.qd_hip[i] = BLDC_Motor[3*i + 1].Velocity;
		SPI_TX_DATA.qd_knee[i] = BLDC_Motor[3*i + 2].Velocity;
		
		SPI_TX_DATA.flags[i] = flag_leg[i];
	}
}

void SPI_DATA_INIT(void)
{
  memset(TX_BUFFER, 0, SPI_MESSAGE_TX_LENGTH * sizeof(uint8_t));
	memset(RX_BUFFER, 0, SPI_MESSAGE_RX_LENGTH * sizeof(uint8_t));
	memset(&SPI_RX_COMMAND, 0, sizeof(SPI_RX_COMMAND_T));
	memset(&SPI_TX_COMMAND, 0, sizeof(SPI_TX_COMMAND_T));
	memset(&SPI_RX_DATA, 0, sizeof(SPI_RX_DATA_T));
	memset(&SPI_TX_DATA, 0, sizeof(SPI_TX_DATA_T));
	memset(flag_leg, 0, 2 * sizeof(int32_t));
}

void SPI_RUN()
{
	SPI_REFRESH_DATA();
	SPI_DATA_2_COMMAND(&SPI_TX_COMMAND,&SPI_TX_DATA);
	uint32touint8((uint32_t*)&SPI_TX_COMMAND, TX_BUFFER, 15);
	
	HAL_SPI_TransmitReceive_DMA(&hspi4, (uint8_t*)TX_BUFFER, (uint8_t*)RX_BUFFER, SPI_MESSAGE_LENGTH);
	while(HAL_SPI_GetState(&hspi4) != HAL_SPI_STATE_READY)
	{
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	uint8touint32((uint32_t*)&SPI_RX_COMMAND, RX_BUFFER, 33);
	SPI_COMMAND_2_DATA(&SPI_RX_COMMAND,&SPI_RX_DATA);
	
	
	if((flag_leg[0] == 0) && (SPI_RX_DATA.flags[0] == 1))
	{
		INIT_MOTORS(0);
		flag_leg[0] = 1;
		enable++;
		delay_ms(500);
		return;
	}
	else if((flag_leg[0] == 1) && (SPI_RX_DATA.flags[0] == 0))
	{
		Disable_Leg(0);
		flag_leg[0] = 0;
		return;
	}
	else
	{}
	
}

const SPI_RX_COMMAND_T* get_SPI_RX_COMMAND()
{
	return &SPI_RX_COMMAND;
}

const SPI_TX_COMMAND_T* get_SPI_TX_COMMAND()
{
	return &SPI_TX_COMMAND;
}

const SPI_RX_DATA_T* get_SPI_RX_DATA()
{
	return &SPI_RX_DATA;
}

const SPI_TX_DATA_T* get_SPI_TX_DATA()
{
	return &SPI_TX_DATA;
}
