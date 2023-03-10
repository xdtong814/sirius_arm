#ifndef SPI_TASK_H_
#define SPI_TASK_H_

#include "struct_typedef.h"
#include "bsp_spi.h"

// 8_bit_length
#define SPI_MESSAGE_LENGTH 132
#define SPI_MESSAGE_RX_LENGTH 132
#define SPI_MESSAGE_TX_LENGTH 60
#define SPI_MESSAGE_RX_CHECKLENGTH 32
#define SPI_MESSAGE_TX_CHECKLENGTH 14

// SPI_
#define SPI_TASK_INIT_TIME 500

// 3*4*6+4 = 76
typedef struct SPI_TX_COMMAND_T_
{
	float q_abad[2];
  float q_hip[2];
  float q_knee[2];
  float qd_abad[2];
  float qd_hip[2];
  float qd_knee[2];
  int32_t flags[2];
  int32_t checksum;
} SPI_TX_COMMAND_T;

//60
typedef struct SPI_TX_DATA_T_
{
	float q_abad[2];
  float q_hip[2];
  float q_knee[2];
  float qd_abad[2];
  float qd_hip[2];
  float qd_knee[2];
  int32_t flags[2];
	
} SPI_TX_DATA_T;

//132
typedef struct SPI_RX_COMMAND_T_
{
	fp32 q_des_abad[2];
  fp32 q_des_hip[2];
  fp32 q_des_knee[2];
  fp32 qd_des_abad[2];
  fp32 qd_des_hip[2];
  fp32 qd_des_knee[2];
  fp32 kp_abad[2];
  fp32 kp_hip[2];
  fp32 kp_knee[2];
  fp32 kd_abad[2];
  fp32 kd_hip[2];
  fp32 kd_knee[2];
  fp32 tau_abad_ff[2];
  fp32 tau_hip_ff[2];
  fp32 tau_knee_ff[2];
  int32_t flags[2];
  int32_t checksum;
} SPI_RX_COMMAND_T;

typedef struct SPI_RX_DATA_T_
{
  fp32 q_des_abad[2];
  fp32 q_des_hip[2];
  fp32 q_des_knee[2];
  fp32 qd_des_abad[2];
  fp32 qd_des_hip[2];
  fp32 qd_des_knee[2];
  fp32 kp_abad[2];
  fp32 kp_hip[2];
  fp32 kp_knee[2];
  fp32 kd_abad[2];
  fp32 kd_hip[2];
  fp32 kd_knee[2];
  fp32 tau_abad_ff[2];
  fp32 tau_hip_ff[2];
  fp32 tau_knee_ff[2];
  int32_t flags[2];
} SPI_RX_DATA_T;

uint32_t data_checksum(uint32_t* data_to_check, uint32_t check_length);

uint8_t SPI_COMMAND_2_DATA(SPI_RX_COMMAND_T* sPI_RX_COMMAND, SPI_RX_DATA_T* sPI_RX_DATA);
uint8_t SPI_DATA_2_COMMAND(SPI_TX_COMMAND_T* sPI_TX_COMMAND, SPI_TX_DATA_T* sPI_TX_DATA);

void uint32touint8(uint32_t* vector_32, uint8_t* vector_8, uint16_t lengthof32);
void uint8touint32(uint32_t* vector_32, uint8_t* vector_8, uint16_t lengthof8);

const SPI_RX_COMMAND_T* get_SPI_RX_COMMAND(void);
const SPI_TX_COMMAND_T* get_SPI_TX_COMMAND(void);
const SPI_RX_DATA_T* get_SPI_RX_DATA(void);
const SPI_TX_DATA_T* get_SPI_TX_DATA(void);

void SPI_DATA_INIT(void);
void SPI_REFRESH_DATA(void);
void SPI_RUN(void);

#endif
