/*!
 * @file rt_spi.h
 * @brief SPI communication to spine board
 */

#ifndef _rt_spi
#define _rt_spi

#ifdef linux

#include <fcntl.h>      //Needed for SPI port
#include <sys/ioctl.h>  //Needed for SPI port

// incredibly obscure bug in SPI_IOC_MESSAGE macro is fixed by this
#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

#include <linux/spi/spidev.h>

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>  //Needed for SPI port

const int K_UINT32_TX_COMMAND = 33;
const int K_UINT32_RX_DATA = 15;
const int K_CHECKSIZE_SENT_PER_STM = 32;
const int K_CHECKSIZE_RECEIVED_FROM_STM = 14;
#define K_WORDS_PER_MESSAGE 132
#define K_KNEE_OFFSET_POS 2.5f //our dog is 325.6/2
typedef struct {
    float q_des_abad[6];
    float qd_des_abad[6];
    float kp_abad[6];
    float kd_abad[6];
    float tau_abad_ff[6];
    int32_t flags[6];
} spi_command_t;

typedef struct {
    float      q_abad[6];
    float      qd_abad[6];
    int32_t    flags[6];
    int32_t    spi_driver_status;
    float      tau_abad[6];
} spi_data_t;


void init_spi();
int spi_open();
void spi_close();

void spi_send_receive(spi_command_t* command, spi_data_t* data);
void spi_driver_run();
void uint8touint32(uint32_t* vector_32, uint8_t* vector_8, uint16_t lengthof32);
void uint32touint8(uint32_t* vector_32, uint8_t* vector_8, uint16_t lengthof32);
uint32_t xor_checksum(uint32_t *data, size_t len);
spi_data_t* get_spi_data();
spi_command_t* get_spi_command();

/*!
 * SPI command message
 * 32 + 1 = 33 32bitsword = 132bits
 */


typedef struct {
    float q_des_abad[6];
    float qd_des_abad[6];
    float kp_abad[6];
    float kd_abad[6];
    float tau_abad_ff[6];
    int32_t flags[6];
    int32_t checksum;
} spine_cmd_t;

/*!
 * SPI data message
 */
typedef struct {
    float q_abad[6];
    float qd_abad[6];
    int32_t flags[6];
    int32_t checksum;
} spine_data_t;

typedef struct {
    float tau_abad[6];
} spi_torque_t;

#endif // END of #ifdef linux

#endif

