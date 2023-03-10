#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <linux/spi/spidev.h>
#include "spi.h"
#include <iostream>

unsigned char spi_mode = SPI_MODE_0;
unsigned char spi_bits_per_word = 8;
// spi_speed relates the dead_lock of Upboard
unsigned int spi_speed = 6000000;

// Only use for simulation

int spi_1_fd = -1;
int spi_2_fd = -1;

int spi_open();

//data send to stm
static spine_cmd_t g_spine_cmd;
// data received from stm
static spine_data_t g_spine_data;

static spi_command_t spi_command_drv;
static spi_data_t spi_data_drv;

pthread_mutex_t spi_mutex;

const float max_torque[3]      =  {96.f, 96.f, 96.f};  //
const float wimp_torque[3]     =  {6.f, 6.f, 6.f};     //
const float disabled_torque[3] =  {0.f, 0.f, 0.f};


uint32_t xor_checksum(uint32_t *data, size_t len) {
    uint32_t t = 0;
    for (size_t i = 0; i < len; i++) t = t ^ data[i];
    return t;
}

/*!
 * Emulate the spi board to estimate the torque.
 */
void fake_spine_control(spi_command_t *cmd, spi_data_t *data,
                        spi_torque_t *torque_out, int board_num) {
    torque_out->tau_abad[board_num] =
            cmd->kp_abad[board_num] *
            (cmd->q_des_abad[board_num] - data->q_abad[board_num]) +
            cmd->kd_abad[board_num] *
            (cmd->qd_des_abad[board_num] - data->qd_abad[board_num]) +
            cmd->tau_abad_ff[board_num];

    const float *torque_limits = disabled_torque;

    if (cmd->flags[board_num] & 0b1) {
        if (cmd->flags[board_num] & 0b10)
            torque_limits = wimp_torque;
        else
            torque_limits = max_torque;
    }

    if (torque_out->tau_abad[board_num] > torque_limits[0])
        torque_out->tau_abad[board_num] = torque_limits[0];
    if (torque_out->tau_abad[board_num] < -torque_limits[0])
        torque_out->tau_abad[board_num] = -torque_limits[0];

}

/*!
 * Initialize SPI
 */
void init_spi() {
    // check sizes:
    memset(&g_spine_cmd, 0, sizeof(g_spine_cmd));
    memset(&g_spine_data,0,sizeof (g_spine_data));
    memset(&spi_command_drv, 0, sizeof(spi_command_drv));
    memset(&spi_data_drv, 0, sizeof(spi_data_drv));

    if (pthread_mutex_init(&spi_mutex, NULL) != 0)
        printf("[ERROR: RT SPI] Failed to create spi data mutex\n");
    spi_open();
}

/*!
 * Open SPI device
 */
int spi_open() {
    int rv = 0;
    (void) rv;
    spi_1_fd = open("/dev/spidev1.0", O_RDWR);
    if (spi_1_fd < 0) perror("[ERROR] Couldn't open spidev 1.0");
    spi_2_fd = open("/dev/spidev1.1", O_RDWR);
    if (spi_2_fd < 0) perror("[ERROR] Couldn't open spidev 1.1");

    rv = ioctl(spi_1_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (2)");
    return rv;
}

void spi_close()
{
    if((spi_1_fd < 0) && (spi_2_fd < 0))
        return;

    if((close(spi_1_fd) < 0) || (close(spi_2_fd) < 0))
    {
        std::cout << "[ERROR!] Close SPI falied!" << std::endl;
        return;
    }
    spi_1_fd = -1;
    spi_2_fd = -1;
}

/*!
 * convert spi command to spine_cmd_t
 */
void spi_to_spine(spi_command_t *cmd, spine_cmd_t *spine_cmd, int leg_0) {
    for (int i = 0; i < 6; i++) {

        spine_cmd->q_des_abad[i] = cmd->q_des_abad[i + leg_0] ;
        spine_cmd->kp_abad[i] = cmd->kp_abad[i + leg_0];
        spine_cmd->kd_abad[i] = cmd->kd_abad[i + leg_0];
        spine_cmd->tau_abad_ff[i] =
                cmd->tau_abad_ff[i + leg_0];
        spine_cmd->flags[i] = cmd->flags[i + leg_0];

    }
    spine_cmd->checksum = xor_checksum((uint32_t *)spine_cmd, K_CHECKSIZE_SENT_PER_STM);
}

/*!
 * convert spine_data_t to spi data
 */
void spine_to_spi(spi_data_t *data, spine_data_t *spine_data, int leg_0) {
    uint32_t calc_checksum = xor_checksum((uint32_t *)spine_data, K_CHECKSIZE_RECEIVED_FROM_STM);
    if (calc_checksum == (uint32_t)spine_data->checksum) {
        for (int i = 0; i < 6; i++) {
            data->q_abad[i + leg_0] = spine_data->q_abad[i];

            data->qd_abad[i + leg_0] = spine_data->qd_abad[i];

            data->flags[i + leg_0] = spine_data->flags[i + leg_0];
        }
    }
    else
    {
        std::cout << "[SPI Check Error] Supposed: " << spine_data->checksum << " CalResult: "
                  << calc_checksum << std::endl;
    }
}
/*!
 * send receive data and command from spine
 */
void spi_send_receive(spi_command_t *command, spi_data_t *data) {
    spi_open();
    // update driver status flag
    //spi_driver_iterations++;
    //data->spi_driver_status = spi_driver_iterations << 16;

    // transmit and receive buffers
    uint8_t tx_buf[K_WORDS_PER_MESSAGE];
    uint8_t rx_buf[K_WORDS_PER_MESSAGE];

    for (int spi_board = 0; spi_board < 2; spi_board++) {
        // copy command into spine type:
        spi_to_spine(command, &g_spine_cmd, spi_board * 2);
        memset(rx_buf, 0, K_WORDS_PER_MESSAGE);
        memset(tx_buf,0,K_WORDS_PER_MESSAGE);
        // spi message struct
        uint32touint8((uint32_t*)&g_spine_cmd, tx_buf, K_UINT32_TX_COMMAND);
        struct spi_ioc_transfer spi_message[1];
        // zero message struct.
        memset(spi_message, 0, 1 * sizeof(struct spi_ioc_transfer));

        // set up message struct
        for (int i = 0; i < 1; i++) {
            spi_message[i].bits_per_word = spi_bits_per_word;
            spi_message[i].cs_change = 1;
            spi_message[i].delay_usecs = 0;
            spi_message[i].len = K_WORDS_PER_MESSAGE;
            spi_message[i].rx_buf = (uint64_t)rx_buf;
            spi_message[i].tx_buf = (uint64_t)tx_buf;
        }
        // do spi communication
        int rv = ioctl(spi_board == 0 ? spi_1_fd : spi_2_fd, SPI_IOC_MESSAGE(1),
                       &spi_message);
        (void)rv;
        uint8touint32((uint32_t*)&g_spine_data, rx_buf, K_UINT32_RX_DATA);
        spine_to_spi(data, &g_spine_data, spi_board * 2);
    }
    spi_close();
}
/*!
 * Run SPI
 */
void spi_driver_run() {
    // in here, the driver is good
    pthread_mutex_lock(&spi_mutex);
    spi_send_receive(&spi_command_drv, &spi_data_drv);
    pthread_mutex_unlock(&spi_mutex);
}

/*!
 * Get the spi command
 */
spi_command_t *get_spi_command() {
    return &spi_command_drv;
}

/*!
 * Get the spi data
 */
spi_data_t *get_spi_data()
{
    return &spi_data_drv;
}

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