#include "Sirius_Arm/Arm_Interface.h"
#include "spi/spi.h"
#include "Sirius_Arm/Sirius_Arm.h"
#include "Eigen/Core"

//use_spi spi_com;
Eigen::Matrix3d des_orientation;
Eigen::Vector3d des_position;
spi_command_t spi_command_drv;
spi_data_t spi_data_drv;
spine_cmd_t g_spine_cmd;
spine_data_t g_spine_data;
pthread_mutex_t mutex;

int main(int argc, char  *argv[]) {
    // Create a robot_arm
    std::shared_ptr <Arm> Arm1 = Arm::create_arm("/home/lingwei/catkin_ws/src/six_dof_arm/urdf/six_dof_arm.urdf");
    des_orientation << 1., 0., 0., 0., 1., 0., 0., 0., 1.;
    des_position << -0.4, 0., 0.4;
    pthread_mutex_lock(&mutex);
    spi_send_receive(&spi_command_drv, &spi_data_drv);
    Arm1->Get_Motor_Data(spi_data_drv);
    Arm1->arm_run(des_position, des_orientation);
    Arm1->Send_Motor_Cmd(spi_command_drv);
    pthread_mutex_unlock(&mutex);
}
//
// Created by lingwei on 2/27/23.
//
