#ifndef TYPEDEF_H_
#define TYPEDEF_H_
#include <memory>
#include "../spi/spi.h"
#include <Eigen/Core>

class Arm{
public:
    virtual ~Arm(){};    //Create an Arm
    static std::shared_ptr<Arm> create_arm(const std::string& _urdf_filename);
    //Get the Motor Data(Position and Velocity)
    virtual void Get_Motor_Data(spi_data_t& in_data) = 0;
    //Update the joint placements and spatial velocities according to the current joint configuration and velocity.
    //this function is setting the destination of arm
    virtual void Set_Effector_Des(const Eigen::Matrix3d& des_orientation, const Eigen::Vector3d& des_position) = 0;
    //this function is to compute the destination of the arm
    virtual void Compute_Cmd(const Eigen::Matrix3d& _des_orientation, const Eigen::Vector3d& _des_position) = 0;
    //this function is to compute out the torque
    //this function is to send cmd to motors
    virtual void Send_Motor_Cmd(spi_command_t& send_out_cmd) = 0;
    virtual void arm_run(const Eigen::Vector3d& _des_position, const Eigen::Matrix3d& _des_orientation) = 0;
};

#endif
