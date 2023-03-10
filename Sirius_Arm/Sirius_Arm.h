#ifndef MY_ARM_H_
#define MY_ARM_H_

#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/fwd.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/multibody/liegroup/liegroup.hpp"
#include <vector>
#include <Eigen/Core>
#include <string>
#include <iostream>
#include "Arm_Interface.h"
#include "../spi/spi.h"
#include <cmath>
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include <unistd.h>


# define M_PI           3.14159265358979323846
# define dt 0.01
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef double Scalar;
typedef pinocchio::SpecialEuclideanOperationTpl<3,Scalar> SE3Operation;


class Sirius_Arm : public Arm{
public:
    explicit Sirius_Arm(const std::string& _urdf_filename);
    ~Sirius_Arm()  override;
    void Get_Motor_Data(spi_data_t& in_data) override;
    void Set_Effector_Des(const Eigen::Matrix3d& des_orientation, const Eigen::Vector3d& des_position) override;
    void Compute_Cmd(const Eigen::Matrix3d& _des_orientation, const Eigen::Vector3d& _des_position) override; //position and velocity
    void Send_Motor_Cmd(spi_command_t& send_out_cmd) override;
    void arm_run(const Eigen::Vector3d& _des_position, const Eigen::Matrix3d& _des_orientation) override;
private:
    Vector6d q_cmd;
    Vector6d v_cmd;
    Vector6d t_cmd;
    Vector6d q_data;
    Vector6d v_data;
    Vector6d alpha;
    Vector6d a;
    Vector6d d;
    Eigen::Matrix4d T;
    std::string urdf_filename;
    std::shared_ptr<pinocchio::Model> arm_model;
    std::shared_ptr<pinocchio::Data> arm_data;
    std::shared_ptr<pinocchio::SE3> oMdes;
};

#endif
