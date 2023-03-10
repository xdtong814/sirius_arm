#include "Sirius_Arm.h"
#include <memory>


Sirius_Arm::Sirius_Arm(const std::string& _urdf_filename)
        : urdf_filename(_urdf_filename)
{
    arm_model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_filename,*arm_model);
    oMdes = std::make_shared<pinocchio::SE3>(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 0.));
    arm_data = std::make_shared<pinocchio::Data>(*arm_model);
}

Sirius_Arm::~Sirius_Arm() = default;


void Sirius_Arm::Get_Motor_Data(spi_data_t& in_data)
{
    for(int i = 0; i < 6; i++) {
//        q_data[i] = in_data->q_motors[i];
//        v_data[i] = in_data->qd_motors[i];
        q_data[i] = in_data.q_abad[i];
        //v_data[i] = qd_data_drv[i];
    }
}


void Sirius_Arm::Set_Effector_Des(const Eigen::Matrix3d& _des_orientation, const Eigen::Vector3d& _des_position)
{
    oMdes->rotation() = _des_orientation;
    oMdes->translation() = _des_position;
}

void Sirius_Arm::Compute_Cmd(const Eigen::Matrix3d& _des_orientation, const Eigen::Vector3d& _des_position)
{
    const int JOINT_ID = 6;
    const double eps  = 1e-4;
    const int IT_MAX  = 1000;
    const double DT   = 1e-1;
    const double damp = 1e-6;

    oMdes->rotation() = _des_orientation;
    oMdes->translation() = _des_position;
    pinocchio::Data::Matrix6 JJt;
    Vector6d err;
    bool success = false;
    Eigen::VectorXd v(arm_model->nv);
    pinocchio::Data::Matrix6x J(6, arm_model->nv);
    J.setZero();
    Eigen::VectorXd q = q_data;

    for (int i=0;;i++)
    {

        pinocchio::forwardKinematics(*arm_model,*arm_data,q);
        const pinocchio::SE3 dMi = oMdes->actInv(arm_data->oMi[JOINT_ID]);
        err = pinocchio::log6(dMi).toVector();
//        bool flag = true;
//        for(int m = 0; m < 6; m++){
//            if(q[m] > arm_model->upperPositionLimit[m] || q[m] < arm_model->lowerPositionLimit[m])
//                flag = false;
//        }
        if(err.norm() < eps)
        {
            success = true;
            break;
        }
        if (i >= IT_MAX)
        {
            success = false;
            break;
        }
        pinocchio::computeJointJacobian(*arm_model,*arm_data,q,JOINT_ID,J);
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
        q = pinocchio::integrate(*arm_model,q,v*DT);

    }

    if(success)
    {
//        for(int i = 0; i < 6; i++){
//            normalize(q[i]);
//        }
        q_cmd = q;
        std::cout << "Convergence achieved!" << std::endl;
    }
    else
    {
        std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
    }

    std::cout << "result: " << q_cmd.transpose() << std::endl;
    //std::cout << "final error: " << err.transpose() << std::endl;
}


void Sirius_Arm::Send_Motor_Cmd(spi_command_t& send_out_cmd)
{
    for(int i = 0; i < 6; i++) {
//        send_out_cmd->q_des_motors[i] = q_cmd[i];
//        send_out_cmd->qd_des_motors[i] = v_cmd[i];
//        send_out_cmd->tau_motors_ff[i] = t_cmd[i];
        send_out_cmd.q_des_abad[i] = q_cmd[i];
        //qd_cmd_drv[i] = v_cmd[i];
        //qdd_cmd_drv[i] = t_cmd[i];
    }
}

void Sirius_Arm::arm_run(const Eigen::Vector3d& _des_position, const Eigen::Matrix3d& _des_orientation)
{
    this->Set_Effector_Des(_des_orientation, _des_position);
    this->Compute_Cmd(_des_orientation, _des_position);

}

std::shared_ptr<Arm> Arm::create_arm(const std::string &_urdf_filename) {
    return std::shared_ptr<Arm>(new Sirius_Arm(_urdf_filename));
}
//
// Created by lingwei on 2/24/23.
//
