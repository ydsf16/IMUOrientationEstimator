#include <Initializer.h>

#include <iostream>

namespace OriEst {

bool Initializer::Initialize(const Eigen::Vector3d& acc, Eigen::Matrix3d* G_R_I) {
    // Feed new data.
    acc_buffer_.push_back(acc);
    if (acc_buffer_.size() <= config_.acc_buffer_size) {
        return false;
    }
    acc_buffer_.pop_front();

    // Compute mean acc.
    Eigen::Vector3d mean_acc(0., 0., 0.);
    for (const Eigen::Vector3d& one_acc : acc_buffer_) {
        mean_acc += one_acc;
    }
    mean_acc = mean_acc / static_cast<double>(acc_buffer_.size());

    // Compute std acc.
    Eigen::Vector3d std_acc(0., 0., 0.);
    for (const Eigen::Vector3d& one_acc : acc_buffer_) {
        std_acc += (one_acc - mean_acc).cwiseAbs2();
    }
    std_acc = (std_acc / static_cast<double>(acc_buffer_.size())).cwiseSqrt();
    if (std_acc.norm() > config_.max_acc_std) {
        std::cout << "[Initialize]: Initializaion failed. Too big acc std: " << std::fixed << std_acc.transpose(); 
        return false;
    }

    // Get initial orientaion.
    const Eigen::Vector3d z_axis = mean_acc.normalized();
    const Eigen::Vector3d x_axis = 
        (Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX()).normalized();
    const Eigen::Vector3d y_axis = (z_axis.cross(x_axis)).normalized();
    
    Eigen::Matrix3d I_R_G;
    I_R_G.block<3, 1>(0, 0) = x_axis;
    I_R_G.block<3, 1>(0, 1) = y_axis;
    I_R_G.block<3, 1>(0, 2) = z_axis;

    *G_R_I = I_R_G.transpose();

    return true;
}

}  // namespace OriEst