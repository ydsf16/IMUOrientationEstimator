#pragma once 

#include <Eigen/Dense>

#include <Utils.h>

namespace OriEst {

void Update(const Eigen::Matrix3d& prior_G_R_I, 
            const Eigen::Vector3d& prior_bg, 
            const Eigen::Matrix<double, 6, 6>& prior_cov,
            const Eigen::Vector3d& acc, 
            const Eigen::Matrix3d& acc_noise,
            Eigen::Matrix3d* posterior_G_R_I,
            Eigen::Vector3d* posterior_bg,
            Eigen::Matrix<double, 6, 6>* posterior_cov);

}  // namespace OriEst