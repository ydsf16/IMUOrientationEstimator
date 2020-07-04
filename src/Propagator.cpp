#include <Propagator.h>

#include <Utils.h>

namespace OriEst {

Propagator::Propagator(const double& gyro_noise, const double& gyro_bias_noise)
    : gyro_noise_(gyro_noise), gyro_bias_noise_(gyro_bias_noise) { }

void Propagator::PropagateMeanAndCov(const Eigen::Matrix3d& begin_G_R_I, 
                                     const Eigen::Vector3d& begin_bg, 
                                     const Eigen::Matrix<double, 6, 6>& begin_cov,
                                     const Eigen::Vector3d& gyro, 
                                     const double delta_t,
                                     Eigen::Matrix3d* end_G_R_I,
                                     Eigen::Vector3d* end_bg,
                                     Eigen::Matrix<double, 6, 6>* end_cov) {
    // Mean propagation.
    const Eigen::Vector3d unbiased_gyro = gyro - begin_bg;
    const Eigen::Vector3d angle_vec = unbiased_gyro * delta_t;
    Eigen::Matrix3d delta_rot;
    if (angle_vec.norm() < 1e-12) {
        delta_rot = Eigen::Quaterniond(Eigen::Matrix3d::Identity() + SkewMat(angle_vec)).normalized().toRotationMatrix();
    } else {
        const double angle = angle_vec.norm();
        const Eigen::Vector3d axis = angle_vec / angle;
        delta_rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }
    *end_G_R_I = begin_G_R_I * delta_rot;
    *end_bg = begin_bg;

    // Jacobian.
    Eigen::Matrix<double, 6, 6> Fx;
    Fx.topLeftCorner<3, 3>() = delta_rot.transpose();
    Fx.topRightCorner<3,3>() = -Eigen::Matrix3d::Identity() * delta_t;
    Fx.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Zero();
    Fx.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * gyro_noise_ * delta_t * delta_t;
    Q.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * gyro_bias_noise_ * delta_t;

    *end_cov = Fx * begin_cov * Fx.transpose() + Q;
}

}  // namespac OriEst