#include <Updater.h>

namespace OriEst {

void Update(const Eigen::Matrix3d& prior_G_R_I, 
            const Eigen::Vector3d& prior_bg, 
            const Eigen::Matrix<double, 6, 6>& prior_cov,
            const Eigen::Vector3d& acc, 
            const Eigen::Matrix3d& acc_noise,
            Eigen::Matrix3d* posterior_G_R_I,
            Eigen::Vector3d* posterior_bg,
            Eigen::Matrix<double, 6, 6>* posterior_cov) {
    // Residual
    Eigen::Vector3d gravity_vec(0., 0., 1.);
    Eigen::Vector3d residual = acc.normalized() - prior_G_R_I.transpose() * gravity_vec;
    
    // Jacobian.
    Eigen::Matrix<double, 3, 6> H;
    H.setZero();
    H.block<3, 3>(0, 0) = SkewMat(prior_G_R_I.transpose() * gravity_vec);

    // Kalman gain.
    Eigen::Matrix<double, 6, 3> K = prior_cov * H.transpose() * (H * prior_cov * H.transpose() + acc_noise).inverse();

    // Delta x.
    Eigen::Matrix<double, 6, 1> delta_x = K * residual;

    // Update state.
    Eigen::Matrix3d delta_Rot;
    if (delta_x.topRows<3>().norm() < 1e-12) {
        delta_Rot = 
            Eigen::Quaterniond(
                Eigen::Matrix3d::Identity() + SkewMat(delta_x.topRows<3>())).normalized().toRotationMatrix();
    } else {
        const double angle = delta_x.topRows<3>().norm();
        const Eigen::Vector3d axis = delta_x.topRows<3>() / angle;
        delta_Rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }
    *posterior_G_R_I = prior_G_R_I * delta_Rot;
    *posterior_bg = prior_bg + delta_x.bottomRows<3>();

    // Update covariance.
    const Eigen::Matrix<double, 6, 6> I_mins_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    *posterior_cov = I_mins_KH * prior_cov * I_mins_KH.transpose() + K * acc_noise * K.transpose();
}
} // namesapce OriEst