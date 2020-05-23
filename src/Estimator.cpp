#include <Estimator.h>

#include <iostream>

namespace OriEst {

Estimator::Estimator(const double gyro_noise, const double gyro_bias_noise, const double acc_noise)
    : status_(Status::kInvalid), 
      last_timestamp_(-1.),
      initializer_(std::make_unique<Initializer>()),
      propagator_(std::make_unique<Propagator>(gyro_noise, gyro_bias_noise)) { 
    acc_noise_mat_ = Eigen::Matrix3d::Identity() * acc_noise;
}

Status Estimator::Estimate(double timestamp, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc, Eigen::Matrix3d* G_R_I) {
    if (status_ == Status::kInvalid) {
        if (!initializer_->Initialize(acc, &G_R_I_)) {
            G_R_I->setIdentity();
            return Status::kInvalid;
        }

        bg_.setZero();
        cov_.setZero();
        cov_.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.5 * 0.5 * kDeg2Rad * kDeg2Rad;
        cov_.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.1 * 0.1 * kDeg2Rad * kDeg2Rad;
        last_timestamp_ = timestamp;

        // Send out.
        *G_R_I = G_R_I_;

        status_ = Status::kValid;
        return Status::kValid;
    }

    double delta_t = timestamp - last_timestamp_;
    last_timestamp_ = timestamp;

    // Propagation.
    Eigen::Matrix3d prior_G_R_I;
    Eigen::Vector3d prior_bg;
    Eigen::Matrix<double, 6, 6> prior_cov;
    propagator_->PropagateMeanAndCov(G_R_I_, bg_, cov_, gyro, delta_t, &prior_G_R_I, &prior_bg, &prior_cov);

    acc_buffer_.push_back(acc);
    if (acc_buffer_.size() > config_.acc_buffer_size) {
        acc_buffer_.pop_front();
    }
    // compute mean.
    Eigen::Vector3d mean_acc(0., 0., 0.);
    for (const Eigen::Vector3d& one_acc : acc_buffer_) {
        mean_acc += one_acc;
    }
    mean_acc = mean_acc / static_cast<double>(acc_buffer_.size());

    // Update
    Update(prior_G_R_I, prior_bg, prior_cov, mean_acc, acc_noise_mat_, &G_R_I_, &bg_, &cov_);

    std::cout << "GyroBias: " << std::fixed << bg_.transpose() << std::endl;

    // Send out.
    *G_R_I = G_R_I_;

    return Status::kValid;
}

}  // namespace OriEst