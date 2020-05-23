#pragma once

#include <memory>
#include <Eigen/Core>

#include <Initializer.h>
#include <Propagator.h>
#include <Updater.h>

namespace OriEst {

enum class Status {
    kValid,
    kInvalid
};

class Estimator {
public:
    Estimator(const double gyro_noise, const double gyro_bias_noise, const double acc_noise);

    Status Estimate(double timestamp, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc, Eigen::Matrix3d* G_R_I);

private:
    // State.
    Eigen::Matrix3d G_R_I_;
    Eigen::Vector3d bg_;
    Eigen::Matrix<double, 6, 6> cov_;

    Status status_;
    double last_timestamp_;
    Eigen::Matrix3d acc_noise_mat_;

    std::unique_ptr<Initializer> initializer_;
    std::unique_ptr<Propagator> propagator_;
};

}  // namespace OriEst