#pragma once

#include <deque>

#include <Eigen/Dense>

namespace OriEst {

class Initializer {
public:
    struct Config {
        size_t acc_buffer_size = 10;
        double max_acc_std = 0.5;
    };

    Initializer() = default;

    bool Initialize(const Eigen::Vector3d& acc, Eigen::Matrix3d* G_R_I);

private:
    const Config config_;

    std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> acc_buffer_;
};

} // namespace OriEst