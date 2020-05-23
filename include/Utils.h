#pragma once 

#include <Eigen/Core>

namespace OriEst {

constexpr double kDeg2Rad = M_PI / 180.;
constexpr double kRad2Deg = 180. / M_PI;
constexpr double kGravity = 9.8;

inline Eigen::Matrix3d SkewMat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w <<  0.,   -v(2),  v(1),
          v(2),  0.,   -v(0),
         -v(1),  v(0),  0.;
    return w;
}


}  // namespace OriEst