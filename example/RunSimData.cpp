#include <iostream>
#include <Estimator.h>
#include <opencv2/viz.hpp>

int main(int argc, char** argv) {
    // Set viz.
    cv::viz::Viz3d viz_windows("IMU Orientation");
    viz_windows.showWidget("ENU Frame", cv::viz::WCoordinateSystem(0.5));
    viz_windows.showWidget("IMU Frame", cv::viz::WCoordinateSystem(0.8));

    const double gyro_noise = 1e-4;
    const double gyro_bias_noise = 1e-12;
    const double acc_noise = 1e-4;
    OriEst::Estimator orientation_estimatro(gyro_noise, gyro_bias_noise, acc_noise);

    for (double timestamp = 0.; timestamp < 100.; timestamp += 0.01) {
        Eigen::Vector3d acc(0., 0., 9.8);
        Eigen::Vector3d gyro(0., 0., 10. * OriEst::kDeg2Rad);
        gyro += Eigen::Vector3d(1 * OriEst::kDeg2Rad, 2 * OriEst::kDeg2Rad, 0.01 * OriEst::kDeg2Rad);

        Eigen::Matrix3d G_R_I;
        OriEst::Status status = orientation_estimatro.Estimate(timestamp, gyro, acc, &G_R_I);
        
        std::cout << "status " << static_cast<int>(status) << ", Rot\n" << G_R_I << "\n\n";

        // Show result.
        cv::Mat cv_R =(cv::Mat_<float>(3, 3) << G_R_I(0, 0), G_R_I(0, 1), G_R_I(0, 2), 
                                                G_R_I(1, 0), G_R_I(1, 1), G_R_I(1, 2), 
                                                G_R_I(2, 0), G_R_I(2, 1), G_R_I(2, 2));
        cv::Affine3d pose;
        pose.linear(cv_R);
        pose.translate(cv::Vec3d(0., 0., 0.));
        viz_windows.setWidgetPose("IMU Frame", pose);
        viz_windows.spinOnce(1, true);
    }

    return EXIT_SUCCESS;
}
