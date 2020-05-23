#include <Estimator.h>

#include <iostream>

#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/viz.hpp>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Please input: Rosbag file!\n";
        return EXIT_FAILURE;
    }

    // Set viz.
    cv::viz::Viz3d viz_windows("IMU Orientation");
    viz_windows.showWidget("ENU Frame", cv::viz::WCoordinateSystem(0.8));
    viz_windows.showWidget("IMU Frame", cv::viz::WCoordinateSystem(0.5));

    // Init OriEst.
    const double gyro_noise = 1e-6;
    const double gyro_bias_noise = 1e-8;
    const double acc_noise = 1e-6;
    OriEst::Estimator orientation_estimatro(gyro_noise, gyro_bias_noise, acc_noise);

    // Play Data
    const std::string rosbag_path = argv[1];
    const std::string kImuTopic = "/imu0";
    rosbag::Bag bag;
    bag.open(rosbag_path);  // BagMode is Read by default
    for (rosbag::MessageInstance const m : rosbag::View(bag)) {
        if (m.getTopic() == kImuTopic) {
            const auto& msg = m.instantiate<sensor_msgs::Imu>();
            const double timestamp = msg->header.stamp.toSec();
            Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

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
    }

    return EXIT_SUCCESS;
}