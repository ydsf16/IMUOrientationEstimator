#include <Estimator.h>

#include <iostream>
#include <glog/logging.h>

#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Please input: Rosbag file!\n";
        return EXIT_FAILURE;
    }

    const std::string rosbag_path = argv[1];
    const std::string kImuTopic = "/imu0";

    const double gyro_noise = 1e-6;
    const double gyro_bias_noise = 1e-8;
    const double acc_noise = 1e-3;
    OriEst::Estimator orientation_estimatro(gyro_noise, gyro_bias_noise, acc_noise);

    /****** Play Data ******/
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
            // Waitkey.
            std::cin.ignore();        
        } 
    }

    return EXIT_SUCCESS;
}