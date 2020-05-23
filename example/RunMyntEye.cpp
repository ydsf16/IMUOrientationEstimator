#include <mynteye/logger.h>
#include <mynteye/device/device.h>
#include <mynteye/device/utils.h>
#include <mynteye/util/times.h>

#include <Estimator.h>
#include <opencv2/viz.hpp>

MYNTEYE_USE_NAMESPACE

int main(int argc, char **argv) {
    // Init out orientation estimator.
    const double gyro_noise = 1e-6;
    const double gyro_bias_noise = 1e-8;
    const double acc_noise = 1e-3;
    OriEst::Estimator orientation_estimatro(gyro_noise, gyro_bias_noise, acc_noise);

    // Set viz.
    cv::viz::Viz3d viz_windows("IMU Orientation");
    viz_windows.showWidget("ENU Frame", cv::viz::WCoordinateSystem(0.3));
    viz_windows.showWidget("IMU Frame", cv::viz::WCoordinateSystem(0.8));

    // Set up our camera.
    auto &&device = device::select();
    if (!device) return EXIT_FAILURE;

    bool ok;
    auto &&request = device::select_request(device, &ok);
    if (!ok) return EXIT_FAILURE;
    device->ConfigStreamRequest(request);

    std::size_t imu_count = 0;
    device->SetMotionCallback([&imu_count, &orientation_estimatro, &viz_windows](const device::MotionData &data) {
        CHECK_NOTNULL(data.imu);
        ++imu_count;
        std::cout << "Imu count: " << imu_count << std::endl;
        std::cout << ", timestamp: " << data.imu->timestamp
                  << ", accel_x: " << data.imu->accel[0]
                  << ", accel_y: " << data.imu->accel[1]
                  << ", accel_z: " << data.imu->accel[2]
                  << ", gyro_x: " << data.imu->gyro[0]
                  << ", gyro_y: " << data.imu->gyro[1]
                  << ", gyro_z: " << data.imu->gyro[2]
                  << ", temperature: " << data.imu->temperature << std::endl;

        Eigen::Vector3d acc(data.imu->accel[0] * OriEst::kGravity, 
                            data.imu->accel[1] * OriEst::kGravity, 
                            data.imu->accel[2] * OriEst::kGravity);
        Eigen::Vector3d gyro(data.imu->gyro[0] * OriEst::kDeg2Rad, 
                                data.imu->gyro[1] * OriEst::kDeg2Rad, 
                                data.imu->gyro[2] * OriEst::kDeg2Rad);

        double timestamp = data.imu->timestamp * 1e-6;
        Eigen::Matrix3d G_R_I;
        OriEst::Status status = orientation_estimatro.Estimate(timestamp, gyro, acc, &G_R_I);  

        // Show result.
        cv::Mat cv_R =(cv::Mat_<float>(3, 3) << G_R_I(0, 0), G_R_I(0, 1), G_R_I(0, 2), 
                                                G_R_I(1, 0), G_R_I(1, 1), G_R_I(1, 2), 
                                                G_R_I(2, 0), G_R_I(2, 1), G_R_I(2, 2));
        cv::Affine3d pose;
        pose.linear(cv_R);
        pose.translate(cv::Vec3d(0., 0., 0.));
        viz_windows.setWidgetPose("IMU Frame", pose); 

        if (imu_count % 10 == 0) {
            viz_windows.spinOnce(1);
        }
    });

    // Enable this will cache the motion datas until you get them.
    device->EnableMotionDatas();
    device->Start(Source::ALL);
    
    // Wait key to stop.
    std::cin.ignore();

    device->Stop(Source::ALL);

    return EXIT_SUCCESS;
}
