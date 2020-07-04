cmake_minimum_required(VERSION 2.8.3)
project(IMUOrientationEstimator)

# Set C++14
add_compile_options(-std=c++14)

# Release mode.
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)
add_library(${PROJECT_NAME} SHARED 
    src/Initializer.cpp
    src/Propagator.cpp
    src/Updater.cpp
    src/Estimator.cpp
)
target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBS})

option(BUILD_EXAMPLE "Build example code" ON)

if (BUILD_EXAMPLE) 
    find_package(OpenCV 4 REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    # ROSBAG
    find_package(catkin REQUIRED COMPONENTS
        sensor_msgs
        rosbag
    )
    include_directories(${catkin_INCLUDE_DIRS})

    add_executable(RunRosbag example/RunRosbag.cpp)
    target_link_libraries(RunRosbag
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
    )

    # MYNTEYE
    find_package(mynteye REQUIRED)
    message(STATUS "Found mynteye: ${mynteye_VERSION}")
    add_executable(RunMyntEye example/RunMyntEye.cpp)
    target_link_libraries(RunMyntEye ${PROJECT_NAME} mynteye ${OpenCV_LIBS})

endif()