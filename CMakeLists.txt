cmake_minimum_required(VERSION 2.8.3)
project(IMUOrientationEstimator)

# Set C++14
add_compile_options(-std=c++14)

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

add_executable(RunCSVData  example/RunCSVData.cpp)
#target_link_libraries(test_${PROJECT_NAME} )

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    rosbag
)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(RunRosbag example/RunRosbag.cpp)
target_link_libraries(RunRosbag
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
)