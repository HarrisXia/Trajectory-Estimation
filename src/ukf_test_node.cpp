#define _USE_MATH_DEFINES
#define LOG_FLAG
#include <cmath>


#include "ukf_test/SystemModel.hpp"
#include "ukf_test/VioMeasurementModel.hpp"

#include <kalman/UnscentedKalmanFilter.hpp>
#include "ukf_test/predict_part.hpp"

#include <iostream>
#include <random>
#include <chrono>

#include "ros/ros.h"

#ifdef LOG_FLAG
#include <string>
#include <iostream>
#include <fstream>
#endif

typedef float T;
typedef Test1::State<T> S_;

typedef Test1::Control<T> C_;
typedef Test1::SystemModel<T> SM_;


typedef Test1::GpsMeasurement<T> GMeas_;
typedef Test1::GpsMeasurementModel<T> GM_;

typedef Test1::VioMeasurement<T> VMeas_;
typedef Test1::VioMeasurementModel<T> VM_;

int main(int argc, char** argv) {
    ros::init(argc, argv, "ukf_node");
    ros::NodeHandle node("~");
    std::cout << "hello" << std::endl;
    Filter_predict_part<T, S_, C_, SM_, GMeas_, GM_, VMeas_, VM_ > filter_core(node);
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
    return 0;
}