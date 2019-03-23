#define _USE_MATH_DEFINES
#include <cmath>


#include "ukf_test/SystemModel.hpp"
#include "ukf_test/VioMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>

#include "ros/ros.h"
#include <string>
#include <iostream>
#include <fstream>
// #include <time.h>

typedef double T;

typedef Test1::State<T> State;
typedef Test1::Control<T> Control;
typedef Test1::SystemModel<T> SystemModel;

typedef Test1::VioMeasurement<T> VioMeasurement;
typedef Test1::VioMeasurementModel<T> VioModel;

int main(int argc, char** argv) {
    State x;
    x.setZero();
    // x.qw() = 1.0f;

    std::ofstream logger_predict;
    std::ofstream logger_ukf;
    std::ofstream logger_u;
    std::ofstream logger_m;

    logger_predict.open("/home/lhc/work/estimator_ws/src/ukf_test/logger/predict.csv", std::ios::out);
    logger_ukf.open("/home/lhc/work/estimator_ws/src/ukf_test/logger/ukf.csv", std::ios::out);
    logger_u.open("/home/lhc/work/estimator_ws/src/ukf_test/logger/u.csv", std::ios::out);
    logger_m.open("/home/lhc/work/estimator_ws/src/ukf_test/logger/m.csv", std::ios::out);
    logger_predict << "x" << "," << "y" << "," << "z" << "," << "vx" << "," << "vy" << "," << "vz" << "," << "phi" << "," << "theta" << "," << "psi" << std::endl;
    logger_ukf << "x" << "," << "y" << "," << "z" << "," << "vx" << "," << "vy" << "," << "vz" << "," << "phi" << "," << "theta" << "," << "psi" << std::endl;
    logger_u << "ax" << "," << "ay" << "," << "az" << "," << "wx" << "," << "wy" << "," << "wz" << std::endl;
    logger_m << "x" << "," << "y" << "," << "z" << "," << "phi" << "," << "theta" << "," << "psi" << std::endl;

    Control u;
    SystemModel sys;
    VioModel vm;

    std::default_random_engine generator;
    generator.seed( std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<T> noise(0, 1);

    Kalman::UnscentedKalmanFilter<State> ukf(1);

    ukf.init(x);

    T pos_systemNoise = 0.2f;
    T vel_systemNoise = 0.3f;
    T att_systemNoise = 0.2f;
    T acc_Noise = 0.2f;
    T gyro_Noise = 0.02f;
    T measureNoise = 0.1;
    T attmeasureNoise = 0.5;

    const size_t N = 2000;

    int _j = 0;
    for (size_t i = 1; i <= N; i++) {
        u.ax() = std::sin( T(i) * T(M_PI) * T(2) / T(N) ) * 1.0f;
        u.ay() = std::sin( T(i) * T(M_PI) * T(2) / T(N) ) * 0.5f;
        u.az() = std::sin( T(i) * T(M_PI) * T(2) / T(N) ) * 0.2f + 9.8f;
        u.wx() = std::sin( T(i) * T(M_PI) * T(2) / T(N) ) * 0.2f;
        u.wy() = std::sin( T(i) * T(M_PI) * T(2) / T(N) ) * 0.2f;
        u.wz() = std::sin( T(i) * T(M_PI) * T(2) / T(N) ) * 0.2f;
        u.dt() = 0.005f;
        std::cout << "***********" << i << "**************" <<std::endl;
        x = sys.f(x,u);

        x.x() += pos_systemNoise*noise(generator);
        x.y() += pos_systemNoise*noise(generator);
        x.z() += pos_systemNoise*noise(generator);
        x.vx() += vel_systemNoise*noise(generator);
        x.vy() += vel_systemNoise*noise(generator);
        x.vz() += vel_systemNoise*noise(generator);
        // x.qw() += att_systemNoise*noise(generator);
        x.qx() += att_systemNoise*noise(generator);
        x.qy() += att_systemNoise*noise(generator);
        x.qz() += att_systemNoise*noise(generator);
        // x.bax() += pos_systemNoise*noise(generator);
        // x.bay() += pos_systemNoise*noise(generator);
        // x.baz() += pos_systemNoise*noise(generator);
        // x.bwx() += pos_systemNoise*noise(generator);
        // x.bwy() += pos_systemNoise*noise(generator);
        // x.bwz() += pos_systemNoise*noise(generator);

        u.ax() += acc_Noise*noise(generator);
        u.ay() += acc_Noise*noise(generator);
        u.az() += acc_Noise*noise(generator);
        u.wx() += gyro_Noise*noise(generator);
        u.wy() += gyro_Noise*noise(generator);
        u.wz() += gyro_Noise*noise(generator);

        logger_predict << x.x() << ",";
        logger_predict << x.y() << ",";
        logger_predict << x.z() << ",";
        logger_predict << x.vx() << ",";
        logger_predict << x.vy() << ",";
        logger_predict << x.vz() << ",";
        logger_predict << x.qx() << ",";
        logger_predict << x.qy() << ",";
        logger_predict << x.qz() << std::endl;

        logger_u << u.ax() << ",";
        logger_u << u.ay() << ",";
        logger_u << u.az() << ",";
        logger_u << u.wx() << ",";
        logger_u << u.wy() << ",";
        logger_u << u.wz() << std::endl;

        auto x_ukf = ukf.predict(sys, u);

        logger_ukf << x_ukf.x() << ",";
        logger_ukf << x_ukf.y() << ",";
        logger_ukf << x_ukf.z() << ",";
        logger_ukf << x_ukf.vx() << ",";
        logger_ukf << x_ukf.vy() << ",";
        logger_ukf << x_ukf.vz() << ",";
        logger_ukf << x_ukf.qx() << ",";
        logger_ukf << x_ukf.qy() << ",";
        logger_ukf << x_ukf.qz() << std::endl;

        if ( _j > 10 ) {
            _j = 0;
            VioMeasurement  vio_state = vm.h(x);
            vio_state.vio_x() += measureNoise*noise(generator);
            vio_state.vio_y() += measureNoise*noise(generator);
            vio_state.vio_z() += measureNoise*noise(generator);
            // vio_state.vio_qw() += measureNoise*noise(generator);
            vio_state.vio_qx() += attmeasureNoise*noise(generator);
            vio_state.vio_qy() += attmeasureNoise*noise(generator);
            vio_state.vio_qz() += attmeasureNoise*noise(generator);
            logger_m << vio_state.vio_x() << ",";
            logger_m << vio_state.vio_y() << ",";
            logger_m << vio_state.vio_z() << ",";
            logger_m << vio_state.vio_qx() << ",";
            logger_m << vio_state.vio_qy() << ",";
            logger_m << vio_state.vio_qz() << std::endl;

            x_ukf = ukf.update(vm, vio_state);
        } else {
            _j ++;
        }
        std::cout << "q:  [" <<  x.qx() << ", " << x.qy() << ", " << x.qz() << "]" << std::endl;
        std::cout << "qu: [" <<  x_ukf.qx() << ", " << x_ukf.qy() << ", " << x_ukf.qz() << "]" << std::endl;
        std::cout << "x:  [" << x.vx() <<", "<< x_ukf.vx() << "]"<< std::endl;
        // std::cout << "y:[" << x.y() <<", "<< x_ukf.y() << "]"<< std::endl;
        // std::cout << "z:[" << x.z() <<", "<< x_ukf.z() << "]"<<std::endl;
    }

}