#ifndef VIOMEASUREMENTMODE_HPP_
#define VIOMEASUREMENTMODE_HPP_

#include <kalman/MeasurementModel.hpp>
#include "geometry_math_type.h"

namespace Test1 {

template<typename T>
class GpsMeasurement : public Kalman::Vector<T, 2> {
    public:
        KALMAN_VECTOR(GpsMeasurement, T, 2)

        static constexpr size_t GPS_X = 0;
        static constexpr size_t GPS_Y = 1;

        T gps_x()     const { return (*this)[ GPS_X ]; }  
        T gps_y()     const { return (*this)[ GPS_Y ]; }  

        T& gps_x()     { return (*this)[ GPS_X ]; }
        T& gps_y()     { return (*this)[ GPS_Y ]; }
};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class GpsMeasurementModel : public Kalman::MeasurementModel<State<T>, GpsMeasurement<T>, CovarianceBase> {
    public:
        typedef Test1::State<T> S;
        typedef Test1::GpsMeasurement<T> M;
        GpsMeasurementModel() {
            this->P(0,0) = T(0.05);
            this->P(1,1) = T(0.05);
        }

        M h(const S& x) const {
            M measurement;
            measurement.gps_x() = x.x();
            measurement.gps_y() = x.y();
            return measurement;
        }
};

template<typename T>
class VioMeasurement : public Kalman::Vector<T, 9> {
    public:
        KALMAN_VECTOR(VioMeasurement, T, 9)

        static constexpr size_t VIO_X = 0;
        static constexpr size_t VIO_Y = 1;
        static constexpr size_t VIO_Z = 2;

        static constexpr size_t VIO_QX = 3;
        static constexpr size_t VIO_QY = 4;
        static constexpr size_t VIO_QZ = 5;

        static constexpr size_t VIO_VX = 6;
        static constexpr size_t VIO_VY = 7;
        static constexpr size_t VIO_VZ = 8;


        T vio_x()       const { return (*this)[ VIO_X ]; } // change value    const to keep the address of *this
        T vio_y()       const { return (*this)[ VIO_Y ]; }
        T vio_z()       const { return (*this)[ VIO_Z ]; }
        // T vio_qw()       const { return (*this)[ VIO_QW ]; }
        T vio_qx()       const { return (*this)[ VIO_QX ]; }
        T vio_qy()       const { return (*this)[ VIO_QY ]; }
        T vio_qz()       const { return (*this)[ VIO_QZ ]; }

        T vio_vx()       const { return (*this)[ VIO_VX ]; }
        T vio_vy()       const { return (*this)[ VIO_VY ]; }
        T vio_vz()       const { return (*this)[ VIO_VZ ]; }

        T& vio_x()       { return (*this)[ VIO_X ]; } //return address for what? 
        T& vio_y()       { return (*this)[ VIO_Y ]; }
        T& vio_z()       { return (*this)[ VIO_Z ]; }

        T& vio_qx()       { return (*this)[ VIO_QX ]; }
        T& vio_qy()       { return (*this)[ VIO_QY ]; }
        T& vio_qz()       { return (*this)[ VIO_QZ ]; }

        T& vio_vx()       { return (*this)[ VIO_VX ]; }
        T& vio_vy()       { return (*this)[ VIO_VY ]; }
        T& vio_vz()       { return (*this)[ VIO_VZ ]; }
};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class VioMeasurementModel : public Kalman::MeasurementModel<State<T>, VioMeasurement<T>, CovarianceBase> {
    public:
        typedef Test1::State<T> S;
        typedef Test1::VioMeasurement<T> M;
        VioMeasurementModel() {}

        M h(const S& x) const {
            M measurement;
            measurement.vio_x() = x.x();
            measurement.vio_y() = x.y();
            measurement.vio_z() = x.z();

            Eigen::Quaterniond vio_q;
            measurement.vio_qx() = x.qx();
            measurement.vio_qy() = x.qy();
            measurement.vio_qz() = x.qz();

            measurement.vio_vx() = x.vx();
            measurement.vio_vy() = x.vy();
            measurement.vio_vz() = x.vz();
            return measurement;
        }
};
}

#endif