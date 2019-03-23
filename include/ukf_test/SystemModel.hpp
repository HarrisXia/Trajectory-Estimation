#ifndef SYSTEMMODEL_HPP_
#define SYSTEMMODEL_HPP_

#include <kalman/SystemModel.hpp>
#include <iostream>
#include "geometry_math_type.h"

#define ONE_G 9.78036
namespace Test1 {

template<typename T>
class State : public Kalman::Vector<T, 15> {
    public:
        KALMAN_VECTOR(State, T, 15)

        //! X-position
        static constexpr size_t X = 0;
        //! Y-position
        static constexpr size_t Y = 1;
        //! Z-position
        static constexpr size_t Z = 2;
        //! X-velocity
        static constexpr size_t VX = 3;
        //! Y-velocity
        static constexpr size_t VY = 4;
        //! Z-velocity
        static constexpr size_t VZ = 5;
        static constexpr size_t AX = 6;
        static constexpr size_t AY = 7;
        static constexpr size_t AZ = 8;
        //! X-orientation
        static constexpr size_t QX = 9;
        //! Y-orientation
        static constexpr size_t QY = 10;
        //! Z-orientation
        static constexpr size_t QZ = 11;
        //! X-omega
        static constexpr size_t WX = 12;
        //! Y-omega
        static constexpr size_t WY = 13;
        //! Z-omega
        static constexpr size_t WZ = 14;       
        
        T x()        const { return (*this)[ X ]; }
        T y()        const { return (*this)[ Y ]; }
        T z()        const { return (*this)[ Z ]; }
        T vx()       const { return (*this)[ VX ]; }
        T vy()       const { return (*this)[ VY ]; }
        T vz()       const { return (*this)[ VZ ]; }
        T ax()       const { return (*this)[ AX ]; }
        T ay()       const { return (*this)[ AY ]; }
        T az()       const { return (*this)[ AZ ]; }
        T qx()       const { return (*this)[ QX ]; }
        T qy()       const { return (*this)[ QY ]; }
        T qz()       const { return (*this)[ QZ ]; }
        T wx()       const { return (*this)[ WX ]; }
        T wy()       const { return (*this)[ WY ]; }
        T wz()       const { return (*this)[ WZ ]; }

        T& x()        { return (*this)[ X ]; }
        T& y()        { return (*this)[ Y ]; }
        T& z()        { return (*this)[ Z ]; }
        T& vx()       { return (*this)[ VX ]; }
        T& vy()       { return (*this)[ VY ]; }
        T& vz()       { return (*this)[ VZ ]; }
        T& ax()       { return (*this)[ AX ]; }
        T& ay()       { return (*this)[ AY ]; }
        T& az()       { return (*this)[ AZ ]; }
        T& qx()       { return (*this)[ QX ]; }
        T& qy()       { return (*this)[ QY ]; }
        T& qz()       { return (*this)[ QZ ]; }
        T& wx()       { return (*this)[ WX ]; }
        T& wy()       { return (*this)[ WY ]; }
        T& wz()       { return (*this)[ WZ ]; }

};


template<typename T>
class Control : public Kalman::Vector<T, 7> {
    public:
        KALMAN_VECTOR(Control, T, 7)

        //! accelerator
        static constexpr size_t aX = 0;
        static constexpr size_t aY = 1;
        static constexpr size_t aZ = 2;
        //! gyro
        static constexpr size_t wX = 3;
        static constexpr size_t wY = 4;
        static constexpr size_t wZ = 5;

        static constexpr size_t DT = 6;

        T ax()      const { return (*this)[ aX ]; }
        T ay()      const { return (*this)[ aY ]; }
        T az()      const { return (*this)[ aZ ]; }
        T wx()      const { return (*this)[ wX ]; }
        T wy()      const { return (*this)[ wY ]; }
        T wz()      const { return (*this)[ wZ ]; }
        T dt()      const { return (*this)[ DT ]; }

        T& ax()      { return (*this)[ aX ]; }
        T& ay()      { return (*this)[ aY ]; }
        T& az()      { return (*this)[ aZ ]; }
        T& wx()      { return (*this)[ wX ]; }
        T& wy()      { return (*this)[ wY ]; }
        T& wz()      { return (*this)[ wZ ]; }
        T& dt()      { return (*this)[ DT ]; }

};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::SystemModel<State<T>, Control<T>, CovarianceBase> {
    public:
        typedef Test1::State<T> S;
        typedef Test1::Control<T> C;

        SystemModel() {
            this->P(0,0) = T(1.0);
            this->P(1,1) = T(1.0);
            this->P(2,2) = T(1.0);
            this->P(3,3) = T(1.0);
            this->P(4,4) = T(1.0);
            this->P(5,5) = T(1.0);
            this->P(6,6) = T(1.0);
            this->P(7,7) = T(2.0);
            this->P(8,8) = T(2.0);
            this->P(9,9) = T(2.0);
            this->P(10,10) = T(2.0);
            this->P(11,11) = T(2.0);
        }
        
        mutable T ax_out;
        mutable T ay_out;
        mutable T az_out;
        mutable T wx_out;
        mutable T wy_out;
        mutable T wz_out;

        S f(const S& x, const C& u) const {
            S x_;
            T ax = u.ax();
            T ay = u.ay();
            T az = u.az();

            T x_new = x.x() + x.vx() * u.dt() + 0.5 * x.ax() * u.dt() * u.dt();
            T y_new = x.y() + x.vy() * u.dt() + 0.5 * x.ay() * u.dt() * u.dt();
            T z_new = 0;

            Eigen::Vector3d e_;
            e_(0) = x.qx()/T(180)*T(M_PI);
            e_(1) = x.qy()/T(180)*T(M_PI);
            e_(2) = x.qz()/T(180)*T(M_PI);
            Eigen::Matrix3d R_;
            get_dcm_from_euler(R_, e_);

            T ga_x = R_(0, 0) * ax + R_(0, 1) * ay + R_(0, 2) * az;
            T ga_y = R_(1, 0) * ax + R_(1, 1) * ay + R_(1, 2) * az;
            T ga_z = R_(2, 0) * ax + R_(2, 1) * ay + R_(2, 2) * az - T(ONE_G);
            x_.ax() = ga_x;
            x_.ay() = ga_y;
            x_.az() = ga_z;
            ax_out = ga_x;
            ay_out = ga_y;
            az_out = ga_z;
            
            T vx_new = x.vx() + x.ax() * u.dt();
            T vy_new = x.vy() + x.ay() * u.dt();
            T vz_new = x.vz() + x.az() * u.dt();

            T wx_new = u.wx();
            T wy_new = u.wy();
            T wz_new = u.wz();

            wx_out = wx_new;
            wy_out = wy_new;
            wz_out = wz_new;

            Eigen::Matrix3d R_new;
            Eigen::Matrix3d Omega_cha;
            Omega_cha(0,0) = 1;
            Omega_cha(1,1) = 1;
            Omega_cha(2,2) = 1;
            Omega_cha(0,1) = -wz_new * u.dt();
            Omega_cha(0,2) = wy_new * u.dt();
            Omega_cha(1,0) = wz_new * u.dt();
            Omega_cha(1,2) = -wx_new * u.dt();
            Omega_cha(2,0) = -wy_new * u.dt();
            Omega_cha(2,1) = wx_new * u.dt();
            
            R_new = R_ * Omega_cha;

            Kalman::Vector<T, 3> axis_temp;
            axis_temp(0) = std::sqrt(R_new(0,0)*R_new(0,0) + R_new(1,0)*R_new(1,0) + R_new(2,0)*R_new(2,0));
            axis_temp(1) = std::sqrt(R_new(0,1)*R_new(0,1) + R_new(1,1)*R_new(1,1) + R_new(2,1)*R_new(2,1));
            axis_temp(2) = std::sqrt(R_new(0,2)*R_new(0,2) + R_new(1,2)*R_new(1,2) + R_new(2,2)*R_new(2,2));

            for (int i = 0; i < 3 ; i++) {
                for (int j = 0; j < 3; j++) {
                    R_new(i,j) = R_new(i,j)/axis_temp(j);
                }
            }

            Eigen::Vector3d new_e;
            
            get_euler_from_R(new_e, R_new);
            new_e(2) = x.qz()/T(180)*T(M_PI) + u.wz()*u.dt();
           
            std::cout << "pos: " << x_new << " " << y_new << std::endl;
            x_.x() = x_new;
            x_.y() = y_new;
            x_.z() = z_new;
            x_.vx() = vx_new;
            x_.vy() = vy_new;
            x_.vz() = vz_new;
    
            x_.qx() = new_e(0)/T(M_PI)*T(180);//q_new(1);
            x_.qy() = new_e(1)/T(M_PI)*T(180);//q_new(1);
            x_.qz() = new_e(2)/T(M_PI)*T(180);//q_new(1);

            return x_;
        }
};

}

#endif