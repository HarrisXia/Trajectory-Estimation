#ifndef EKF_IMU_H
#define EKF_IMU_H

#include <map>
#include <set>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_math_type.h"
// namespace ekf_imu{}

class ekf_imu{
    public:
    ekf_imu();
    ~ekf_imu();
    private:
};  
    const double pi = 3.1415926535;
    bool gps_flag = false;
    double yaw = 0;

    double raw_;
    Vector4d orientation_;
    ros::Publisher q_pub;
    Vector4d q;//wxyz
    Quaterniond Q;//WXYZ
    Quaterniond Q_;//XYZW
    Vector4d q_;//xyzw
    double time_old;
    double time_new; 
    bool imu_init = false;
    std::vector<sensor_msgs::Imu> imu_msg_buffer;
    Eigen::Vector3d gyro_bias;
    Vector3d gravity_imu;
    // Vector3d gravity;

    Vector4d orientation;
    static Eigen::Vector3d gravity;
    // void initializeGravityAndBias();

    void initGravity();
    void processModel(const double& time, const Vector3d& m_gyro, const Vector3d& m_acc);



inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& w) {
  Eigen::Matrix3d w_hat;
  w_hat(0, 0) = 0;
  w_hat(0, 1) = -w(2);
  w_hat(0, 2) = w(1);
  w_hat(1, 0) = w(2);
  w_hat(1, 1) = 0;
  w_hat(1, 2) = -w(0);
  w_hat(2, 0) = -w(1);
  w_hat(2, 1) = w(0);
  w_hat(2, 2) = 0;
  return w_hat;
}

inline Eigen::Matrix3d quaternionToRotation(
    const Eigen::Vector4d& q) {
  const Eigen::Vector3d& q_vec = q.block(0, 0, 3, 1);
  const double& q4 = q(3);
  Eigen::Matrix3d R =
    (2*q4*q4-1)*Eigen::Matrix3d::Identity() -
    2*q4*skewSymmetric(q_vec) +
    2*q_vec*q_vec.transpose();
  //TODO: Is it necessary to use the approximation equation
  //    (Equation (87)) when the rotation angle is small?
  return R;
}

inline void quaternionNormalize(Eigen::Vector4d& q) {
  double norm = q.norm();
  q = q / norm;
  return;
}

inline void vector3dtoQ(Eigen::Vector4d& q, Eigen::Quaterniond& Q) {
    Q.w() = q(0);
    Q.x() = q(1);
    Q.y() = q(2);
    Q.z() = q(3);
  return;
}

inline void Q_tovector3d(Eigen::Quaterniond& Q, Eigen::Vector4d& q) {
    q(0) = Q.x();
    q(1) = Q.y();
    q(2) = Q.z();
    q(3) = Q.w();
  return;
}

inline Eigen::Vector4d rotationToQuaternion(
    const Eigen::Matrix3d& R) {
  Eigen::Vector4d score;
  score(0) = R(0, 0);
  score(1) = R(1, 1);
  score(2) = R(2, 2);
  score(3) = R.trace();

  int max_row = 0, max_col = 0;
  score.maxCoeff(&max_row, &max_col);
  std::cout<<max_row<<std::endl;
  // max_row = 0;
  Eigen::Vector4d q = Eigen::Vector4d::Zero();
  if (max_row == 0) {
    q(0) = std::sqrt(1+2*R(0, 0)-R.trace()) / 2.0;
    q(1) = (R(0, 1)+R(1, 0)) / (4*q(0));
    q(2) = (R(0, 2)+R(2, 0)) / (4*q(0));
    q(3) = (R(1, 2)-R(2, 1)) / (4*q(0));
  } else if (max_row == 1) {
    q(1) = std::sqrt(1+2*R(1, 1)-R.trace()) / 2.0;
    q(0) = (R(0, 1)+R(1, 0)) / (4*q(1));
    q(2) = (R(1, 2)+R(2, 1)) / (4*q(1));
    q(3) = (R(2, 0)-R(0, 2)) / (4*q(1));
  } else if (max_row == 2) {
    q(2) = std::sqrt(1+2*R(2, 2)-R.trace()) / 2.0;
    q(0) = (R(0, 2)+R(2, 0)) / (4*q(2));
    q(1) = (R(1, 2)+R(2, 1)) / (4*q(2));
    q(3) = (R(0, 1)-R(1, 0)) / (4*q(2));
  } else {
    q(3) = std::sqrt(1+R.trace()) / 2.0;
    q(0) = (R(1, 2)-R(2, 1)) / (4*q(3));
    q(1) = (R(2, 0)-R(0, 2)) / (4*q(3));
    q(2) = (R(0, 1)-R(1, 0)) / (4*q(3));
    // q(0) = std::sqrt(1+R.trace()) / 2.0;
    // q(1) = (R(1, 2)-R(2, 1)) / (4*q(0));
    // q(2) = (R(2, 0)-R(0, 2)) / (4*q(0));
    // q(3) = (R(0, 1)-R(1, 0)) / (4*q(0));
  }

  if (q(3) < 0) q = -q;
  // if (q(0) < 0) q = -q;
  quaternionNormalize(q);
  return q;
}

inline Eigen::Quaterniond eulartoq(Eigen::Vector3d& eular){
  double yaw = eular(0);
  double pitch = eular(1);
  double roll = eular(2);
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

}

#endif