#ifndef UPDATA_PART_HPP_
#define UPDATA_PART_HPP_
#include "ros/ros.h"
// #include "ukf_test/SystemModel.hpp"
// #include "ukf_test/VioMeasurementModel.hpp"
#include "geometry_math_type.h"
#include "gps.hpp"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/NavSatFix.h"

#include <kalman/UnscentedKalmanFilter.hpp>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <pthread.h>

#ifdef LOG_FLAG
#include <string>
#include <iostream>
#include <fstream>
#endif

template<typename T, class State, class Control, class SystemModel, class GpsMeasurement, class GpsMeasurementModel, class VioMeasurement, class VioModel>
class Filter_update_part {
    public:
 
        Filter_update_part():
        nh_("~update_part") {
            _has_init = false;
            _predict_has_init = false;
            _gps_has_init = false;
            _update_has_ready = false;
            _loopClosure_done = false;
            gps_to_imu << 0, 0, 0;
            std::string gps_topic_name;
            std::string true_topic_name;
            std::string _postopic, _veltopic, _acctopic, _atttopic, wgs_topic;
            pthread_mutex_init(&_ukf_core_mutex, NULL);

            nh_.param<std::string>("gps_topic", gps_topic_name, "/GPS" );
            nh_.param<std::string>("true_topic", true_topic_name, "/true" );
            nh_.param<bool>("verbose", _verbose, false);
            nh_.param<std::string>("postopic", _postopic, "/vio_data_rigid1/pos");
            nh_.param<std::string>("veltopic", _veltopic, "/vio_data_rigid1/vel");
            nh_.param<std::string>("acctopic", _acctopic, "/vio_data_rigid1/acc");
            nh_.param<std::string>("atttopic", _atttopic, "/vio_data_rigid1/att");
            nh_.param<std::string>("wgstopic", wgs_topic, "/wgs");
            nh_.param<float>("delta_yaw", delta_yaw, 0.0f);
            
            Gps_measure_sub = nh_.subscribe(gps_topic_name, 1000, &Filter_update_part::gps_update_cb, this);
            true_measure_sub = nh_.subscribe(true_topic_name, 1000, &Filter_update_part::true_update_cb, this);
            
            _rigid_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>(_postopic, 2);
            _rigid_vel_pub = nh_.advertise<geometry_msgs::Vector3Stamped>(_veltopic, 2);
            _rigid_acc_pub = nh_.advertise<geometry_msgs::Vector3Stamped>(_acctopic, 2);
            _rigid_att_pub = nh_.advertise<geometry_msgs::PoseStamped>(_atttopic, 2);
            _WGS_84_pub = nh_.advertise<sensor_msgs::NavSatFix>(wgs_topic, 2);
            _gps_start_pub = nh_.advertise<nav_msgs::Path>("/gps_path", 10);
            _true_start_pub = nh_.advertise<nav_msgs::Path>("/true_path", 10);
            _state_start_pub = nh_.advertise<nav_msgs::Path>("/state_path", 10);
        }

        ~Filter_update_part() {
            delete _ukf_ptr;
        }
        
        void true_update_cb(const sensor_msgs::NavSatFix& msg) {
            if(_has_init){
                Eigen::Vector3d true_data;
                true_data[0] = msg.latitude;
                true_data[1] = msg.longitude;
                true_data[2] = 0; 
                Eigen::Vector3d gps_xyz;
                WGS_to_xyz(true_data, Center, gps_xyz, Center_yaw, delta_yaw);
                true_path.header.stamp = msg.header.stamp;
                true_path.header.frame_id = "camera_init";
                geometry_msgs::PoseStamped _true_pos;
                _true_pos.pose.position.x = gps_xyz[0];
                _true_pos.pose.position.y = gps_xyz[1];
                _true_pos.pose.position.z = gps_xyz[2];
                true_path.poses.push_back(_true_pos);
                _true_start_pub.publish(true_path);
            }
        }

        void gps_update_cb(const sensor_msgs::NavSatFix& msg) {
            if (!_has_init ) {
                State x;
                x.setZero();
                Eigen::Vector3d Start_wgs;
                Start_wgs[0] = msg.latitude;
                Start_wgs[1] = msg.longitude;
                Start_wgs[2] = 0;
                Center_yaw << 0, 0, 0;
                Eigen::Vector3d Start_to_Origin;
                Start_to_Origin << 0,0,0;
                xyz_to_WGS(Start_to_Origin, Start_wgs, Center, Center_yaw, delta_yaw);
                x.x() = 0;
                x.y() = 0;
                x.z() = 0;
                x.qx() = 0;
                x.qy() = 0;
                x.qz() = 0;
                init_process(x);
                _has_init = true;
            }
            if (!_has_init || !_predict_has_init) {
                return;
            } else {
                    Eigen::Vector3d gps_data;
                    gps_data[0] = msg.latitude;
                    gps_data[1] = msg.longitude;
                    gps_data[2] = msg.altitude;

                    Eigen::Vector3d gps_xyz;
                    WGS_to_xyz(gps_data, Center, gps_xyz, Center_yaw, delta_yaw);
                    std::cout << "gps_xyz:" << gps_xyz.transpose() << std::endl;

                    gps_path.header.stamp = msg.header.stamp;
                    gps_path.header.frame_id = "camera_init";
                    geometry_msgs::PoseStamped _gps_pos;
                    _gps_pos.pose.position.x = gps_xyz[0];
                    _gps_pos.pose.position.y = gps_xyz[1];
                    _gps_pos.pose.position.z = gps_xyz[2];
                    gps_path.poses.push_back(_gps_pos);
                    _gps_start_pub.publish(gps_path);

                    GpsMeasurement gps_state;
                    gps_state.gps_x() = gps_xyz[0] + gps_to_imu[0];
                    gps_state.gps_y() = gps_xyz[1] + gps_to_imu[1];
                    if (_update_has_ready) {
                            update_process_gps(gps_state, msg.header.stamp);
                        _update_has_ready = false;
                    }
            }
       }

        void reinit() {
            ROS_WARN("update reinit");
            _has_init = false;
            _predict_has_init = false;
            _gps_has_init = false;
        }

        void init_process(State& init_state) {
            pthread_mutex_lock(&_ukf_core_mutex);
            _ukf_ptr->init(init_state);
            pthread_mutex_unlock(&_ukf_core_mutex);
        }

        void update_process_gps(GpsMeasurement& gps_state, ros::Time _time_stamp) {
            pthread_mutex_lock(&_ukf_core_mutex);
            _x_ukf = _ukf_ptr->update(_gpsm, gps_state);
            pthread_mutex_unlock(&_ukf_core_mutex);
            if (_verbose) {
                std::cout << "x(gps): [" << gps_state << "]" << std::endl;
                std::cout << "x(ukf): [" << _x_ukf << "]" << std::endl;
            }
        }


        void predict_process(Control& imu_state, ros::Time _time_stamp) {
            pthread_mutex_lock(&_ukf_core_mutex);
            _x_ukf = _ukf_ptr->predict(_sys, imu_state);
            pthread_mutex_unlock(&_ukf_core_mutex);
            ros_publish_state(_x_ukf, _time_stamp); 
            _update_has_ready = true;
        }

        bool get_state( State & now_state) {
            now_state = _x_ukf;
            return _predict_has_init&_has_init;
        }

        bool get_update_init_state() {
            return _has_init;
        }

        void set_predict_valid(bool temp) {
            _predict_has_init = temp;
        }

        void ros_publish_state( State & now_state, ros::Time & _timestamp) {
            geometry_msgs::PoseStamped _pos_msg;
            geometry_msgs::Vector3Stamped _vel_msg;
            geometry_msgs::Vector3Stamped _acc_msg;
            geometry_msgs::PoseStamped _att_msg;
            sensor_msgs::NavSatFix WGS_84;
            geometry_msgs::PoseStamped _state_pos;
            
            state_path.header.stamp = _timestamp;
            state_path.header.frame_id = "camera_init";

            _state_pos.pose.position.x = now_state.x();
            _state_pos.pose.position.y = now_state.y();
            _state_pos.pose.position.z = now_state.z();
            state_path.poses.push_back(_state_pos);
            _state_start_pub.publish(state_path);

            _pos_msg.header.stamp = _timestamp;
            _pos_msg.pose.position.x = now_state.x();
            _pos_msg.pose.position.y = now_state.y();
            _pos_msg.pose.position.z = now_state.z();

            _vel_msg.header.stamp = _timestamp;
            _vel_msg.vector.x = now_state.vx();
            _vel_msg.vector.y = now_state.vy();
            _vel_msg.vector.z = now_state.vz();

            _acc_msg.header.stamp = _timestamp;
            _acc_msg.vector.x = now_state.ax();
            _acc_msg.vector.y = now_state.ay();
            _acc_msg.vector.z = now_state.az();

            Eigen::Vector3d _eular_att;
            _eular_att(0) = now_state.qx()*T(M_PI)/T(180);
            _eular_att(1) = now_state.qy()*T(M_PI)/T(180);
            _eular_att(2) = now_state.qz()*T(M_PI)/T(180);
            Eigen::Quaterniond _quat_att;
            get_q_from_euler(_quat_att, _eular_att);
            _att_msg.header.stamp = _timestamp;
            _att_msg.pose.orientation.w = _quat_att.w();
            _att_msg.pose.orientation.x = _quat_att.x();
            _att_msg.pose.orientation.y = _quat_att.y();
            _att_msg.pose.orientation.z = _quat_att.z();

            Eigen::Vector3d pos_xyz;
            pos_xyz[0] = now_state.x();
            pos_xyz[1] = now_state.y();
            pos_xyz[2] = now_state.z();
            T pos_yaw;
            pos_yaw = - now_state.qz() + Center_yaw[2]; 
            Eigen::Vector3d pos_XYZ, pos_wgs; 
            xyz_to_WGS(pos_xyz, Center, pos_wgs, Center_yaw, delta_yaw);
            WGS_84.header.stamp = _timestamp;
            WGS_84.latitude = pos_wgs[0];
            WGS_84.longitude = pos_wgs[1];
            WGS_84.position_covariance[0] = pos_yaw;

            _rigid_pos_pub.publish(_pos_msg);
            _rigid_vel_pub.publish(_vel_msg);
            _rigid_acc_pub.publish(_acc_msg);
            _rigid_att_pub.publish(_att_msg);
            _WGS_84_pub.publish(WGS_84);
        }

    private:
        ros::NodeHandle nh_;
        Kalman::UnscentedKalmanFilter<State> * _ukf_ptr;
        bool _has_init;
        bool _predict_has_init;
        bool _gps_has_init;
        bool _update_has_ready;
        bool _loopClosure_done;
        ros::Subscriber Gps_measure_sub;
        ros::Subscriber true_measure_sub;
        ros::Publisher _rigid_pos_pub;
        ros::Publisher _rigid_vel_pub;
        ros::Publisher _rigid_acc_pub;
        ros::Publisher _rigid_att_pub;
        ros::Publisher _WGS_84_pub;
        ros::Publisher _gps_start_pub;
        ros::Publisher _true_start_pub;
        ros::Publisher _state_start_pub;
        nav_msgs::Path gps_path;
        nav_msgs::Path true_path;
        nav_msgs::Path state_path;
        Eigen::Vector3d Start_in_earth;
        Eigen::Vector3d Center;
        Eigen::Vector3d Center_yaw;
        T delta_yaw;
        State _x_ukf;
        SystemModel _sys;
        VioModel _vm;
        GpsMeasurementModel _gpsm;
        bool _verbose;
        Eigen::Vector3d gps_to_imu;
        pthread_mutex_t _ukf_core_mutex;
};

#endif
