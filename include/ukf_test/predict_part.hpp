#ifndef PREDICT_PART_HPP_
#define PREDICT_PART_HPP_

#include "ukf_test/update_part.hpp"
#include "sensor_msgs/Imu.h"



// #include <kalman/UnscentedKalmanFilter.hpp>

template<typename T, class State, class Control, class SystemModel,  class GpsMeasurement, class GpsMeasurementModel, class VioMeasurement, class VioModel>
class Filter_predict_part : public Filter_update_part<T, State, Control, SystemModel, GpsMeasurement, GpsMeasurementModel, VioMeasurement, VioModel>{
    public:
        Filter_predict_part(ros::NodeHandle & temp_nh):
        _first_predict(true),
        nh_predict_(temp_nh) {
            this->set_predict_valid(false);
            std::string imu_topic_name;
            int predict_rate;
            nh_predict_.param<std::string>("imu_topic", imu_topic_name, "/imu");
            nh_predict_.param<std::string>("frame_id", frameId, std::string("imu"));
            nh_predict_.param<int>("imu_rate", predict_rate, 100);
            imu_sub = nh_predict_.subscribe(imu_topic_name, 20, &Filter_predict_part::imu_update_cb, this);
            _delta_t_max = 1.0f/float(predict_rate);
            std::cout << "max dt = [" << _delta_t_max << "]" << std::endl;
        }

        ~Filter_predict_part() {

        }

        void imu_update_cb(const sensor_msgs::Imu& msg) {
            if (_first_predict) {
                _last_timestamp = msg.header.stamp;
                _first_predict = false;
            } else {
                double dt = (msg.header.stamp - _last_timestamp).toSec();
                if ( dt > _delta_t_max && dt < 10.0f) { // limit predict rate?
                    _last_timestamp = msg.header.stamp;
                    if (this->get_update_init_state()) {
                        Control _C;
                        _C.ax() = msg.linear_acceleration.x;
                        _C.ay() = msg.linear_acceleration.y;
                        _C.az() = msg.linear_acceleration.z;
                        _C.wx() = msg.angular_velocity.x;
                        _C.wy() = msg.angular_velocity.y;
                        _C.wz() = msg.angular_velocity.z;
                        _C.dt() = dt;
                        this->predict_process(_C, msg.header.stamp);
                        this->set_predict_valid(true);
                    }
                } else if (dt >= 10.0f) {
                    _first_predict = true;
                    ROS_INFO("time out !");
                    // this->reinit();
                }
            }
        }


    private:
        ros::NodeHandle nh_predict_;
        ros::Subscriber imu_sub;
        bool _first_predict;
        ros::Time _last_timestamp;
        std::string frameId;
        float _delta_t_max;
};

#endif