#ifndef GPS_HPP_
#define GPS_HPP_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include "geometry_math_type.h"
double constrain(double value, double min, double max) {
    if (value < min ) {
        value = min;
    } else if (value > max) {
        value = max;
    }
    return value;
}

double constrain_yaw(double yaw) {
	while (yaw > 180) {
		yaw -= 360;
	}

	while (yaw < -180) {
		yaw += 360;
	}

	return yaw;
}

void WGS_to_xyz(const Eigen::Vector3d &wgs, const Eigen::Vector3d &Center, Eigen::Vector3d &xyz_imu, const Eigen::Vector3d &Center_yaw, const float& _delta_yaw) {
	Eigen::Vector3d xyz;
    // double a = 6371000; //meters (m)
    double a = 6378137.0; //meters (m)
    double lat_rad = wgs[0] / 180 * M_PI;
    double lon_rad = wgs[1] / 180 * M_PI;
    double Center_lat_rad = Center[0]  /180 * M_PI;
    double Center_lon_rad = Center[1]  /180 * M_PI;

	double sin_lat = sin(lat_rad);
	double cos_lat = cos(lat_rad);

	double cos_d_lon = cos(lon_rad - Center_lon_rad);

	double arg = constrain(sin(Center_lat_rad) * sin_lat + cos(Center_lat_rad) * cos_lat * cos_d_lon, -1.0,  1.0);
	double c = acos(arg);

	double k = 1.0;

	if (fabs(c) > 0) {
		k = (c / sin(c));
	}

	// NED
	xyz[0] = k * (cos(Center_lat_rad) * sin_lat - sin(Center_lat_rad) * cos_lat * cos_d_lon) * a;
	xyz[1] = k * cos_lat * sin(lon_rad - Center_lon_rad) * a;
	xyz[2] = 0;

	Eigen::Vector3d start_to_ned_euler;
	start_to_ned_euler[2] = (Center_yaw[2] -_delta_yaw) / 180 * M_PI;
	Eigen::Matrix3d start_to_ned_R;
	get_dcm_from_euler(start_to_ned_R, start_to_ned_euler);
	xyz_imu = start_to_ned_R.transpose() * xyz;

}

void xyz_to_WGS(const Eigen::Vector3d &xyz_imu, const Eigen::Vector3d &Center, Eigen::Vector3d &wgs, const Eigen::Vector3d &Center_yaw, const float& _delta_yaw) {
	Eigen::Vector3d start_to_ned_euler;
	start_to_ned_euler[2] = (Center_yaw[2] -_delta_yaw) / 180 * M_PI;
	Eigen::Matrix3d start_to_ned_R;
	get_dcm_from_euler(start_to_ned_R, start_to_ned_euler);
	Eigen::Vector3d xyz;
	xyz = start_to_ned_R * xyz_imu;
	// NED
    double a = 6378137; //meters (m)
    double x_rad = xyz[0] / a;
    double y_rad = xyz[1] / a;
    double c = sqrt(x_rad * x_rad + y_rad * y_rad);
    double Center_lat_rad = Center[0]  /180 * M_PI;
    double Center_lon_rad = Center[1]  /180 * M_PI;


    if (fabs(c) > 0) {
		double sin_c = sin(c);
		double cos_c = cos(c);

		double lat_rad = asin(cos_c * sin(Center_lat_rad) + (x_rad * sin_c * cos(Center_lat_rad)) / c);
		double lon_rad = (Center_lon_rad + atan2(y_rad * sin_c, c * cos(Center_lat_rad) * cos_c - x_rad * sin(Center_lat_rad) * sin_c));

		wgs[0] = lat_rad / M_PI * 180;
		wgs[1] = lon_rad / M_PI * 180;
		wgs[2] = 0;

	} else {
		wgs[0] = Center_lat_rad / M_PI * 180;
		wgs[1] = Center_lon_rad / M_PI * 180;
		wgs[2] = 0;
	}
}

#endif