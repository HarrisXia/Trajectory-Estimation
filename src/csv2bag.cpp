#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
// #include <sensor_msgs/Float.h>
using namespace std;

class csvdata1{
public:
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
};//行的类定义
class csvdata2{
public:
    float latitude;
    float longitude;
    float a;
    float b;
    float c;
    float d;
};//行的类定义
class csvdata3{
    public:
    float longitude;
    float latitude;
    float hight;
};
vector<csvdata1> incsv1;
vector<csvdata2> incsv2;
vector<csvdata3> incsv3;
ros::Publisher imu_pub;
ros::Publisher gps_pub;
ros::Publisher true_pub;
ros::Time timestamp;

void csv2bag(){
    csvdata1 intp1;
    FILE *fp1;
    // fp = fopen("/home/work/csv2bag/src/csv2bag/src/IMU.csv","r");
    fp1=fopen("IMU.csv","r");//你自己的文件路径
    if(fp1 == NULL)
    cout<<"0"<<endl;
    else
    while(1){
        // fscanf(fp,"%lf,%lf,%lf,%lf,%lf,%lf",&intp.ax,&intp.ay,&intp.az,&intp.gx,&intp.gy,&intp.gz);
        fscanf(fp1,"%f,%f,%f,%f,%f,%f",&intp1.ax,&intp1.ay,&intp1.az,&intp1.gx,&intp1.gy,&intp1.gz);
        // fscanf(fp,"%f",&intp.ax);
        incsv1.push_back(intp1);
        if (feof(fp1))break;
    }
    fclose(fp1);

    csvdata2 intp2;
    FILE *fp2;
    fp2=fopen("GPS.csv","r");
    if(fp2 == NULL)
    cout<<"0"<<endl;
    else
    while(1){
        fscanf(fp2,"%f,%f,%f,%f,%f,%f",&intp2.longitude,&intp2.latitude,&intp2.a,&intp2.b,&intp2.c,&intp2.d);
        incsv2.push_back(intp2);
        if (feof(fp2))break;
    }
    fclose(fp2);

    csvdata3 intp3;
    FILE *fp3;
    fp3=fopen("GroundTruth.csv","r");
    if(fp3 == NULL)
    cout<<"0"<<endl;
    else
    while(1){
        fscanf(fp3,"%f,%f,%f",&intp3.longitude,&intp3.latitude,&intp3.hight);
        incsv3.push_back(intp3);
        if (feof(fp3))break;
    }
    fclose(fp3);
    cout << "read_over" <<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "csv2bag");
    ros::NodeHandle n("~");
    imu_pub = n.advertise<sensor_msgs::Imu>("/IMU",1000);
    gps_pub = n.advertise<sensor_msgs::NavSatFix>("/GPS",1000);
    true_pub = n.advertise<sensor_msgs::NavSatFix>("/true",1000);
    csv2bag();
    cout << "msg_act" << endl;
    timestamp.sec = 0;
    timestamp.nsec = 0;
    for(int i=0;i<incsv1.size();i++){
        sensor_msgs::Imu imu_msg;
        sensor_msgs::NavSatFix gps_msg;
        sensor_msgs::NavSatFix truth_msg;
        // timestamp = ros::Time::now();
        imu_msg.linear_acceleration.x = incsv1[i].ax;
        imu_msg.linear_acceleration.y = incsv1[i].ay;
        imu_msg.linear_acceleration.z = incsv1[i].az;
        imu_msg.angular_velocity.x = incsv1[i].gx;
        imu_msg.angular_velocity.y = incsv1[i].gy;
        imu_msg.angular_velocity.z = incsv1[i].gz;
        imu_msg.header.stamp = timestamp;
        gps_msg.latitude = incsv2[i].latitude;
        gps_msg.longitude = incsv2[i].longitude;
        gps_msg.header.stamp = timestamp;
        truth_msg.latitude = incsv3[i].latitude;
        truth_msg.longitude = incsv3[i].longitude;
        truth_msg.header.stamp = timestamp;
        imu_pub.publish(imu_msg);
        gps_pub.publish(gps_msg);
        true_pub.publish(truth_msg);
        timestamp.sec ++;
        cout << "i= " << i <<" size="<<incsv1.size() << endl;
        if(ros::ok()){}
        else break;
        // delay_msec(200);
        usleep(10000);
        // sleep(1);
    }
    cout << "msg_end" << endl;
    return 0;
}
