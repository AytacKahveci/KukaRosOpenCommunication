/* @author Aytaç Kahveci */
#include "ros/ros.h"
#include "kuka_hw_cart_vel/kuka_class.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include <iostream>
#include <std_msgs/Float64.h>
#define PI 3.1415

//<kuka_vars>
float A_ = 180; //unit degree
float B_ = 0; //unit degree
float C_ = 180; //unit degree
float movingRes = 0.1; //unit mm
float movingThresh = 1; //unit mm
float kukaVel = 0.5; //unit m/s
//</kuka_vars>


int main(int argc, char *argv[]) {
    ros::init(argc,argv,"KukaSinControl");
    ros::NodeHandle nh;
    kuka_class::kuka robot(nh, 6);

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_position_controller/command",1);
    ros::Publisher pubVel = nh.advertise<std_msgs::Float64>("/TCP_velocity_command/command",1);
    ros::Publisher debug = nh.advertise<std_msgs::Float64>("/debug/command",1);

    std_msgs::Float64MultiArray cartPos_;

    cartPos_.data.resize(6);
    std_msgs::Float64 vel_;
    std_msgs::Float64 deb;

    ros::Rate loop_rate(50);

    char a;
    std::cout << "Press any key.." << std::endl;
    std::cin >> a;
    int loop = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        cartPos_.data[0] = 0;
        cartPos_.data[1] = 0.1*sin(2*PI*8/50*loop);
        cartPos_.data[2] = 0;
        cartPos_.data[3] = A_;
        cartPos_.data[4] = B_;
        cartPos_.data[5] = C_;
        //vel_.data = kukaVel;
        deb.data = 1*sin(2*PI*8/50*loop);
        pub.publish(cartPos_);
        //pubVel.publish(vel_);
        debug.publish(deb);
        loop++;
        loop_rate.sleep();
    }
    return 0;
}
