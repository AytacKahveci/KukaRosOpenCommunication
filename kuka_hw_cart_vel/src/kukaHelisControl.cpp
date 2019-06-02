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

    ros::Rate loop_rate(25);


    int loop = 0;
    double cond = 0.0;
    bool flag = false;
    double cAngle = 0.0;
    double h = 10;

    cartPos_.data[0] = 8;
    cartPos_.data[1] = 0;
    cartPos_.data[2] = 0;
    cartPos_.data[3] = 180;
    cartPos_.data[4] = 0;
    cartPos_.data[5] = 180 + atan(h / (2*PI*8)) * 180/PI;
    pub.publish(cartPos_);

    ros::spinOnce();

    char a;
    std::cout << "Press any key.." << std::endl;
    std::cin >> a;
    while(ros::ok())
    {
        ros::spinOnce();
        cartPos_.data[0] = 17*cos(2*PI*0.5/25*loop);
        cartPos_.data[1] = 17*sin(2*PI*0.5/25*loop);
        cAngle = atan(h / (2*PI*17)) * 180/PI;
        cond = atan2(cartPos_.data[1], cartPos_.data[0])* 180 /PI;
        if(cond <179 && !flag)
        {

            cartPos_.data[2] = (h * atan2(cartPos_.data[1], cartPos_.data[0])) / (2*PI);
            ROS_INFO("theta: %f", atan2(cartPos_.data[1], cartPos_.data[0]));
            ROS_INFO("z command : %f", cartPos_.data[2]);
            cartPos_.data[3] = A_ + atan2(cartPos_.data[1], cartPos_.data[0])* 180 /PI;
            //cartPos_.data[3] = A_;
            cartPos_.data[4] = B_;
            cartPos_.data[5] = C_ - cAngle;
            //vel_.data = kukaVel;
            deb.data = cartPos_.data[1];
            pub.publish(cartPos_);
            //pubVel.publish(vel_);
            debug.publish(deb);
        }
        else
        {
            //break;
            flag = true;
        }
            loop++;
        loop_rate.sleep();
    }
    return 0;
}
