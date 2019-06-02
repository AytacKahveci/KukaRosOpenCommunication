/* @author Aytaç Kahveci */
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "SquareTrajectory");
    ros::NodeHandle nh;
    ros::Publisher pub_ = nh.advertise<std_msgs::Float64MultiArray>("/joint_position_controller/command",1);
    ros::Publisher pubVel_ = nh.advertise<std_msgs::Float64>("/TCP_velocity_command/command",1);

    std_msgs::Float64MultiArray cartPos_;
    std_msgs::Float64 vel_;

    ros::Rate loop_rate(0.16);

    cartPos_.data.resize(6);

    double wayPoints[10][6] = {{50,0,0,0,0,0},{50,50,0,0,0,0},{0,50,0,0,0,0},{0,0,0,0,0,0} \
                            ,{0,0,50,0,0,0},{50,0,50,0,0,0},{50,50,50,0,0,0},{0,50,50,0,0,0}\
                            ,{0,0,50,0,0,0},{0,0,0,0,0,0}};
    double velData[4] = {0.1, 0.1, 0.1, 0.1};

    int i=0;
    char c;

    std::cout << "Press any key to start" << std::endl;
    std::cin >> c;

    while(ros::ok())
    {
        for(int j=0; j<6; j++)
        {
            cartPos_.data[j] = wayPoints[i][j];
            std::cout << "data_" << j << ": " << wayPoints[i][j] << std::endl;
        }
        i++;
        if(i <= 5)
        {
            vel_.data = velData[0];
        }
        else if(i>5 && i < 9)
        {
            vel_.data = velData[1];
        }
        else if(i > 9)
        {
            i = 0;
            vel_.data = 0.1;
        }
        pubVel_.publish(vel_);
        pub_.publish(cartPos_);

        loop_rate.sleep();
    }
    return 0;
}
