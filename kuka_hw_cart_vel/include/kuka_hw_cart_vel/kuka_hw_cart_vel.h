/* @author Aytaç Kahveci */
#ifndef KUKA_HARDWARE_INTERFACE_CART_VEL_H
#define KUKA_HARDWARE_INTERFACE_CART_VEL_H

#include <vector>
#include <string>
#include <mutex>
#include <iostream>

//ROS
#include <ros/ros.h>

//ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_manager/controller_manager.h>
#include <kuka_ros_open_comm/KRLPos.h>
#include <kuka_ros_open_comm/KRLE6Pos.h>
#include <kuka_ros_open_comm/KRLReal.h>
//Timers
#include <chrono>

//KUKA CrossCommClient
#include <kuka_ros_open_comm/kuka_client.h>

namespace kuka_hw_interface
{
    class kukaHardwareInterface : public hardware_interface::RobotHW
    {
    private:
        ros::NodeHandle nh_;

        unsigned int n_dof_ = 6;

        std::vector<std::string> joint_names_;
        std::vector<std::string> vel_joint_names_;
        std::vector<double> starting_coordinates_;

        double *pos_;
        double *vel_;
        double *eff_;
        double *cmd_pos_;
        double *last_cmd_pos_;
        double *last_cmd_vel_;
        double *cmd_vel_;
        double *cmd_eff_;

        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        ros::Duration control_period_;
        double loop_hz_;

        kukaClient* client_;

        char *hostName_;
        int port_;

        std::mutex mutex_;
    public:
        kukaHardwareInterface();

        ~kukaHardwareInterface();

        void read();

        void write();

        void start();

        void stop();

        KRLPos *posAct;
        KRLPos *myPos;
        KRLReal *myVel;
    };
} // namespace kuka_hw_interface
#endif
