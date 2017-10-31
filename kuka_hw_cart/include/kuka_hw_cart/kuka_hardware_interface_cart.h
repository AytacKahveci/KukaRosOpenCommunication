/* @author Aytaç Kahveci */
#ifndef KUKA_HARDWARE_INTERFACE_H
#define KUKA_HARDWARE_INTERFACE_H

#include <vector>
#include <string>

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

        double *pos_;
        double *vel_;
        double *eff_;
        double *cmd_pos_;
        double *last_cmd_pos_;
        double *cmd_vel_;
        double *cmd_eff_;

        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;

        ros::Duration control_period_;
        double loop_hz_;

        kukaClient* client_;

        char *hostName_;
        int port_;
    public:
        kukaHardwareInterface();

        ~kukaHardwareInterface();

        void read();

        void write();

        void start();

        KRLPos *posAct;
        KRLPos *myPos;
    };
} // namespace kuka_hw_interface
#endif
