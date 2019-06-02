/* @author Aytaç Kahveci */
#ifndef KUKA_ROBOT_STATE_PUBLISHER_H
#define KUKA_ROBOT_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <math.h>

class KukaRobotStatePublisher
{

public:

    KukaRobotStatePublisher(ros::NodeHandle &nh);

    ~KukaRobotStatePublisher(){}

    void stateCB(const sensor_msgs::JointStateConstPtr& msg);

private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    tf::TransformBroadcaster tf_broadcaster_;
    std::string base_frame_ = "kuka_fixed_frame";
    std::string child_frame_ = "end_effector";
};

#endif
