/* @author Aytaç Kahveci */
#include <kuka_hw_cart_vel/kuka_robot_state_publisher.h>

KukaRobotStatePublisher::KukaRobotStatePublisher(ros::NodeHandle &nh)
                                            : nh_(nh)
{
    sub_ = nh_.subscribe("/joint_states", 1, &KukaRobotStatePublisher::stateCB, this);
}

void KukaRobotStatePublisher::stateCB(const sensor_msgs::JointStateConstPtr& msg_)
{
    tf::Transform tf_transform;
    tf::Quaternion quaternian;

    tf_transform.setOrigin(tf::Vector3(msg_->position[3]*1e-3, msg_->position[4]*1e-3, msg_->position[5]*1e-3) );
    quaternian.setEulerZYX(msg_->position[0]*M_PI/180.0, msg_->position[1]*M_PI/180.0, msg_->position[2]*M_PI/180.0);
    tf_transform.setRotation(quaternian);
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), base_frame_, child_frame_));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kuka_robot_state_publisher");
    ros::NodeHandle nh;

    KukaRobotStatePublisher krs(nh);

    ros::spin();
    return 0;
}
