/* @author Aytaç Kahveci */
#include <kuka_hw_cart_vel/kuka_hw_cart_vel.h>
#include <kuka_ros_open_comm/KRLPos.h>
#include <kuka_ros_open_comm/KRLE6Pos.h>

int main(int argc, char **argv)
{
    ROS_INFO_STREAM_NAMED("hardware_interface", "Starting hardware interface...");

    ros::init(argc, argv, "kuka_hardware_interface");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    kuka_hw_interface::kukaHardwareInterface robot;

    ros::Time previous = ros::Time::now();
    ros::Rate loop_rate(100);

    controller_manager::ControllerManager controller_manager(&robot, nh);

    robot.start();

    sleep(1);

    while(ros::ok())
    {
        ros::Duration period = ros::Time::now() - previous;
        previous = ros::Time::now();
        robot.read();
        controller_manager.update(previous, period);
        robot.write();
        loop_rate.sleep();

    }

    robot.stop();
    spinner.stop();
    ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");

    return 0;
}
