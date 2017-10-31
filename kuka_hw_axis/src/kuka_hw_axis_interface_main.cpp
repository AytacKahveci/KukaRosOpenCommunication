/* @author Aytaç Kahveci */
#include <kuka_hw_axis/kuka_hardware_interface.h>
#include <unistd.h>
int main(int argc, char **argv)
{
    ROS_INFO_STREAM_NAMED("hardware_axis_interface", "Starting hardware interface...");

    ros::init(argc, argv, "kuka_hardware_axis_interface");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    kuka_hw_interface::kukaHardwareInterface robot;

    ros::Time timestamp;
    timestamp = ros::Time::now();
    ros::Rate loop_rate(10);

    robot.start();

    controller_manager::ControllerManager controller_manager(&robot, nh);


    sleep(1);

    while(ros::ok())
    {
        ros::Duration period = ros::Time::now() - timestamp;
        robot.read();
        timestamp = ros::Time::now();
        controller_manager.update(timestamp, period);
        robot.write();
        //usleep(100);
        loop_rate.sleep();

    }

    spinner.stop();
    ROS_INFO_STREAM_NAMED("hardware_axis_interface", "Shutting down.");

    return 0;
}
