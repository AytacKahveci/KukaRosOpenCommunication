/* @author Aytaç Kahveci */
#ifndef KUKA_CLASS_H_
#define KUKA_CLASS_H_

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

namespace kuka_class
{
    class kuka
    {
    public:

        kuka()
        {

        }

        kuka(ros::NodeHandle& n, const int sz)
        {
            position.resize(sz);
            sub = n.subscribe("/joint_states",1,&kuka::subCb,this);
        }

        ~kuka()
        {
            sub.shutdown();
        }

        void subCb(const sensor_msgs::JointState &pos)
        {
            position = pos.position;
        }

    std::vector<double> position;
    private:
        ros::Subscriber sub;
        ros::Publisher pub;
    };
}
#endif
