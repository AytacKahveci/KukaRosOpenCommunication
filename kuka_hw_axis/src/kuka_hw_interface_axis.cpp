/* @author Aytaç Kahveci */
#include <kuka_hw_axis/kuka_hardware_interface.h>
#include <math.h>
#define PI 3.14159
#include <stdexcept>

namespace kuka_hw_interface
{
    kukaHardwareInterface::kukaHardwareInterface()
    {
        pos_ = new double[6]{0, 0, 0, 0, 0, 0};
        vel_ = new double[6]{0, 0, 0, 0, 0, 0};
        eff_ = new double[6]{0, 0, 0, 0, 0, 0};
        cmd_pos_ = new double[6]{0, 0, 0, 0, 0, 0};
        last_cmd_pos_ = new double[6]{0, 0, 0, 0, 0, 0};
        cmd_vel_ = new double[6]{0, 0, 0, 0, 0, 0};
        cmd_eff_ = new double[6]{0, 0, 0, 0, 0, 0};

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
        if(!nh_.getParam("controller_joint_names", joint_names_))
        {
            ROS_ERROR("Couldn't find required parameter 'controller_joint_names' on the parameter server.");
            throw std::runtime_error("Couldn't find required parameter 'controller_joint_names' on the parameter server.");
        }

        for(size_t i=0; i<n_dof_; ++i)
        {
            joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &pos_[i], &vel_[i], &eff_[i]));

            position_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]), &cmd_pos_[i]));
        }

        ROS_INFO_STREAM_NAMED("hardware_interface", "Loaded kuka_hardware_interface");
    }

    kukaHardwareInterface::~kukaHardwareInterface()
    {
        delete [] pos_;
        delete [] vel_;
        delete [] eff_;
        delete [] cmd_pos_;
        delete [] cmd_vel_;
        delete [] cmd_eff_;
        delete axisAct;
        delete myAxis;
    }

      void kukaHardwareInterface::read()
      {
          client_->readVariable<KRLE6Axis>(axisAct);
          for(size_t i=0; i<n_dof_; i++)
          {
              pos_[i] = axisAct->asArrayA1ToA6()[i]*PI/180;
          }
      }

     void kukaHardwareInterface::write()
      {
          bool changes_pos=false;
      	for(std::size_t i=0; i<n_dof_; i++)
      	{
              if(last_cmd_pos_[i]!=cmd_pos_[i])
      		{
      			last_cmd_pos_[i]= cmd_pos_[i];
      			changes_pos = true;
      		}
      	}
              if(changes_pos)
          	{
                myAxis->setA1ToA6({cmd_pos_[0]*180/PI,cmd_pos_[1]*180/PI,cmd_pos_[2]*180/PI,cmd_pos_[3]*180/PI,cmd_pos_[4]*180/PI,cmd_pos_[5]*180/PI});
                client_->writeVariable<KRLAxis>(myAxis);
          		ROS_INFO_STREAM("POSSENDED "<< 0<<": "<< cmd_pos_[0]);
          		ROS_INFO("SEND POS!");
          	}
      }

    void kukaHardwareInterface::start()
    {
        std::string host_;
        if(!nh_.getParam("Robot_IP",host_))
        {
            ROS_ERROR_STREAM("Couldn't find required parameter 'Robot_IP' on the parameter server");
            throw std::runtime_error("Couldn't find required parameter 'Robot_IP' on the parameter server");
        }
        hostName_ = new char[host_.length() + 1];
        strcpy(hostName_,host_.c_str());
        if(!nh_.getParam("Robot_Port",port_))
        {
            ROS_ERROR_STREAM("Couldn't find required parameter 'Robot_Port' on the parameter server");
            throw std::runtime_error("Couldn't find required parameter 'Robot_Port' on the parameter server");
        }
        client_ = new kukaClient(hostName_,port_);
        ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Got connection from robot");

        axisAct = new KRLE6Axis("$AXIS_ACT",{"A1", "A2", "A3", "A4", "A5", "A6", "E1", "E2", "E3", "E4", "E5", "E6"});
        myAxis = new KRLAxis("MYAXIS");
    }
}
