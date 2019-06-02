/* @author Aytaç Kahveci */
#include <kuka_hw_cart_vel/kuka_hw_cart_vel.h>

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
        cmd_vel_ = new double[1]{0};
        last_cmd_vel_ = new double[1]{0};
        cmd_eff_ = new double[6]{0, 0, 0, 0, 0, 0};

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&velocity_joint_interface_);

        if(!nh_.getParam("controller_joint_names", joint_names_))
        {
            ROS_ERROR("Couldn't find required parameter 'controller_joint_names' on the parameter server.");
            throw std::runtime_error("Couldn't find required parameter 'controller_joint_names' on the parameter server.");
        }

        if(!nh_.getParam("starting_coordinates", starting_coordinates_))
        {
           ROS_ERROR("Couldn't find required parameter 'starting_coordinates' on the parameter server.");
           throw std::runtime_error("Couldn't find required parameter 'starting_coordinates' on the parameter server."); 
        }
        if(starting_coordinates_.size() != 6)
        {
            ROS_ERROR("starting_coordinates should exactly include 6 values: X,Y,Z,A,B,C your input includes:%zu values", 
                                                                                            starting_coordinates_.size());
            throw std::runtime_error("starting_coordinates should exactly include 6 values: X,Y,Z,A,B,C"); 
        }

        //+1 for velocity controlled variable
        for(size_t i=0; i<(n_dof_+1); ++i)
        {
            joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &pos_[i], &vel_[i], &eff_[i]));

            position_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]), &cmd_pos_[i]));
        }
        //register for velocity control variable
            velocity_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[n_dof_]), &cmd_vel_[0]));

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
        delete posAct;
        delete myPos;
    }

      void kukaHardwareInterface::read()
      {
          client_->readVariable<KRLPos>(posAct);
          for(size_t i=0; i<n_dof_; i++)
          {
              pos_[i] = posAct->asArrayXToC()[i];
          }
      }

     void kukaHardwareInterface::write()
      {
        bool changes_pos=false;
        bool changes_vel=false;
      	for(std::size_t i=0; i<n_dof_; i++)
      	{
            if(last_cmd_pos_[i]!=cmd_pos_[i])
      		{
      			last_cmd_pos_[i]= cmd_pos_[i];
      			changes_pos = true;
      		}
      	}
            if(last_cmd_vel_[0] != cmd_vel_[0])
            {
                last_cmd_vel_[0] = cmd_vel_[0];
                changes_vel = true;
            }

            if(changes_pos)
          	{
                myPos->setXToZ({cmd_pos_[0], cmd_pos_[1], cmd_pos_[2]});
                myPos->setAToC({cmd_pos_[3],cmd_pos_[4],cmd_pos_[5]});
                client_->writeVariable<KRLPos>(myPos);
          		ROS_INFO_STREAM("POSSENDED "<< 0<<": "<< cmd_pos_[0]);
          		ROS_INFO("SEND POS!");
          	}

            if(changes_vel)
            {
                myVel->setValue(cmd_vel_[0]);
                client_->writeVariable<KRLReal>(myVel);
                ROS_INFO_STREAM("VELSENDED "<< 0 <<":"<< cmd_vel_[0]);
                ROS_INFO("SEND VEL!");
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

        //Define variables that will be read from KUKAVARPROXY and write to KUKAVARPROXY
        posAct = new KRLPos("$POS_ACT");
        myPos = new KRLPos("MYPOS");
        myVel = new KRLReal("MYVEL");
    }

    void kukaHardwareInterface::stop()
    {
        mutex_.lock();

        myPos->setXToZ(std::vector<double>(starting_coordinates_.begin(), starting_coordinates_.begin()+3));
        myPos->setAToC(std::vector<double>(starting_coordinates_.begin()+3, starting_coordinates_.end()));
        client_->writeVariable<KRLPos>(myPos);
        std::cout << "POSSENDED " << 0 << ": " << cmd_pos_[0] << std::endl;
        std::cout << "SEND HOME POSITIONS!" << std::endl;

        mutex_.unlock();
    }
}
