#ifndef XBOT_ROBOTINTERFACE_ROS_H
#define XBOT_ROBOTINTERFACE_ROS_H

#include <XBotInterface/RobotInterface.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "custom_messages/CustomCmnd.h"
#include "custom_services/getJointNames.h"

namespace XBot {

        class RobotInterfaceROS : public RobotInterface {

        public:
                
        protected:
            
            virtual bool init(const std::string& path_to_cfg);
            virtual bool move_internal();
            virtual bool sense_internal();
                
        private:
            
            explicit RobotInterfaceROS(const XBotCoreModel& XBotModel);
            
            void jointStateCallback(sensor_msgs::JointState::ConstPtr joint_state_msg);
            
            bool parseControllerIds();
            bool parseJointStateIds();
            
            ros::NodeHandle _nh;
            ros::Subscriber _joint_state_sub;
            
            sensor_msgs::JointState _joint_state_msg;
            bool _joint_state_received;
            
            std::string _joint_state_name;
            std::string _joint_service_name;
            
            std::map<std::string, int> _controller_ids;
            std::map<std::string, int> _joint_state_ids;
            
            
            
                
        };
}
#endif