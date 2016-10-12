#ifndef XBOT_ROBOTINTERFACE_ROS_H
#define XBOT_ROBOTINTERFACE_ROS_H

#include <XBotInterface/RobotInterface.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "custom_messages/CustomCmnd.h"
#include "custom_services/getJointNames.h"

namespace XBot {

        class RobotInterfaceROS : public RobotInterface {
			
			friend RobotInterface;

        public:
			
			std::string control_mode_TORQUE;
			std::string control_mode_IDLE;
			std::string control_mode_POS;
			std::string control_mode_POS_FFTORQUE;
			

			virtual bool getControlMode(std::map< std::string, std::string >& joint_control_mode_map); 
			
			virtual bool setControlMode(const std::map< std::string, std::string >& joint_control_mode_map);
			virtual bool setControlMode(const std::string& robot_control_mode);
                
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
			ros::Publisher _control_command_pub;
            
            sensor_msgs::JointState _joint_state_msg;
			custom_messages::CustomCmnd _controller_msg;
			
            bool _joint_state_received;
            
            std::string _joint_state_name;
            std::string _joint_service_name;
			std::string _command_topic_name;
            
            std::map<std::string, int> _controller_ids;
            std::map<std::string, int> _joint_state_ids;
            
			std::map<std::string, double> _jointstate_msg_map_position;
			std::map<std::string, double> _jointstate_msg_map_velocity;
			std::map<std::string, double> _jointstate_msg_map_effort;
			
			std::map<std::string, double> _command_msg_map_position;
			std::map<std::string, double> _command_msg_map_velocity;
			std::map<std::string, double> _command_msg_map_effort;
			std::map<std::string, double> _command_msg_map_stiffness;
			std::map<std::string, double> _command_msg_map_damping;
			
			std::map<std::string, std::string> _control_mode_map;
			
			bool checkControlMode(const std::string& control_mode) const;
            
            
                
        };
		
// 		const std::string RobotInterfaceROS::control_mode_TORQUE = "TORQUE";
// 		const std::string RobotInterfaceROS::control_mode_IDLE = "IDLE";
// 		const std::string RobotInterfaceROS::control_mode_POS = "POSITION";
// 		const std::string RobotInterfaceROS::control_mode_POS_FFTORQUE = "POS_FFTORQUE";
}
#endif