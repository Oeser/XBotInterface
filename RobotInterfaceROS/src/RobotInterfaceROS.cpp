#include <RobotInterfaceROS/RobotInterfaceROS.h>


XBot::RobotInterfaceROS::RobotInterfaceROS(const XBot::XBotCoreModel& XBotModel): 
    RobotInterface(XBotModel),
    _joint_state_received(false)
{
	for( const std::string& joint_name : this->getEnabledJointNames() ){
			_control_mode_map[joint_name] = control_mode_IDLE;
	}
}

bool XBot::RobotInterfaceROS::init(const std::string& path_to_cfg)
{
    std::ifstream fin ( path_to_cfg );
    if ( fin.fail() ) {
        printf ( "Can not open %s\n", path_to_cfg.c_str() ); //TBD change it
        return false;
    }

    YAML::Node root_cfg = YAML::LoadFile ( path_to_cfg ); 
    // TBD check if they exist
    _joint_state_name = root_cfg["joint_state_name"].as<std::string>();
    _joint_service_name = root_cfg["joint_service_name"].as<std::string>();
    _command_topic_name = root_cfg["command_topic_name"].as<std::string>();

    bool joint_state_ok = parseJointStateIds();
    bool controller_ok = parseControllerIds();
	
	_control_command_pub = _nh.advertise<custom_messages::CustomCmnd>(_command_topic_name, 1);

    return joint_state_ok && controller_ok;
}

bool XBot::RobotInterfaceROS::parseControllerIds()
{
    ros::ServiceClient client = _nh.serviceClient<custom_services::getJointNames>(_joint_service_name);
    custom_services::getJointNamesRequest req;
    custom_services::getJointNamesResponse res;
    if(!client.call(req, res)) {
        std::cerr << "ERROR service client " << _joint_service_name << " not available! Write() won't work!!" << std::endl;
        return false;
    }
    // res is valid: populate map
    int cont = 0;
    for(const std::string& joint_name: res.joints) {
        
		if(this->hasJoint(joint_name)){
			_controller_ids[joint_name] = res.ids[cont++];
		}
		else{ // TBD: what to do if controller response contains a non-existing (disabled) joint? Currently, nothing
			std::cerr << "ERROR joint " << joint_name << " does not exist (or is disabled)!" << std::endl;
		}
    }
    return true;
}


bool XBot::RobotInterfaceROS::parseJointStateIds()
{
    int max_connect_attempts = 100;
    double sleep_duration = 0.01;
    int attempt = 0;
    
    _joint_state_sub = _nh.subscribe(_joint_state_name, 1, &RobotInterfaceROS::jointStateCallback, this);

    while(!_joint_state_received && attempt < max_connect_attempts){
        ros::Duration(sleep_duration).sleep();
        attempt++;
    }
    if(!_joint_state_received) {
        std::cerr << "ERROR No message received on topic " << _joint_state_name << "! Read() won't work!!" << std::endl;
        return false;
    }
    // joint state msg is valid: populate map
    int cont = 0;
    for(const std::string& joint_name: _joint_state_msg.name) {
        
       if(this->hasJoint(joint_name)){
        _joint_state_ids[joint_name] = cont++;
		_jointstate_msg_map_position[joint_name] = 0;
		_jointstate_msg_map_velocity[joint_name] = 0;
		_jointstate_msg_map_effort[joint_name] = 0;
	   }
	   else{ // TBD: what to do if joint state msg contains a non-existing (disabled) joint? Currently, nothing
		   std::cerr << "ERROR joint " << joint_name << " does not exist (or is disabled)!" << std::endl;
	   }
    }
    return true;
}


void XBot::RobotInterfaceROS::jointStateCallback(sensor_msgs::JointState::ConstPtr joint_state_msg)
{
    _joint_state_msg = *joint_state_msg;
    _joint_state_received = true;
}

bool XBot::RobotInterfaceROS::sense_internal()
{
    ros::spinOnce();
	// NOTE: should we check if we actually received any message?
	// Right now NO (chains are updated even if msg is equal to last sense)
	for(int i=0; i<_joint_state_msg.name.size(); i++){
		
		_jointstate_msg_map_position[_joint_state_msg.name[i]] = _joint_state_msg.position[i];
		_jointstate_msg_map_velocity[_joint_state_msg.name[i]] = _joint_state_msg.velocity[i];
		_jointstate_msg_map_effort[_joint_state_msg.name[i]] = _joint_state_msg.effort[i];
	}
	
	this->setLinkPos(_jointstate_msg_map_position);
	this->setLinkVel(_jointstate_msg_map_velocity);
	this->setEffort(_jointstate_msg_map_effort);
	
	_joint_state_received = false;
    
}


bool XBot::RobotInterfaceROS::move_internal()
{
	// Get references from chains
	this->getPosRef(_command_msg_map_position);
	this->getVelRef(_command_msg_map_velocity);
	this->getEffortRef(_command_msg_map_effort);
	this->getStiffness(_command_msg_map_stiffness);
	this->getDamping(_command_msg_map_damping);
	
	// Make sure we allocated enough memory (assumes consecutive indices) TBD support non-consecutive idx??
	_controller_msg.position.reserve(_controller_ids.size());
	_controller_msg.velocity.reserve(_controller_ids.size());
	_controller_msg.effort.reserve(_controller_ids.size());
	_controller_msg.onlineGain1.reserve(_controller_ids.size());
	_controller_msg.onlineGain2.reserve(_controller_ids.size());
	
	// Fill the command message
	for( const auto& jointname_id_pair : _controller_ids ) {
	
		const std::string& joint_name = jointname_id_pair.first;
		int id = jointname_id_pair.second;
		
		_controller_msg.position[id] = _command_msg_map_position.at(joint_name);
		_controller_msg.velocity[id] = _command_msg_map_velocity.at(joint_name);
		_controller_msg.effort[id] = _command_msg_map_effort.at(joint_name);
		_controller_msg.onlineGain1[id] = _command_msg_map_stiffness.at(joint_name);
		_controller_msg.onlineGain2[id] = _command_msg_map_damping.at(joint_name);
		
	}
	
	// Send the command message
	_control_command_pub.publish(_controller_msg);
	

}

bool XBot::RobotInterfaceROS::getControlMode(std::map< std::string, std::string >& joint_control_mode_map)
{
	joint_control_mode_map = _control_mode_map;
	return true;
}

bool XBot::RobotInterfaceROS::setControlMode(const std::string& robot_control_mode)
{
	if(!checkControlMode(robot_control_mode)){
		std::cerr << "Requested control mode " << robot_control_mode << " does not exist in this controller!" << std::endl;
		return false;
	}
	
	for( auto& jointname_ctrlmode_pair : _control_mode_map ){
			jointname_ctrlmode_pair.second = robot_control_mode;
	}
}

bool XBot::RobotInterfaceROS::setControlMode(const std::map< std::string, std::string >& joint_control_mode_map)
{
	// Check for input consistency (joints exist and control modes are defined)
	for( const auto& jointname_ctrlmode_pair : joint_control_mode_map ){

		if(!this->hasJoint(jointname_ctrlmode_pair.first)){
			std::cerr << "Joint " << jointname_ctrlmode_pair.first << " does not exist (or it's diabled)!" << std::endl;
			return false;
		}
		if(!checkControlMode(jointname_ctrlmode_pair.second)){
			std::cerr << "Requested control mode " << jointname_ctrlmode_pair.second << " does not exist in this controller!" << std::endl;
			return false;
		}
		
	}
	
	for( const auto& jointname_ctrlmode_pair : joint_control_mode_map ){
			_control_mode_map.at(jointname_ctrlmode_pair.first) = jointname_ctrlmode_pair.second;
	}
	
	return true;
	
}

bool XBot::RobotInterfaceROS::checkControlMode(const std::string& control_mode) const
{
	return control_mode == control_mode_IDLE || 
				control_mode == control_mode_POS ||
				control_mode == control_mode_TORQUE ||
				control_mode == control_mode_POS_FFTORQUE;
}

