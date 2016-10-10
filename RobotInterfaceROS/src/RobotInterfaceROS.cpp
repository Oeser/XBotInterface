#include <RobotInterfaceROS/RobotInterfaceROS.h>


XBot::RobotInterfaceROS::RobotInterfaceROS(const XBot::XBotCoreModel& XBotModel): 
    RobotInterface(XBotModel),
    _joint_state_received(false)
{

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
    

    bool joint_state_ok = parseJointStateIds();
    bool controller_ok = parseControllerIds();

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
        
        // TBD if(XBotModel.hasJoint(joint_name)
        _controller_ids[joint_name] = res.ids[cont++];
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
        
        // TBD if(XBotModel.has(joint_name)
        _joint_state_ids[joint_name] = cont++;
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
    
}


bool XBot::RobotInterfaceROS::move_internal()
{

}

