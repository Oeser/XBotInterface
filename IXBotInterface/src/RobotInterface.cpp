#include <XBotInterface/RobotInterface.h>

// Static members need to be declared in the cpp as well
XBot::RobotInterface::Ptr XBot::RobotInterface::_instance_ptr;

XBot::RobotInterface::RobotInterface(const XBot::XBotCoreModel& XBotModel) : 
    IXBotInterface(XBotModel),
    model(new IXBotInterface(XBotModel))
{
}

XBot::RobotInterface::Ptr XBot::RobotInterface::getRobot(const std::string& path_to_cfg)
{
	
	
	
	if( _instance_ptr ){ return _instance_ptr; }
	
	
    std::ifstream fin ( path_to_cfg );
    if ( fin.fail() ) {
        printf ( "Can not open %s\n", path_to_cfg.c_str() ); //TBD change it
        return XBot::RobotInterface::Ptr();
    }

    YAML::Node root_cfg = YAML::LoadFile ( path_to_cfg );
    const YAML::Node& x_bot_interface = root_cfg["x_bot_interface"]; // TBD check if exists
    std::string _urdf_path = x_bot_interface["urdf_path"].as<std::string>();
    std::string _srdf_path = x_bot_interface["srdf_path"].as<std::string>();
    std::string _joint_map_config = x_bot_interface["joint_map_config"].as<std::string>();
    
    // initialize the model
    XBotCoreModel XBotModel;
    if( !XBotModel.init( _urdf_path, _srdf_path, _joint_map_config ) ) {
        printf("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n"); //TBD change it
        return XBot::RobotInterface::Ptr();
    }
    // generate the robot
    XBotModel.generate_robot();
    
     std::string _framework = x_bot_interface["framework"].as<std::string>();
     // TBD ifdef MACRO
     if( _framework == "YARP" ) {
// 		 _instance_ptr = RobotInterface::Ptr(new YARPInterface(XBotModel));
		 return _instance_ptr;
//         return XBot::RobotInterface::Ptr(new YARPInterface(XBotModel));
    }
}


bool XBot::RobotInterface::sense(bool sync_model)
{
    bool sense_ok = sense_internal();
    if(sync_model) {
        return sense_ok && sync(*model);
    }
    return sense_ok;
}


bool XBot::RobotInterface::move(bool sync_model)
{
    bool sync_ok = true;
    if(sync_model) {
        sync_ok = sync(*model);
    }
    return sync_ok && move_internal();
}
