#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/RobotInterfaceROS.h>

// plugin system
#include <SharedLibraryClass.h>
#include <SharedLibrary.h>
// plugin test
#include <MyMath.h>

// NOTE Static members need to be declared in the cpp as well
XBot::RobotInterface::Ptr XBot::RobotInterface::_instance_ptr;

XBot::RobotInterface::RobotInterface()
{
}

XBot::RobotInterface::Ptr XBot::RobotInterface::getRobot(const std::string& path_to_cfg, int argc, char **argv)
{

    if( _instance_ptr ){ return _instance_ptr; }
    
    std::ifstream fin ( path_to_cfg );
    if ( fin.fail() ) {
        printf ( "Can not open %s\n", path_to_cfg.c_str() ); //TBD change it
        return false;
    }

    YAML::Node root_cfg = YAML::LoadFile ( path_to_cfg );
    const YAML::Node& x_bot_interface = root_cfg["x_bot_interface"]; // TBD check if exists

     std::string _framework = x_bot_interface["framework"].as<std::string>();
     // TBD ifdef MACRO
     if( _framework == "YARP" ) {
//         _instance_ptr = RobotInterface::Ptr(new YARPInterface(XBotModel));
        return _instance_ptr;
//         return XBot::RobotInterface::Ptr(new YARPInterface(XBotModel));
    }
    if(_framework == "ROS" ){
        ros::init(argc, argv, "from_config");
//         _instance_ptr = RobotInterface::Ptr(new RobotInterfaceROS(XBotModel));
        
        printf("Loading the shared library... \n");
        shlibpp::SharedLibraryClassFactory<RobotInterface> robot_interface_factory("/home/alaurenzi/Code/robotology-superbuild/build/external/RobotInterfaceROS/libRobotInterfaceROS.so", "robot_interface_ros");
        if (!robot_interface_factory.isValid()) {
            printf("error (%s) : %s\n", shlibpp::Vocab::decode(robot_interface_factory.getStatus()).c_str(),
                                        robot_interface_factory.getLastNativeError().c_str());
        }

        // create an instance of the class and call its functions
        shlibpp::SharedLibraryClass<RobotInterface> robot_interface(robot_interface_factory);    
        printf("Calling some of its functions... \n");
        robot_interface->init(path_to_cfg);
		
		_instance_ptr = std::shared_ptr<RobotInterface>(&robot_interface.getContent()); // TBD: very wrong!!!

        return _instance_ptr;
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
