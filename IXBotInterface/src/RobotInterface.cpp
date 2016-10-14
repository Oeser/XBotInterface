#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/RobotInterfaceROS.h> //TBD: remove include

// plugin system



// NOTE Static members need to be declared in the cpp as well
XBot::RobotInterface::Ptr XBot::RobotInterface::_instance_ptr;
shlibpp::SharedLibraryClass<XBot::RobotInterface> XBot::RobotInterface::_robot_interface_instance;

XBot::RobotInterface::RobotInterface()
{
}

XBot::RobotInterface::Ptr XBot::RobotInterface::getRobot(const std::string &path_to_cfg, int argc, char **argv)
{

    if (_instance_ptr) {
        return _instance_ptr;
    }

    std::ifstream fin(path_to_cfg);
    if (fin.fail()) {
        printf("Can not open %s\n", path_to_cfg.c_str());    //TBD change it
        return false;
    }

    YAML::Node root_cfg = YAML::LoadFile(path_to_cfg);
    const YAML::Node &x_bot_interface = root_cfg["x_bot_interface"]; // TBD check if exists

    std::string framework = x_bot_interface["framework"].as<std::string>();
    std::string subclass_name = std::string("RobotInterface") + framework;
    std::string path_to_shared_lib = root_cfg[subclass_name]["path_to_shared_lib"].as<std::string>();
    std::string subclass_factory_name = root_cfg[subclass_name]["subclass_factory_name"].as<std::string>();

    if (framework == "ROS") {
        ros::init(argc, argv, "from_config"); // TBD remove this ugly reference to implementation details
    }

    printf("Loading the shared library... \n");
    shlibpp::SharedLibraryClassFactory<RobotInterface> robot_interface_factory(path_to_shared_lib.c_str(),
            subclass_factory_name.c_str());
    if (!robot_interface_factory.isValid()) {
        printf("error (%s) : %s\n", shlibpp::Vocab::decode(robot_interface_factory.getStatus()).c_str(),
               robot_interface_factory.getLastNativeError().c_str());
    }

    // create an instance of the class and call its functions
    _robot_interface_instance.open(robot_interface_factory); //(robot_interface_factory);
    printf("Calling some of its functions... \n");
    _robot_interface_instance->init(path_to_cfg);

    _instance_ptr = std::shared_ptr<RobotInterface>(&_robot_interface_instance.getContent(), [](RobotInterface* ptr){return;}); // TBD: very wrong!!!

    return _instance_ptr;

}


bool XBot::RobotInterface::sense(bool sync_model)
{
    bool sense_ok = sense_internal();
    if (sync_model) {
        return sense_ok && sync(*model);
    }
    return sense_ok;
}


bool XBot::RobotInterface::move(bool sync_model)
{
    bool sync_ok = true;
    if (sync_model) {
        sync_ok = sync(*model);
    }
    return sync_ok && move_internal();
}
