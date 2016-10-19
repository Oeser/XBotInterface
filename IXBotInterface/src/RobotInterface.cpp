#include <XBotInterface/RobotInterface.h>
#include <ros/ros.h> //TBD: remove include

#define LIB_MIDDLE_PATH "/build/install/lib/"

// NOTE Static members need to be declared in the cpp as well
std::string XBot::RobotInterface::framework;
std::string XBot::RobotInterface::subclass_name;
std::string XBot::RobotInterface::path_to_shared_lib;
std::string XBot::RobotInterface::subclass_factory_name;
XBot::RobotInterface::Ptr XBot::RobotInterface::_instance_ptr;
shlibpp::SharedLibraryClass<XBot::RobotInterface> XBot::RobotInterface::_robot_interface_instance;

XBot::RobotInterface::RobotInterface()
{
}

bool XBot::RobotInterface::parseYAML(const std::string &path_to_cfg)
{
    std::ifstream fin(path_to_cfg);
    if (fin.fail()) {
        printf("Can not open %s\n", path_to_cfg.c_str());
        return false;
    }

    YAML::Node root_cfg = YAML::LoadFile(path_to_cfg);
    YAML::Node x_bot_interface;
    if(root_cfg["x_bot_interface"]) {
        x_bot_interface = root_cfg["x_bot_interface"]; 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : YAML file  " << path_to_cfg << "  does not contain x_bot_interface mandatory node!!" << std::endl;
        return false;
    }
    // check framework
    if(x_bot_interface["framework"]) {
        framework = x_bot_interface["framework"].as<std::string>();
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain framework mandatory node!!" << std::endl;
        return false;
    }
    
    // subclass forced
    subclass_name = std::string("RobotInterface") + framework;
    // check the path to shared lib
    if(root_cfg[subclass_name]["path_to_shared_lib"]) {
        computeAbsolutePath(root_cfg[subclass_name]["path_to_shared_lib"].as<std::string>(), 
                            LIB_MIDDLE_PATH,
                            path_to_shared_lib); 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain path_to_shared_lib mandatory node!!" << std::endl;
        return false;
    }
    
    if(root_cfg[subclass_name]["subclass_factory_name"]) {
        subclass_factory_name = root_cfg[subclass_name]["subclass_factory_name"].as<std::string>();
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain subclass_factory_name mandatory node!!" << std::endl;
        return false;
    }

}


XBot::RobotInterface::Ptr XBot::RobotInterface::getRobot(const std::string &path_to_cfg, int argc, char **argv)
{

    if (_instance_ptr) {
        return _instance_ptr;
    }
    // parsing YAML
    parseYAML(path_to_cfg);
    
    if (framework == "ROS") {
        ros::init(argc, argv, "from_config"); // TBD remove this ugly reference to implementation details
    }

    shlibpp::SharedLibraryClassFactory<RobotInterface> robot_interface_factory( path_to_shared_lib.c_str(),
                                                                                subclass_factory_name.c_str());
    if (!robot_interface_factory.isValid()) {
        // NOTE print to celebrate the wizard
        printf("error (%s) : %s\n", shlibpp::Vocab::decode(robot_interface_factory.getStatus()).c_str(),
               robot_interface_factory.getLastNativeError().c_str());
    }

    // create an instance of the class and call its functions
    _robot_interface_instance.open(robot_interface_factory); 
    _robot_interface_instance->init(path_to_cfg);

    _instance_ptr = std::shared_ptr<RobotInterface>(&_robot_interface_instance.getContent(), [](RobotInterface* ptr){return;}); // TBD: very wrong!!!

    return _instance_ptr;

}


bool XBot::RobotInterface::sense(bool sync_model)
{
    bool sense_ok = sense_internal();
    if (sync_model) {
        return sense_ok && (model->syncFrom(*this));
    }
    return sense_ok;
}


bool XBot::RobotInterface::move(bool sync_model)
{
    bool sync_ok = true;
    if (sync_model) {
        sync_ok = model->syncFrom(*this);
    }
    return sync_ok && move_internal();
}
