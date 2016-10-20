#include <XBotInterface/RobotInterface.h>
#include <ros/ros.h> //TBD: remove include

// NOTE Static members need to be defined in the cpp 
std::string XBot::RobotInterface::_framework;
std::string XBot::RobotInterface::_subclass_name;
std::string XBot::RobotInterface::_path_to_shared_lib;
std::string XBot::RobotInterface::_subclass_factory_name;
XBot::RobotInterface::Ptr XBot::RobotInterface::_instance_ptr;
XBot::ModelInterface::Ptr XBot::RobotInterface::_model;
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

    // loading YAML
    YAML::Node root_cfg = YAML::LoadFile(path_to_cfg);
    YAML::Node x_bot_interface;
    // XBotInterface info
    if(root_cfg["x_bot_interface"]) {
        x_bot_interface = root_cfg["x_bot_interface"]; 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : YAML file  " << path_to_cfg << "  does not contain x_bot_interface mandatory node!!" << std::endl;
        return false;
    }
    // check framework
    if(x_bot_interface["framework"]) {
        _framework = x_bot_interface["framework"].as<std::string>();
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain framework mandatory node!!" << std::endl;
        return false;
    }
    
    // subclass forced
    _subclass_name = std::string("RobotInterface") + _framework;
    // check the path to shared lib
    if(root_cfg[_subclass_name]["path_to_shared_lib"]) {
        computeAbsolutePath(root_cfg[_subclass_name]["path_to_shared_lib"].as<std::string>(), 
                            LIB_MIDDLE_PATH,
                            _path_to_shared_lib); 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain path_to_shared_lib mandatory node!!" << std::endl;
        return false;
    }
    
    if(root_cfg[_subclass_name]["subclass_factory_name"]) {
        _subclass_factory_name = root_cfg[_subclass_name]["subclass_factory_name"].as<std::string>();
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain subclass_factory_name mandatory node!!" << std::endl;
        return false;
    }

}


XBot::RobotInterface::Ptr XBot::RobotInterface::getRobot(const std::string &path_to_cfg, int argc, char **argv)
{
    // NOTE singleton
    if (_instance_ptr) {
        return _instance_ptr;
    }
    // parsing YAML
    if (parseYAML(path_to_cfg)) {
        std::cerr << "ERROR in " << __func__ << " : could not parse the YAML " << path_to_cfg << " . See error above!!" << std::endl;
        return _instance_ptr;
    }
    
    if (_framework == "ROS") {
        ros::init(argc, argv, "from_config"); // TBD remove this ugly reference to implementation details
    }

    // loading the requested robot interface
    shlibpp::SharedLibraryClassFactory<RobotInterface> robot_interface_factory( _path_to_shared_lib.c_str(),
                                                                                _subclass_factory_name.c_str());
    if (!robot_interface_factory.isValid()) {
        // NOTE print to celebrate the wizard
        printf("error (%s) : %s\n", shlibpp::Vocab::decode(robot_interface_factory.getStatus()).c_str(),
               robot_interface_factory.getLastNativeError().c_str());
    }
    // open and init robot interface
    _robot_interface_instance.open(robot_interface_factory); 
    _robot_interface_instance->init(path_to_cfg);
    // static instance of the robot interface
    _instance_ptr = std::shared_ptr<RobotInterface>(&_robot_interface_instance.getContent(), [](RobotInterface* ptr){return;});
    
    // loading the requested model interface internal to the robot
    _model = XBot::ModelInterface::getModel(path_to_cfg);
    
    return _instance_ptr;

}

void XBot::RobotInterface::fillModelOrderedChainFromOrderedJoint ( const std::vector< std::string >& model_ordered_joint_name )
{

    // TBD do it in the base class
    for( int i = 0; i < model_ordered_joint_name.size(); i++ ) {
        
    }
    //_model_ordered_chain_name;
}


const std::vector< std::string >& XBot::RobotInterface::getModelOrderedChainName()
{
//     std::vector<std::string> model_ordered_joint_name;
//     _model->getModelID(model_ordered_joint_name);
//     fillModelOrderedChainFromOrderedJoint(model_ordered_joint_name);
    return _model_ordered_chain_name;
}



bool XBot::RobotInterface::sense(bool sync_model)
{
    bool sense_ok = sense_internal();
    if (sync_model) {
        return sense_ok && (_model->syncFrom(*this));
    }
    return sense_ok;
}


bool XBot::RobotInterface::move(bool sync_model)
{
    bool sync_ok = true;
    if (sync_model) {
        sync_ok = _model->syncFrom(*this);
    }
    return sync_ok && move_internal();
}
