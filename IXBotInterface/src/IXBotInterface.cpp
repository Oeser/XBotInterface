#include <XBotInterface/IXBotInterface.h>

#include <yaml-cpp/yaml.h>
#include <XBotCoreModel.h>

XBot::IXBotInterface::IXBotInterface(const XBot::XBotCoreModel& XBotModel) : 
    _XBotModel(XBotModel)
{
    _joint_num = XBotModel.get_joint_num();
    XBotModel.get_enabled_joint_ids(_ordered_joint_id);
    XBotModel.get_enabled_joint_names(_ordered_joint_name);
    
    for( const std::string& chain_name : XBotModel.get_chain_names() ) {
        XBot::KinematicChain::Ptr actual_chain = std::make_shared<KinematicChain>(chain_name, 
                                                                                  XBotModel);
        _chain_map[chain_name] = actual_chain;
    }
    
}

XBot::IXBotInterface::IXBotInterface(const XBot::IXBotInterface& other):
  _joint_num(other._joint_num),
  _ordered_joint_name(other._ordered_joint_name),
  _ordered_joint_id(other._ordered_joint_id),
  _XBotModel(other._XBotModel)
{
  
  for( const auto& chain_name_ptr_pair : other._chain_map ){
   
    const std::string& chain_name = chain_name_ptr_pair.first;
    const XBot::KinematicChain::Ptr& other_chainptr = chain_name_ptr_pair.second;
    
    XBot::KinematicChain::Ptr chainptr = std::make_shared<XBot::KinematicChain>();
    *chainptr = *other_chainptr;
    
    _chain_map[chain_name] = chainptr;
    
  }

}



XBot::IXBotInterface::Ptr XBot::IXBotInterface::getRobot(const std::string& cfg)
{
    std::ifstream fin ( cfg );
    if ( fin.fail() ) {
        printf ( "Can not open %s\n", cfg.c_str() ); //TBD change it
        return XBot::IXBotInterface::Ptr();
    }

    YAML::Node root_cfg = YAML::LoadFile ( cfg );
    const YAML::Node& x_bot_interface = root_cfg["x_bot_interface"]; // TBD check if exists
    std::string _urdf_path = x_bot_interface["urdf_path"].as<std::string>();
    std::string _srdf_path = x_bot_interface["srdf_path"].as<std::string>();
    std::string _joint_map_config = x_bot_interface["joint_map_config"].as<std::string>();
    
    // initialize the model
    XBotCoreModel XBotModel;
    if( !XBotModel.init( _urdf_path, _srdf_path, _joint_map_config ) ) {
        printf("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n"); //TBD change it
        return XBot::IXBotInterface::Ptr();
    }
    // generate the robot
    XBotModel.generate_robot();
    
     std::string _framework = x_bot_interface["framework"].as<std::string>();
     // TBD ifdef MACRO
     if( _framework == "YARP" ) {
        return XBot::IXBotInterface::Ptr(new YARPInterface(XBotModel));
    }

}

void XBot::YARPInterface::test()
{
    std::cout << "test from YARP called" << std::endl;
}

XBot::YARPInterface::YARPInterface(const XBot::XBotCoreModel& XBotModel) : 
    IXBotInterface(XBotModel)
{
        std::cout << "YARP created" << std::endl;
}



XBot::IXBotInterface::~IXBotInterface()
{

}

XBot::YARPInterface::~YARPInterface()
{

}
