#include <XBotInterface/ModelInterface.h>

// NOTE Static members need to be defined in the cpp 
std::string XBot::ModelInterface::_model_type;
std::string XBot::ModelInterface::_subclass_name;
std::string XBot::ModelInterface::_path_to_shared_lib;
std::string XBot::ModelInterface::_subclass_factory_name;

std::vector<shlibpp::SharedLibraryClass<XBot::ModelInterface> > XBot::ModelInterface::_model_interface_instance;

bool XBot::ModelInterface::parseYAML(const std::string &path_to_cfg)
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
    
    // check model type
    if(x_bot_interface["internal_model_type"]) {
        _model_type = x_bot_interface["internal_model_type"].as<std::string>();
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain internal_model_type mandatory node!!" << std::endl;
        return false;
    }
    
    // subclass forced
    _subclass_name = std::string("ModelInterface") + _model_type;
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

// NOTE check always in the YAML to avoid static issue
bool XBot::ModelInterface::isFloatingBase() const
{
    // check model floating base
    bool is_model_floating_base;
    std::string path_to_cfg = getPathToConfig();
     // loading YAML
    YAML::Node root_cfg = YAML::LoadFile(path_to_cfg);
    YAML::Node x_bot_interface;
    // XBotInterface info
    if(root_cfg["x_bot_interface"]) {
        x_bot_interface = root_cfg["x_bot_interface"]; 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : YAML file  " << path_to_cfg << "  does not contain x_bot_interface mandatory node!!" << std::endl;
    }
    if(x_bot_interface["is_internal_model_flaoting_base"]) {
        is_model_floating_base = x_bot_interface["is_internal_model_flaoting_base"].as<bool>();
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain is_internal_model_flaoting_base mandatory node!!" << std::endl;
    }
    return is_model_floating_base;
}


XBot::ModelInterface::Ptr XBot::ModelInterface::getModel ( const std::string& path_to_cfg )
{
    // Model instance to return
    ModelInterface::Ptr instance_ptr;
    // parsing YAML
    if (parseYAML(path_to_cfg)) {
        std::cerr << "ERROR in " << __func__ << " : could not parse the YAML " << path_to_cfg << " . See error above!!" << std::endl;
        return instance_ptr;
    }
    
    // loading the requested robot interface
    shlibpp::SharedLibraryClassFactory<ModelInterface> model_interface_factory( _path_to_shared_lib.c_str(),
                                                                                _subclass_factory_name.c_str());
    if (!model_interface_factory.isValid()) {
        // NOTE print to celebrate the wizard
        printf("error (%s) : %s\n", shlibpp::Vocab::decode(model_interface_factory.getStatus()).c_str(),
               model_interface_factory.getLastNativeError().c_str());
    }
    // open and init robot interface
    shlibpp::SharedLibraryClass<XBot::ModelInterface> model_instance;
    model_instance.open(model_interface_factory); 
    model_instance->init(path_to_cfg);
    // static instance of the robot interface
    instance_ptr = std::shared_ptr<ModelInterface>(&model_instance.getContent(), [](ModelInterface* ptr){return;});
    
    // save the instance
    _model_interface_instance.push_back(model_instance);    
    
    return instance_ptr;
}

const std::vector< std::string >& XBot::ModelInterface::getModelOrderedChainName()
{
//     std::vector<std::string> model_ordered_joint_name;
//     _model->getModelID(model_ordered_joint_name);
//     fillModelOrderedChainFromOrderedJoint(model_ordered_joint_name);
    return _model_ordered_chain_name;
}


void XBot::ModelInterface::fillModelOrderedChainFromOrderedJoint ( const std::vector< std::string >& model_ordered_joint_name )
{
    if(isFloatingBase()) {
        _virtual_chain = std::make_shared<XBot::KinematicChain>();
        _chain_map["virtual_chain"] = _virtual_chain;
    }
    
    // TBD do it in the base class
    for( int i = 0; i < model_ordered_joint_name.size(); i++ ) {
        
    }
    //_model_ordered_chain_name;
        
}



