/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <XBotInterface/ModelInterface.h>

// NOTE Static members need to be defined in the cpp 
std::vector<shlibpp::SharedLibraryClass<XBot::ModelInterface> > XBot::ModelInterface::_model_interface_instance;

bool XBot::ModelInterface::parseYAML(const std::string &path_to_cfg, std::map<std::string, std::string>& vars)
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
        vars["model_type"] = x_bot_interface["internal_model_type"].as<std::string>();
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain internal_model_type mandatory node!!" << std::endl;
        return false;
    }
    
    // subclass forced
    vars["subclass_name"] = std::string("ModelInterface") + vars.at("model_type");
    vars["path_to_shared_lib"] = "";
    // check the path to shared lib
    if(root_cfg[vars.at("subclass_name")]["path_to_shared_lib"]) {
        computeAbsolutePath(root_cfg[vars.at("subclass_name")]["path_to_shared_lib"].as<std::string>(), 
                            LIB_MIDDLE_PATH,
                            vars.at("path_to_shared_lib")); 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain path_to_shared_lib mandatory node!!" << std::endl;
        return false;
    }
    
    if(root_cfg[vars.at("subclass_name")]["subclass_factory_name"]) {
        vars["subclass_factory_name"] = root_cfg[vars.at("subclass_name")]["subclass_factory_name"].as<std::string>();
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
    std::map<std::string, std::string> vars;
    
    // parsing YAML
    if (parseYAML(path_to_cfg, vars)) {
        std::cerr << "ERROR in " << __func__ << " : could not parse the YAML " << path_to_cfg << " . See error above!!" << std::endl;
        return instance_ptr;
    }
    
    // loading the requested robot interface
    shlibpp::SharedLibraryClassFactory<ModelInterface> model_interface_factory( vars.at("path_to_shared_lib").c_str(),
                                                                                vars.at("subclass_factory_name").c_str());
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
    return _model_ordered_chain_name;
}
 
bool XBot::ModelInterface::init_internal(const std::string& path_to_cfg)
{
    
    if(!fillModelOrderedChain()){
     
        std::cerr << "ERROR in " << __func__ << ": model interface could not be loaded! Model joint ordering must be chain-by-chain and inside each chain joints must go from base link to tip link!" << std::endl;
        
        return false;
        
    }
    
    // Fill _model_chain_map with shallow copies of chains in _chain_map
    
    _model_chain_map.clear();
    
    for( const auto& c : _chain_map){
        
        ModelChain::Ptr model_chain(new ModelChain());
        model_chain->shallowCopy(*c.second);
        
        _model_chain_map[c.first] = model_chain;
        
    }

    return init_model(path_to_cfg);
}



bool XBot::ModelInterface::fillModelOrderedChain()
{
    
    bool success = true;
    
    std::vector<std::string> model_ordered_joint_name;
    this->getModelID(model_ordered_joint_name);
    
    // if model is floating base add virtual chain with six virtual joints,
    // which are the first six provided by getModelID
    
    int joint_idx = 0;
    _joint_id_to_model_id.clear();
    if(isFloatingBase()) {
        
        std::string virtual_chain_name("virtual_chain");
        
        _chain_map[virtual_chain_name] = std::make_shared<XBot::KinematicChain>(virtual_chain_name);
        
        for(int i=0; i<6; i++){
            XBot::Joint::Ptr jptr = std::make_shared<Joint>(model_ordered_joint_name[i], 
                                                            -(i+1), 
                                                            virtual_chain_name);
            
            _chain_map.at(virtual_chain_name)->pushBackJoint(jptr);
            
            _joint_id_to_model_id[jptr->getJointId()] = joint_idx;
            
            joint_idx++;
        }
    }
    
    
    _model_ordered_chain_name.clear();
    while( joint_idx < model_ordered_joint_name.size() ){
     
        // compute the chain which the joint being processed belongs to
        std::string chain_name = getJointByName(model_ordered_joint_name[joint_idx])->getChainName();
        _model_ordered_chain_name.push_back(chain_name);
        
        // check that the joint that follow are equal to the chain ones,
        // ordered from base link to tip link
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        
        int chain_joint_num = chain.getJointNum();
        
        for( int i = 0; i < chain_joint_num; i++ ){
            
            if( chain.jointName(i) == model_ordered_joint_name[joint_idx] ){
                
                _joint_id_to_model_id[chain.jointId(i)] = joint_idx;
                joint_idx++;
                
            }
            else{
                return false;
            }
        
        }
        
        
    }
    //_model_ordered_chain_name;
        
    return success;
}

XBot::ModelChain& XBot::ModelInterface::arm(int arm_id)
{
    if (_XBotModel.get_arms_chain().size() > arm_id) {
        const std::string &requested_arm_name = _XBotModel.get_arms_chain().at(arm_id);
        return *_model_chain_map.at(requested_arm_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a arms with id " << arm_id << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

XBot::ModelChain& XBot::ModelInterface::leg(int leg_id)
{
    if (_XBotModel.get_legs_chain().size() > leg_id) {
        const std::string &requested_leg_name = _XBotModel.get_legs_chain().at(leg_id);
        return *_model_chain_map.at(requested_leg_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a legs with id " << leg_id << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

XBot::ModelChain& XBot::ModelInterface::operator()(const std::string& chain_name)
{
    if (_model_chain_map.count(chain_name)) {
        return *_model_chain_map.at(chain_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a chain with name " << chain_name << " that does not exists!!" << std::endl;
    return _dummy_chain;
}


bool XBot::ModelInterface::getSpatialAcceleration(const std::string& link_name, 
                                                  Eigen::Matrix< double, 6, 1 >& acceleration) const
{
    bool success = getSpatialAcceleration(link_name, _tmp_kdl_twist);
    
    tf::twistKDLToEigen(_tmp_kdl_twist, acceleration);
    
    return success;
}


bool XBot::ModelInterface::getCOM(const std::string& reference_frame, Eigen::Vector3d& com_position) const
{
    bool success = getCOM(reference_frame, _tmp_kdl_vector);
    
    tf::vectorKDLToEigen(_tmp_kdl_vector, com_position);
    
    return success;
}

void XBot::ModelInterface::getCOM(Eigen::Vector3d& com_position) const
{
    getCOM(_tmp_kdl_vector);
    tf::vectorKDLToEigen(_tmp_kdl_vector, com_position);
}

void XBot::ModelInterface::getCOMAcceleration(Eigen::Vector3d& acceleration) const
{
    getCOMAcceleration(_tmp_kdl_vector);
    tf::vectorKDLToEigen(_tmp_kdl_vector, acceleration);
}

void XBot::ModelInterface::getCOMJacobian(Eigen::MatrixXd& J) const
{
    getCOMJacobian(_tmp_kdl_jacobian);
    J = _tmp_kdl_jacobian.data;
}

void XBot::ModelInterface::getCOMVelocity(Eigen::Vector3d& velocity) const
{
    getCOMVelocity(_tmp_kdl_vector);
    tf::vectorKDLToEigen(_tmp_kdl_vector, velocity);
}

bool XBot::ModelInterface::getGravity(const std::string& reference_frame, Eigen::Vector3d& gravity) const
{
    bool success = getGravity(reference_frame, _tmp_kdl_vector);
    
    tf::vectorKDLToEigen(_tmp_kdl_vector, gravity);
    
    return success;
}

void XBot::ModelInterface::getGravity(Eigen::Vector3d& gravity) const
{
    getGravity(_tmp_kdl_vector);
    tf::vectorKDLToEigen(_tmp_kdl_vector, gravity);
}


bool XBot::ModelInterface::getJacobian(const std::string& link_name, Eigen::MatrixXd& J)
{
    bool success = getJacobian(link_name, _tmp_kdl_jacobian);
    J = _tmp_kdl_jacobian.data;
    return success;
}

bool XBot::ModelInterface::getModelID(const std::string& chain_name, std::vector< int >& model_id_vector) const
{
    if(_chain_map.count(chain_name)){
        
        const KinematicChain& chain = *_chain_map.at(chain_name);
        model_id_vector.resize(chain.getJointNum());
        for(int i=0; i< chain.getJointNum(); i++){
            model_id_vector[i] = _joint_id_to_model_id.at(chain.jointId(i));
        }
        
    }
    else{
     std::cerr << "ERROR in " << __func__ << ": requested chain " << chain_name << " is not defined!" << std::endl;
     return false;
    }
    
}

int XBot::ModelInterface::getModelID(const std::string& joint_name) const
{
    auto jptr = getJointByName(joint_name);
    if(jptr) return _joint_id_to_model_id.at(jptr->getJointId());
    else{
        std::cerr << "ERROR in " << __func__ << ": requested joint " << joint_name << " is not defined!" << std::endl;
        return -1;    
    }
}

// bool XBot::ModelInterface::getOrientation(const std::string& source_frame, const std::string& target_frame, Eigen::Matrix3d& target_point) const
// {
// 
// }
// 
// bool XBot::ModelInterface::getOrientation(const std::string& target_frame, KDL::Rotation& target_point) const
// {
// 
// }
// 
// bool XBot::ModelInterface::getOrientation(const std::string& source_frame, const std::string& target_frame, KDL::Rotation& target_point) const
// {
// 
// }
// 
// bool XBot::ModelInterface::getOrientation(const std::string& target_frame, Eigen::Matrix3d& target_point) const
// {
// 
// }
// 
// bool XBot::ModelInterface::getPointJacobian(const std::string& link_name, const Eigen::Vector3d& point, Eigen::MatrixXd& J)
// {
// 
// }
// 
// bool XBot::ModelInterface::getPointPosition(const std::string& source_frame, const std::string& target_frame, const Eigen::Vector3d& source_point, Eigen::Vector3d& target_point) const
// {
// 
// }
// 
// bool XBot::ModelInterface::getPointPosition(const std::string& source_frame, const std::string& target_frame, const KDL::Vector& source_point, KDL::Vector& target_point) const
// {
// 
// }
// 
// bool XBot::ModelInterface::getPointPosition(const std::string& target_frame, const Eigen::Vector3d& source_point, Eigen::Vector3d& target_point) const
// {
// 
// }
// 
// bool XBot::ModelInterface::getPointPosition(const std::string& target_frame, const KDL::Vector& source_point, KDL::Vector& target_point) const
// {
// 
// }
// 
// bool XBot::ModelInterface::getPose(const std::string& source_frame, Eigen::Affine3d& pose) const
// {
// 
// }
// 
// bool XBot::ModelInterface::getPose(const std::string& source_frame, const std::string& target_frame, Eigen::Affine3d& pose) const
// {
// 
// }
// 
// bool XBot::ModelInterface::getSpatialVelocity(const std::string& link_name, Eigen::Matrix< double, int(6), int(1) >& velocity) const
// {
// 
// }
// 
// bool XBot::ModelInterface::setFloatingBasePose(const Eigen::Affine3d& floating_base_pose)
// {
// 
// }
// 
// bool XBot::ModelInterface::setGravity(const std::string& reference_frame, const Eigen::Vector3d& gravity)
// {
// 
// }
// 
// void XBot::ModelInterface::setGravity(const Eigen::Vector3d& gravity)
// {
// 
// }
// 
// bool XBot::ModelInterface::setGravity(const std::string& reference_frame, const KDL::Vector& gravity)
// {
// 
// }

bool XBot::ModelInterface::setJointEffort(const std::map< std::string, double >& tau)
{
return XBot::IXBotInterface::setJointEffort(tau);
}

bool XBot::ModelInterface::setJointEffort(const std::map< int, double >& tau)
{
return XBot::IXBotInterface::setJointEffort(tau);
}

bool XBot::ModelInterface::setJointEffort(const Eigen::VectorXd& tau)
{
return XBot::IXBotInterface::setJointEffort(tau);
}


bool XBot::ModelInterface::setJointPosition(const std::map< std::string, double >& q)
{
return XBot::IXBotInterface::setJointPosition(q);
}

bool XBot::ModelInterface::setJointPosition(const std::map< int, double >& q)
{
return XBot::IXBotInterface::setJointPosition(q);
}

bool XBot::ModelInterface::setJointPosition(const Eigen::VectorXd& q)
{
return XBot::IXBotInterface::setJointPosition(q);
}

bool XBot::ModelInterface::setJointVelocity(const std::map< std::string, double >& qdot)
{
return XBot::IXBotInterface::setJointVelocity(qdot);
}

bool XBot::ModelInterface::setJointVelocity(const std::map< int, double >& qdot)
{
return XBot::IXBotInterface::setJointVelocity(qdot);
}

bool XBot::ModelInterface::setJointVelocity(const Eigen::VectorXd& qdot)
{
return XBot::IXBotInterface::setJointVelocity(qdot);
}

bool XBot::ModelInterface::setMotorPosition(const std::map< std::string, double >& q)
{
return XBot::IXBotInterface::setMotorPosition(q);
}

bool XBot::ModelInterface::setMotorPosition(const std::map< int, double >& q)
{
return XBot::IXBotInterface::setMotorPosition(q);
}

bool XBot::ModelInterface::setMotorPosition(const Eigen::VectorXd& q)
{
return XBot::IXBotInterface::setMotorPosition(q);
}

bool XBot::ModelInterface::setMotorVelocity(const std::map< std::string, double >& qdot)
{
return XBot::IXBotInterface::setMotorVelocity(qdot);
}

bool XBot::ModelInterface::setMotorVelocity(const std::map< int, double >& qdot)
{
return XBot::IXBotInterface::setMotorVelocity(qdot);
}

bool XBot::ModelInterface::setMotorVelocity(const Eigen::VectorXd& qdot)
{
return XBot::IXBotInterface::setMotorVelocity(qdot);
}

bool XBot::ModelInterface::setTemperature(const std::map< std::string, double >& temp)
{
return XBot::IXBotInterface::setTemperature(temp);
}

bool XBot::ModelInterface::setTemperature(const std::map< int, double >& temp)
{
return XBot::IXBotInterface::setTemperature(temp);
}

bool XBot::ModelInterface::setTemperature(const Eigen::VectorXd& temp)
{
return XBot::IXBotInterface::setTemperature(temp);
}

bool XBot::ModelInterface::syncFrom(const XBot::IXBotInterface& other)
{
return XBot::IXBotInterface::syncFrom(other);
}





















































