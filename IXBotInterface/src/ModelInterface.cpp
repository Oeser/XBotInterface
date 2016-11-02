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
shlibpp::SharedLibraryClassFactory<XBot::ModelInterface> XBot::ModelInterface::_model_interface_factory;
std::vector<std::shared_ptr<shlibpp::SharedLibraryClass<XBot::ModelInterface> > > XBot::ModelInterface::_model_interface_instance;

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
    
    return true;

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
    if (!parseYAML(path_to_cfg, vars)) {
        std::cerr << "ERROR in " << __func__ << " : could not parse the YAML " << path_to_cfg << " . See error above!!" << std::endl;
        return instance_ptr;
    }
    
    
    
    
    // loading the requested robot interface
    _model_interface_factory.open( vars.at("path_to_shared_lib").c_str(),
                                   vars.at("subclass_factory_name").c_str());
    if (!_model_interface_factory.isValid()) {
        // NOTE print to celebrate the wizard
        printf("error (%s) : %s\n", shlibpp::Vocab::decode(_model_interface_factory.getStatus()).c_str(),
               _model_interface_factory.getLastNativeError().c_str());
    }
    
    // save the instance
    std::shared_ptr<shlibpp::SharedLibraryClass<XBot::ModelInterface> > ali_ptr(new shlibpp::SharedLibraryClass<XBot::ModelInterface>(_model_interface_factory));
    
    _model_interface_instance.push_back(std::make_shared<shlibpp::SharedLibraryClass<XBot::ModelInterface> >());
    
    shlibpp::SharedLibraryClass<XBot::ModelInterface>& model_instance =  *_model_interface_instance[_model_interface_instance.size()-1];
    
    // open and init robot interface
    model_instance.open(_model_interface_factory); 
    model_instance->init(path_to_cfg);
    // static instance of the robot interface
    instance_ptr = std::shared_ptr<ModelInterface>(&model_instance.getContent(), [](ModelInterface* ptr){return;});

    return instance_ptr;
}

const std::vector< std::string >& XBot::ModelInterface::getModelOrderedChainName() const
{
    return _model_ordered_chain_name;
}
 
bool XBot::ModelInterface::init_internal(const std::string& path_to_cfg)
{
    if(!init_model(path_to_cfg)){
        
        std::cerr << "ERROR in " << __func__ << ": model interface could not be initialized!" << std::endl;
        
        return false;
    }
    
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
    
    

    return true;
}



bool XBot::ModelInterface::fillModelOrderedChain()
{
    
    bool success = true;
    
    std::vector<std::string> model_ordered_joint_name;
    this->getModelOrderedJoints(model_ordered_joint_name);
    
    // if model is floating base add virtual chain with six virtual joints,
    // which are the first six provided by getModelOrderedJoints
    
    int joint_idx = 0;
    _joint_id_to_model_id.clear();
    if(isFloatingBase()) {
        
        std::string virtual_chain_name("virtual_chain");
        
        _chain_map[virtual_chain_name] = std::make_shared<XBot::KinematicChain>(virtual_chain_name);
        
        for(int i=0; i<6; i++){
            XBot::Joint::Ptr jptr = std::make_shared<Joint>(model_ordered_joint_name[i], 
                                                            -(i+1), 
                                                            urdf::JointConstSharedPtr(),
                                                            virtual_chain_name);
            
            _chain_map.at(virtual_chain_name)->pushBackJoint(jptr);
            
            _joint_id_to_model_id[jptr->getJointId()] = joint_idx;
            
            joint_idx++;
        }
        
//         _model_ordered_chain_name.push_back(virtual_chain_name);
    }
    
    

    joint_idx = 0;
    while( joint_idx < model_ordered_joint_name.size() ){
     
        // compute the chain which the joint being processed belongs to
        std::string joint_name = model_ordered_joint_name[joint_idx];
        std::string chain_name = getJointByName(joint_name)->getChainName();
        
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
    
    std::cout << "Model ordered chains: " << std::endl;
    for(const auto& s : _model_ordered_chain_name) std::cout << s << std::endl;
        
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

XBot::ModelChain& XBot::ModelInterface::chain(const std::string& chain_name)
{
    return operator()(chain_name);
}


bool XBot::ModelInterface::getAccelerationTwist(const std::string& link_name, 
                                                  Eigen::Matrix< double, 6, 1 >& acceleration) const
{
    bool success = getAccelerationTwist(link_name, _tmp_kdl_twist);
    
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

bool XBot::ModelInterface::getCOM(const std::string& reference_frame, KDL::Vector& com_position) const
{
    getCOM(com_position);
    bool success = getPose(reference_frame, _tmp_kdl_frame);
    com_position = _tmp_kdl_frame.Inverse(com_position);
    return success;
}

bool XBot::ModelInterface::getGravity(const std::string& reference_frame, KDL::Vector& gravity) const
{
    getGravity(gravity);
    bool success = getPose(reference_frame, _tmp_kdl_frame);
    gravity = _tmp_kdl_frame.Inverse(gravity); 
    return success;
}

bool XBot::ModelInterface::getJacobian(const std::string& link_name, KDL::Jacobian& J) const
{
    SetToZero(_tmp_kdl_vector);
    return getJacobian(link_name, _tmp_kdl_vector, J);

}


bool XBot::ModelInterface::getPose(const std::string& source_frame, 
                                   const std::string& target_frame, 
                                   KDL::Frame& pose) const
{
    bool success_source = getPose(source_frame, _tmp_kdl_frame);
    bool success_target = getPose(target_frame, _tmp_kdl_frame_1);
    
    pose = _tmp_kdl_frame_1.Inverse()*_tmp_kdl_frame;
    
}



bool XBot::ModelInterface::getOrientation(const std::string& source_frame, 
                                          const std::string& target_frame, 
                                          Eigen::Matrix3d& orientation) const
{
    bool success = getOrientation(source_frame, target_frame, _tmp_kdl_rotation);
    rotationKDLToEigen(_tmp_kdl_rotation, orientation);
    return success;
}

bool XBot::ModelInterface::getOrientation(const std::string& target_frame, KDL::Rotation& orientation) const
{
    bool success = getPose(target_frame, _tmp_kdl_frame);
    orientation = _tmp_kdl_frame.M;
    return success;;
}

bool XBot::ModelInterface::getOrientation(const std::string& source_frame, 
                                          const std::string& target_frame, 
                                          KDL::Rotation& orientation) const
{
    bool success = getOrientation(source_frame, _tmp_kdl_rotation);
    success = success && getOrientation(target_frame, _tmp_kdl_rotation_1);
    orientation = _tmp_kdl_rotation_1.Inverse()*_tmp_kdl_rotation;
    return success;
}

bool XBot::ModelInterface::getOrientation(const std::string& target_frame, Eigen::Matrix3d& orientation) const
{
    bool success = getOrientation(target_frame, _tmp_kdl_rotation);
    rotationKDLToEigen(_tmp_kdl_rotation, orientation);
    return success;
}

bool XBot::ModelInterface::getJacobian(const std::string& link_name, 
                                       const Eigen::Vector3d& point, 
                                       Eigen::MatrixXd& J)
{
    tf::vectorEigenToKDL(point, _tmp_kdl_vector);
    bool success = getJacobian(link_name, _tmp_kdl_vector, _tmp_kdl_jacobian);
    J = _tmp_kdl_jacobian.data;
    return success;
}

bool XBot::ModelInterface::getPointPosition(const std::string& source_frame, 
                                            const std::string& target_frame, 
                                            const Eigen::Vector3d& source_point, 
                                            Eigen::Vector3d& target_point) const
{
    tf::vectorEigenToKDL(source_point, _tmp_kdl_vector);
    bool success = getPointPosition(source_frame, target_frame, _tmp_kdl_vector, _tmp_kdl_vector);
    tf::vectorKDLToEigen(_tmp_kdl_vector, target_point);
    return success;
}

bool XBot::ModelInterface::getPointPosition(const std::string& source_frame, 
                                            const std::string& target_frame, 
                                            const KDL::Vector& source_point, 
                                            KDL::Vector& target_point) const
{
    bool success = getPose(source_frame, target_frame, _tmp_kdl_frame);
    target_point = _tmp_kdl_frame*source_point;
    return success;
}

bool XBot::ModelInterface::getPointPosition(const std::string& target_frame, 
                                            const Eigen::Vector3d& source_point, 
                                            Eigen::Vector3d& target_point) const
{
    tf::vectorEigenToKDL(source_point, _tmp_kdl_vector_1);
    bool success = getPointPosition(target_frame, _tmp_kdl_vector_1, _tmp_kdl_vector);
    tf::vectorKDLToEigen(_tmp_kdl_vector, target_point);
    return success;
}

bool XBot::ModelInterface::getPointPosition(const std::string& target_frame, 
                                            const KDL::Vector& source_point, 
                                            KDL::Vector& target_point) const
{
    bool success = getPose(target_frame, _tmp_kdl_frame);
    target_point = _tmp_kdl_frame.Inverse(source_point);
    return success;
}

bool XBot::ModelInterface::getPose(const std::string& source_frame, Eigen::Affine3d& pose) const
{
    bool success = getPose(source_frame, _tmp_kdl_frame);
    tf::transformKDLToEigen(_tmp_kdl_frame, pose);
    return success;
    
}

bool XBot::ModelInterface::getPose(const std::string& source_frame, const std::string& target_frame, Eigen::Affine3d& pose) const
{
    bool success = getPose(source_frame, target_frame, _tmp_kdl_frame);
    tf::transformKDLToEigen(_tmp_kdl_frame, pose);
    return success;
}

bool XBot::ModelInterface::getVelocityTwist(const std::string& link_name, 
                                              Eigen::Matrix< double, int(6), int(1) >& velocity) const
{
    bool success = getVelocityTwist(link_name, _tmp_kdl_twist);
    tf::twistKDLToEigen(_tmp_kdl_twist, velocity);
    return success;
}

bool XBot::ModelInterface::setFloatingBasePose(const Eigen::Affine3d& floating_base_pose)
{
    tf::transformEigenToKDL(floating_base_pose, _tmp_kdl_frame);
    return setFloatingBasePose(_tmp_kdl_frame);
    
}

bool XBot::ModelInterface::setGravity(const std::string& reference_frame, const Eigen::Vector3d& gravity)
{
    tf::vectorEigenToKDL(gravity, _tmp_kdl_vector);
    return setGravity(reference_frame, _tmp_kdl_vector);
}

void XBot::ModelInterface::setGravity(const Eigen::Vector3d& gravity)
{
    tf::vectorEigenToKDL(gravity, _tmp_kdl_vector);
    setGravity(_tmp_kdl_vector);
}

bool XBot::ModelInterface::setGravity(const std::string& reference_frame, const KDL::Vector& gravity)
{
    bool success = getPose(reference_frame, _tmp_kdl_frame);
    setGravity(_tmp_kdl_frame*gravity);
    return success;
}

bool XBot::ModelInterface::computeJdotQdot(const std::string& link_name, 
                         const Eigen::Vector3d& point, 
                         Eigen::Matrix<double,6,1>& jdotqdot) const
{
    tf::vectorEigenToKDL(point, _tmp_kdl_vector);
    bool success = computeJdotQdot(link_name, _tmp_kdl_vector, _tmp_kdl_twist);
    tf::twistKDLToEigen(_tmp_kdl_twist, jdotqdot);
    return success;
}


bool XBot::ModelInterface::getPointAcceleration(const std::string& link_name, 
                                                const Eigen::Vector3d& point, 
                                                Eigen::Vector3d& acceleration) const
{
    tf::vectorEigenToKDL(point, _tmp_kdl_vector);
    bool success = getPointAcceleration(link_name, _tmp_kdl_vector, _tmp_kdl_vector_1);
    tf::vectorKDLToEigen(_tmp_kdl_vector_1, acceleration);
    return success;
}

void XBot::ModelInterface::getCOMAcceleration(Eigen::Vector3d& acceleration) const
{
    getCOMAcceleration(_tmp_kdl_vector);
    tf::vectorKDLToEigen(_tmp_kdl_vector, acceleration);
}

bool XBot::ModelInterface::getChainSelectionMatrix(const std::string& chain_name, 
                                                   Eigen::MatrixXd& S) const
{

    if(!getEigenID(chain_name, _tmp_int_vector)) return false;
    
    S.setZero(_tmp_int_vector.size(), getJointNum());
    for( int i = 0; i < _tmp_int_vector.size(); i++ ){
        S(i, _tmp_int_vector[i]) = 1;
    }
    
    return true;
}

bool XBot::ModelInterface::getJointSelectionMatrix(int joint_id, 
                                                   Eigen::RowVectorXd& S) const
{
    int idx = getEigenID(joint_id);
    if(idx >= 0){
        S.resize(getJointNum());
        S(idx) = 1;
    }
    else return false;
}

bool XBot::ModelInterface::getJointSelectionMatrix(const std::string& joint_name, 
                                                   Eigen::RowVectorXd& S) const
{
    int joint_id = getEigenID(joint_name);
    if( joint_id < 0 ) return false; 
    S.setZero(getJointNum());
    S(joint_id) = 1;
}




bool XBot::ModelInterface::syncFrom(const XBot::IXBotInterface& other)
{
    if( sync_internal(other) ){
        update(true, true, true);
        return true;
    }
    else{
        return false;
    }
}



















































