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
#include <XBotInterface/Utils.h>
#include <srdfdom_advr/model.h>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/SVD>
#include <dlfcn.h>
#include <XBotInterface/SoLib.h>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;


bool XBot::ModelInterface::parseYAML(const std::string &path_to_cfg, std::map<std::string, std::string>& vars)
{
    std::ifstream fin(path_to_cfg);
    if (fin.fail()) {
        std::cerr << "ERROR in " << __func__ << "! Can NOT open " << path_to_cfg << "!" << std::endl;
        return false;
    }

    // loading YAML
    YAML::Node root_cfg = YAML::LoadFile(path_to_cfg);
    
    // core YAML
    std::string core_absolute_path;
    computeAbsolutePath("core.yaml", // NOTE we fixed it.
                        "/",
                        core_absolute_path);
    YAML::Node core_cfg = YAML::LoadFile(core_absolute_path);
    
    YAML::Node x_bot_interface;
    // XBotInterface info
    if(root_cfg["ModelInterface"]) {
        x_bot_interface = root_cfg["ModelInterface"];
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : YAML file  " << path_to_cfg << " does not contain ModelInterface mandatory node!!!" << std::endl;
        return false;
    }

    // check model type
    if(x_bot_interface["model_type"]) {
        vars["model_type"] = x_bot_interface["model_type"].as<std::string>();
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : RobotInterface node of  " << path_to_cfg << " does not contain model_type mandatory node!!" << std::endl;
        return false;
    }

    // subclass forced
    vars["subclass_name"] = std::string("ModelInterface") + vars.at("model_type");
    vars["path_to_shared_lib"] = "";
    // check the path to shared lib
    if(core_cfg[vars.at("subclass_name")]["path_to_shared_lib"]) {
        computeAbsolutePath(core_cfg[vars.at("subclass_name")]["path_to_shared_lib"].as<std::string>(),
                            LIB_MIDDLE_PATH,
                            vars.at("path_to_shared_lib"));
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : YAML file  " << path_to_cfg << " or core.yaml  does not contain " << vars.at("subclass_name") << " mandatory node!!" << std::endl;
        return false;
    }

    if(core_cfg[vars.at("subclass_name")]["subclass_factory_name"]) {
        vars["subclass_factory_name"] = core_cfg[vars.at("subclass_name")]["subclass_factory_name"].as<std::string>();
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : " << vars.at("subclass_name") << " node of  " << path_to_cfg << " or core.yaml does not contain subclass_factory_name mandatory node!!" << std::endl;
        return false;
    }


    return true;

}

// NOTE check always in the YAML to avoid static issue
bool XBot::ModelInterface::isFloatingBase() const
{
    return _is_floating_base;
}


XBot::ModelInterface::Ptr XBot::ModelInterface::getModel ( const std::string& path_to_config, AnyMapConstPtr any_map )
{
    
    // Model instance to return
    ModelInterface::Ptr instance_ptr;
    std::map<std::string, std::string> vars;
    
    //compute absolute path
    std::string abs_path_to_cfg = XBot::Utils::computeAbsolutePath(path_to_config);

    // parsing YAML
    if (!parseYAML(abs_path_to_cfg, vars)) {
        std::cerr << "ERROR in " << __func__ << " : could not parse the YAML " << abs_path_to_cfg << " . See error above!!" << std::endl;
        return instance_ptr;
    }

    // check model floating base
    bool is_model_floating_base;
     // loading YAML
    YAML::Node root_cfg = YAML::LoadFile(abs_path_to_cfg);
    YAML::Node x_bot_interface;
    // XBotInterface info
    if(root_cfg["ModelInterface"]) {
        x_bot_interface = root_cfg["ModelInterface"];
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : YAML file  " << abs_path_to_cfg << "  does not contain ModelInterface mandatory node!!" << std::endl;
    }
    if(x_bot_interface["is_model_floating_base"]) {
        is_model_floating_base = x_bot_interface["is_model_floating_base"].as<bool>();
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : ModelInterface node of  " << abs_path_to_cfg << "  does not contain is_model_floating_base mandatory node!!" << std::endl;
    }

     instance_ptr  = SoLib::getFactory<XBot::ModelInterface>(vars.at("path_to_shared_lib"),"MODEL INTERFACE");
     if( instance_ptr){
            instance_ptr->_is_floating_base = is_model_floating_base;
            instance_ptr->init(abs_path_to_cfg, any_map);
      }

    return instance_ptr;
}


bool XBot::ModelInterface::init_internal(const std::string& path_to_cfg, AnyMapConstPtr any_map)
{

    // set is_floating_base

    std::ifstream fin(path_to_cfg);
    if (fin.fail()) {
        std::cerr << "ERROR in " << __func__ << "! Can NOT open " << path_to_cfg << "!" << std::endl;
        return false;
    }

    // loading YAML
    YAML::Node root_cfg = YAML::LoadFile(path_to_cfg);

    if(!init_model(path_to_cfg)){

        Logger::error() << "ERROR in " << __func__ << ": model interface could not be initialized!" << Logger::endl();

        return false;
    }

    if(!fillModelOrderedChain()){

        Logger::error() << "ERROR in " << __func__ << ": model interface could not be loaded! \nModel joint ordering must be chain-by-chain and inside each chain joints must go from base link to tip link!" << Logger::endl();

        return false;

    }

    // Fill _model_chain_map with shallow copies of chains in _chain_map
    _model_chain_map.clear();

    // update model _chain_map removing the chain with only controlled fixed joint
    seekAndDestroyFixedControlledJoints();

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

            boost::shared_ptr<urdf::Joint> virtual_joint(new urdf::Joint);
            virtual_joint->type = urdf::Joint::UNKNOWN;
            virtual_joint->limits.reset(new urdf::JointLimits);
            virtual_joint->limits->effort = 1e9;
            virtual_joint->limits->lower = -1e9;
            virtual_joint->limits->upper = +1e9;
            virtual_joint->limits->velocity = 1e9;


            XBot::Joint::Ptr jptr = std::make_shared<Joint>(model_ordered_joint_name[i],
                                                            -(i+1),
                                                            virtual_joint,
                                                            virtual_chain_name);

            _chain_map.at(virtual_chain_name)->pushBackJoint(jptr);

            _joint_id_to_model_id[jptr->getJointId()] = joint_idx;

            joint_idx++;
        }

//         _model_ordered_chain_name.push_back(virtual_chain_name);
    }


    _ordered_chain_names.clear();
    joint_idx = 0;
    while( joint_idx < model_ordered_joint_name.size() ){

        // compute the chain which the joint being processed belongs to
        std::string joint_name = model_ordered_joint_name[joint_idx];
        XBot::Joint::ConstPtr joint_ptr = getJointByName(joint_name);
        if(!joint_ptr) {
            return false;
        }

        std::string chain_name = joint_ptr->getChainName();

        _ordered_chain_names.push_back(chain_name);

        // check that the joint that follow are equal to the chain ones,
        // ordered from base link to tip link
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);

        int chain_joint_num = chain.getJointNum();

        for( int i = 0; i < chain_joint_num; i++ ){

            if( chain.getJointName(i) == model_ordered_joint_name[joint_idx] ){

                _joint_id_to_model_id[chain.getJointId(i)] = joint_idx;
                joint_idx++;

            }
            else{
                return false;
            }

        }


    }

    //_model_ordered_chain_name;

    Logger::info() << "Model ordered chains: \n";
    for(const auto& s : _ordered_chain_names) Logger::log() << s << "\n";
    Logger::log() << Logger::endl();

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

const XBot::ModelChain& XBot::ModelInterface::arm(int arm_id) const
{
     if (_XBotModel.get_arms_chain().size() > arm_id) {
        const std::string &requested_arm_name = _XBotModel.get_arms_chain().at(arm_id);
        return *_model_chain_map.at(requested_arm_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a arms with id " << arm_id << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

const XBot::ModelChain& XBot::ModelInterface::chain(const std::string& chain_name) const
{
    return operator()(chain_name);
}

const XBot::ModelChain& XBot::ModelInterface::leg(int leg_id) const
{
    if (_XBotModel.get_legs_chain().size() > leg_id) {
        const std::string &requested_leg_name = _XBotModel.get_legs_chain().at(leg_id);
        return *_model_chain_map.at(requested_leg_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a legs with id " << leg_id << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

const XBot::ModelChain& XBot::ModelInterface::operator()(const std::string& chain_name) const
{
    if (_model_chain_map.count(chain_name)) {
        return *_model_chain_map.at(chain_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a chain with name " << chain_name << " that does not exists!!" << std::endl;
    return _dummy_chain;
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
    J = _tmp_kdl_jacobian.data.block(0,0,3,getJointNum());
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


bool XBot::ModelInterface::getJacobian(const std::string& link_name, Eigen::MatrixXd& J) const
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


bool XBot::ModelInterface::getRelativeJacobian(const std::string& target_link_name,
                                               const std::string& base_link_name,
                                               KDL::Jacobian& J) const
{
    /* Define some alias for easier reading */
    KDL::Jacobian& base_jacobian = _tmp_kdl_jacobian;
    KDL::Jacobian& target_jacobian = _tmp_kdl_jacobian_1;
    KDL::Rotation& world_R_base = _tmp_kdl_rotation;
    KDL::Vector& world_deltap_21 = _tmp_kdl_vector;

    /* Compute base e target link jacobians */
    bool success = getJacobian(base_link_name, base_jacobian);
    success = getJacobian(target_link_name, target_jacobian) && success;

    /* Compute orientation of base link w.r.t world */
    success = getOrientation(base_link_name, world_R_base) && success;

    /* Compute the vector origin_target-origin_base w.r.t. world */
    KDL::Vector world_origin_base, world_origin_target;
    getPointPosition(base_link_name, KDL::Vector::Zero(), world_origin_base);
    getPointPosition(target_link_name, KDL::Vector::Zero(), world_origin_target);
    world_deltap_21 = world_origin_target - world_origin_base;

    /* Change ref point of base jacobian from origin_base to origin_target */
    base_jacobian.changeRefPoint(world_deltap_21);

    /* Compute relative jacobian in world frame */
    J.data = target_jacobian.data - base_jacobian.data;

    /* Rotate to base frame */
    J.changeBase(world_R_base.Inverse());


    return success;
}

bool XBot::ModelInterface::getRelativeJacobian(const std::string& target_link_name,
                                               const std::string& base_link_name,
                                               Eigen::MatrixXd& J) const
{
    bool success = getRelativeJacobian(target_link_name, base_link_name, _tmp_kdl_jacobian);
    J = _tmp_kdl_jacobian.data;
    return success;
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
                                       Eigen::MatrixXd& J) const
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

bool XBot::ModelInterface::getPointPosition(const std::string& source_frame,
                                            const Eigen::Vector3d& source_point,
                                            Eigen::Vector3d& world_point) const
{
    tf::vectorEigenToKDL(source_point, _tmp_kdl_vector_1);
    bool success = getPointPosition(source_frame, _tmp_kdl_vector_1, _tmp_kdl_vector);
    tf::vectorKDLToEigen(_tmp_kdl_vector, world_point);
    return success;
}

bool XBot::ModelInterface::getPointPosition(const std::string& source_frame,
                                            const KDL::Vector& source_point,
                                            KDL::Vector& world_point) const
{
    bool success = getPose(source_frame, _tmp_kdl_frame);
    world_point = _tmp_kdl_frame*(source_point);
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

    if(!getDofIndex(chain_name, _tmp_int_vector)) return false;

    S.setZero(_tmp_int_vector.size(), getJointNum());
    for( int i = 0; i < _tmp_int_vector.size(); i++ ){
        S(i, _tmp_int_vector[i]) = 1;
    }

    return true;
}

bool XBot::ModelInterface::getJointSelectionMatrix(int joint_id,
                                                   Eigen::RowVectorXd& S) const
{
    int idx = getDofIndex(joint_id);
    if(idx >= 0){
        S.resize(getJointNum());
        S(idx) = 1;
    }
    else return false;
}


bool XBot::ModelInterface::getJointSelectionMatrix(const std::string& joint_name,
                                                   Eigen::RowVectorXd& S) const
{
    int joint_id = getDofIndex(joint_name);
    if( joint_id < 0 ) return false;
    S.setZero(getJointNum());
    S(joint_id) = 1;
}


bool XBot::ModelInterface::maskJacobian(const std::string& chain_name, Eigen::MatrixXd& J) const
{
    if( J.cols() < getJointNum() ){
        std::cerr << "ERROR in " << __func__ << "! Provided jacobian has less than " << getJointNum() << " columns!" << std::endl;
        return false;
    }
    auto it = _chain_map.find(chain_name);
    if( it == _chain_map.end() ){
        std::cerr << "ERROR in " << __func__ << "! Chain " << chain_name << " is NOT defined!" << std::endl;
        return false;
    }

    const KinematicChain& chain = *(it->second);
    for( int i = 0; i < chain.getJointNum(); i++ ){
        int dof_idx = getDofIndex(chain.getJointId(i));
        J.col(dof_idx).setZero();
    }

    return true;
}

bool XBot::ModelInterface::maskJacobian(const std::string& chain_name, KDL::Jacobian& J) const
{
    const Eigen::MatrixXd& Je = J.data;
    return maskJacobian(chain_name, const_cast<Eigen::MatrixXd&>(Je));
}

bool XBot::ModelInterface::computeConstrainedInverseDynamics(const Eigen::MatrixXd& J,
                                                             const Eigen::VectorXd& weights,
                                                             Eigen::VectorXd& tau) const
{
    int num_constraints = J.rows();
    Eigen::VectorXd _h, _b, _w;
    Eigen::MatrixXd _Q, _A;
    computeInverseDynamics(_h);

    Eigen::HouseholderQR<Eigen::MatrixXd> _qr(J.transpose());
    _Q = _qr.householderQ();

    _b = (_Q.transpose() * _h).tail(getJointNum() - num_constraints);
    _A = _Q.bottomRows(getJointNum()).transpose().bottomRows(getJointNum() - num_constraints);

    _w = weights.array().inverse().sqrt();

    _A.noalias() = _A*_w.asDiagonal();

    tau = _w.asDiagonal() * _A.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(_b);

}

void XBot::ModelInterface::seekAndDestroyFixedControlledJoints()
{
    for( auto& c : _chain_map) {
        XBot::KinematicChain& kin_chain = *c.second;
        // find the joints to remove

        for(int i = 0; i < kin_chain.getJointNum(); i++) {
            if( kin_chain.getJoint(i)->isFixedControlledJoint() ) {
                // remove the joints
                kin_chain.removeJoint(i);
                i--;
            }
        }
        // remove the chains if it is empty
        if(kin_chain.getJointNum() == 0) {
            _chain_map.erase(c.first);
        }
    }
}

void XBot::ModelInterface::getPosturalJacobian(Eigen::MatrixXd& J) const
{
    if(isFloatingBase()){
        J.setZero(getJointNum() - 6, getJointNum());
        J.rightCols(getJointNum() - 6) = Eigen::MatrixXd::Identity(getJointNum() - 6, getJointNum() - 6);
    }else{
        J.setIdentity(getJointNum(), getJointNum());
    }
}




int XBot::ModelInterface::getActuatedJointNum() const
{
    return isFloatingBase() ? (getJointNum()-6) : getJointNum();
}


bool XBot::ModelInterface::setFloatingBaseTwist(const Eigen::Vector6d& floating_base_twist)
{
    tf::twistEigenToKDL(floating_base_twist, _tmp_kdl_twist);
    return setFloatingBaseTwist(_tmp_kdl_twist);
}


bool XBot::ModelInterface::getFloatingBasePose(Eigen::Affine3d& floating_base_pose) const
{
    bool success = getFloatingBasePose(_tmp_kdl_frame);

    if(success){
        tf::transformKDLToEigen(_tmp_kdl_frame, floating_base_pose);
    }

    return success;
}

bool XBot::ModelInterface::getFloatingBaseTwist(Eigen::Vector6d& floating_base_twist) const
{
    bool success = getFloatingBaseTwist(_tmp_kdl_twist);

    if(success){
        tf::twistKDLToEigen(_tmp_kdl_twist, floating_base_twist);
    }

    return success;
}

bool XBot::ModelInterface::getFloatingBasePose(KDL::Frame& floating_base_pose) const
{
    std::string fb_link;
    bool success = getFloatingBaseLink(fb_link);
    success = getPose(fb_link, floating_base_pose);
    return success;

}

bool XBot::ModelInterface::getFloatingBaseTwist(KDL::Twist& floating_base_twist) const
{
    std::string fb_link;
    bool success = getFloatingBaseLink(fb_link);
    success = getVelocityTwist(fb_link, floating_base_twist);
    return success;
}

bool XBot::ModelInterface::getJacobian(const std::string& link_name,
                                       const std::string& target_frame,
                                       KDL::Jacobian& J) const
{
    bool success = getJacobian(link_name, J);
    success = getOrientation(target_frame, _tmp_kdl_rotation) && success;
    J.changeBase(_tmp_kdl_rotation.Inverse());
    return success;
}

bool XBot::ModelInterface::getJacobian(const std::string& link_name,
                                       const std::string& target_frame,
                                       Eigen::MatrixXd& J) const
{
    bool success = getJacobian(link_name, target_frame, _tmp_kdl_jacobian);
    J = _tmp_kdl_jacobian.data;
    return success;
}


bool XBot::ModelInterface::setFloatingBaseAngularVelocity(const Eigen::Vector3d& floating_base_omega)
{
    Eigen::Vector6d twist;
    bool success = getFloatingBaseTwist(twist);

    if(!success) return false;

    twist.tail<3>() = floating_base_omega;

    return setFloatingBaseTwist(twist);
}

bool XBot::ModelInterface::setFloatingBaseAngularVelocity(const KDL::Vector& floating_base_omega)
{
    KDL::Twist twist;
    bool success = getFloatingBaseTwist(twist);

    if(!success) return false;

    twist.rot = floating_base_omega;

    return setFloatingBaseTwist(twist);
}

bool XBot::ModelInterface::setFloatingBaseOrientation(const Eigen::Matrix3d& world_R_floating_base)
{
    Eigen::Affine3d w_T_fb;
    bool success = getFloatingBasePose(w_T_fb);

    if(!success) return false;

    w_T_fb.linear() = world_R_floating_base;

    return setFloatingBasePose(w_T_fb);

}

bool XBot::ModelInterface::setFloatingBaseOrientation(const KDL::Rotation& world_R_floating_base)
{
    KDL::Frame w_T_fb;
    bool success = getFloatingBasePose(w_T_fb);

    if(!success) return false;

    w_T_fb.M = world_R_floating_base;

    return setFloatingBasePose(w_T_fb);
}

void XBot::ModelInterface::getCentroidalMomentumMatrix(Eigen::MatrixXd& centroidal_momentum_matrix) const
{
    Eigen::Vector6d cmmdotqdot;
    getCentroidalMomentumMatrix(centroidal_momentum_matrix, cmmdotqdot);
}

void XBot::ModelInterface::getCentroidalMomentumMatrix(Eigen::MatrixXd& centroidal_momentum_matrix, 
                                                       Eigen::Vector6d& CMMdotQdot) const
{
    centroidal_momentum_matrix.setZero(6, getJointNum());
    
    if(!isFloatingBase()){
        std::cerr << "ERROR in " << __func__ << "! Only implemented for floating base robots!" << std::endl;
        return;
    }
    getInertiaMatrix(_tmp_inertia);
    computeNonlinearTerm(_tmp_nleffect);
    computeGravityCompensation(_tmp_gcomp);
    Eigen::Vector3d com;
    getCOM(com);
    
    Eigen::Affine3d w_T_fb;
    getFloatingBasePose(w_T_fb);
    
    Eigen::Vector3d fb_com = w_T_fb.inverse() * com;
    
    getFloatingBaseLink(_floating_base_link);
    getJacobian(_floating_base_link, fb_com, _tmp_jacobian);
    
    Eigen::Matrix<double, 6, 6> Ju = _tmp_jacobian.block<6,6>(0,0);
    Eigen::Matrix<double, 6, 6> Ju_T_inv = Ju.transpose().inverse();
    
    centroidal_momentum_matrix = Ju_T_inv * _tmp_inertia.block(0, 0, 6, getJointNum());
    CMMdotQdot = Ju_T_inv * (_tmp_nleffect - _tmp_gcomp).head<6>();
    
}

void XBot::ModelInterface::getCOMJacobian(Eigen::MatrixXd& J, Eigen::Vector3d& dJcomQdot) const
{
    Eigen::Vector6d dcmmqdot;
    getCentroidalMomentumMatrix(_tmp_jacobian, dcmmqdot);
    J = _tmp_jacobian.topRows(3) / getMass();
    dJcomQdot = dcmmqdot.head<3>() / getMass();
}


void XBot::ModelInterface::getCOMJacobian(KDL::Jacobian& J, KDL::Vector& dJcomQdot) const
{
    std::cout << "TBD IMPLEMENT!!!" << std::endl;
}

void XBot::ModelInterface::getCOMJacobian(KDL::Jacobian& J) const
{
    std::cout << "TBD IMPLEMENT!!!" << std::endl;
}

void XBot::ModelInterface::getInertiaInverseTimesVector(const Eigen::VectorXd& vec, Eigen::VectorXd& minv_vec) const
{
    _tmp_M.setIdentity(getJointNum(), getJointNum());
    getInertiaMatrix(_tmp_M);
    minv_vec = _tmp_M.inverse()*vec;
}

void XBot::ModelInterface::getInertiaInverseTimesMatrix(const Eigen::MatrixXd& Mat, Eigen::MatrixXd& Minv_Mat) const
{
    Minv_Mat.setZero(getJointNum(), getJointNum());
    _tmp_inv_inertia.setZero(getJointNum());
    for(int i = 0; i < Minv_Mat.rows(); ++i)
    {
        getInertiaInverseTimesVector(Mat.col(i), _tmp_inv_inertia);
        Minv_Mat.col(i) = _tmp_inv_inertia;
    }
}

void XBot::ModelInterface::getInertiaInverse(Eigen::MatrixXd& Minv) const
{
    _tmp_I.setIdentity(getJointNum(), getJointNum());
    getInertiaInverseTimesMatrix(_tmp_I, Minv);
}

bool XBot::ModelInterface::setFloatingBaseState(const Eigen::Affine3d& pose, const Eigen::Vector6d& twist)
{
    bool success = setFloatingBasePose(pose);
    success = update() && success;
    success = setFloatingBaseTwist(twist);
    return success;
}

bool XBot::ModelInterface::setFloatingBaseState(XBot::ImuSensor::ConstPtr imu)
{
    Eigen::Matrix3d fb_R_imu;
    getFloatingBaseLink(_floating_base_link);
    bool success = getOrientation(imu->getSensorName(), _floating_base_link,  fb_R_imu);
    
    Eigen::Affine3d fb_pose;
    Eigen::Vector6d fb_twist;
    
    getFloatingBasePose(fb_pose);
    getFloatingBaseTwist(fb_twist);
    
    Eigen::Matrix3d w_R_imu;
    Eigen::Vector3d imu_ang_vel;
    imu->getOrientation(w_R_imu);
    imu->getAngularVelocity(imu_ang_vel);
    
    fb_pose.linear() = w_R_imu*fb_R_imu.transpose();
    fb_twist.tail<3>() = w_R_imu * imu_ang_vel;
    
    return setFloatingBaseState(fb_pose, fb_twist);
}

bool XBot::ModelInterface::setFloatingBaseState(const KDL::Frame& pose, const KDL::Twist& twist)
{
    bool success = setFloatingBasePose(pose);
    success = update() && success;
    success = setFloatingBaseTwist(twist);
    return success;
}






























