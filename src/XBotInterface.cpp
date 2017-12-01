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

#include <XBotInterface/XBotInterface.h>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;


XBot::XBotInterface::XBotInterface()
{

}

XBot::XBotInterface::XBotInterface(const XBot::XBotInterface &other):
    _joint_num(other._joint_num),
    _ordered_joint_name(other._ordered_joint_name),
    _ordered_joint_id(other._ordered_joint_id),
    _XBotModel(other._XBotModel),
    _urdf_string(other._urdf_string),
    _srdf_string(other._srdf_string),
    _joint_id_to_eigen_id(other._joint_id_to_eigen_id),
    _joint_name_to_eigen_id(other._joint_name_to_eigen_id),
    _joint_map_config(other._joint_map_config),
    _urdf_path(other._urdf_path),
    _srdf_path(other._srdf_path),
    _path_to_cfg(other._path_to_cfg),
    _ordered_chain_names(other._ordered_chain_names)
{

    for (const auto & chain_name_ptr_pair : other._chain_map) {

        const std::string &chain_name = chain_name_ptr_pair.first;
        const XBot::KinematicChain::Ptr &other_chainptr = chain_name_ptr_pair.second;

        XBot::KinematicChain::Ptr chainptr = std::make_shared<XBot::KinematicChain>();
        *chainptr = *other_chainptr;

        _chain_map[chain_name] = chainptr;

    }

    for( const auto& j : other._ordered_joint_vector ){

        Joint::Ptr jptr = std::make_shared<Joint>();
        *jptr = *j;

        _ordered_joint_vector.push_back(jptr);
    }

    for( const auto& j : other._joint_vector ){

        Joint::Ptr jptr = std::make_shared<Joint>();
        *jptr = *j;

        _joint_vector.push_back(jptr);
    }

    for( const auto& ft_pair : other._ft_map ){

        ForceTorqueSensor::Ptr ftptr = std::make_shared<ForceTorqueSensor>();
        *ftptr = *ft_pair.second;

        _ft_map[ft_pair.first] = ftptr;

    }

    for( const auto& imu_pair : other._imu_map ){

        ImuSensor::Ptr imuptr = std::make_shared<ImuSensor>();
        *imuptr = *imu_pair.second;

        _imu_map[imu_pair.first] = imuptr;

    }

}

int XBot::XBotInterface::getJointNum() const
{
    return _joint_num;
}


const std::vector< std::string >& XBot::XBotInterface::getModelOrderedChainName() const
{
    return _ordered_chain_names;
}

bool XBot::XBotInterface::hasChain(const std::string& chain_name) const
{
    return _chain_map.count(chain_name) == 1;
}


bool XBot::XBotInterface::computeAbsolutePath (  const std::string& input_path,
                                                 const std::string& middle_path,
                                                 std::string& absolute_path)
{
    // if not an absolute path
    if(!(input_path.at(0) == '/')) {
        // if you are working with the Robotology Superbuild
        const char* env_p = std::getenv("ROBOTOLOGY_ROOT");
        // check the env, otherwise error
        if(env_p) {
            std::string current_path(env_p);
            // default relative path when working with the superbuild
            current_path += middle_path;
            current_path += input_path;
            absolute_path = current_path;
            return true;
        }
        else {
            std::cerr << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << std::endl;
            return false;
        }
    }
    // already an absolute path
    absolute_path = input_path;
    return true;
}

bool XBot::XBotInterface::parseYAML ( const std::string& path_to_cfg )
{
    std::ifstream fin(path_to_cfg);
    if (fin.fail()) {
        Logger::error() << "in " << __func__ << "! Can NOT open " << path_to_cfg << "!" << Logger::endl();
        return false;
    }

    YAML::Node root_cfg = YAML::LoadFile(path_to_cfg);
    YAML::Node x_bot_interface;
    if(root_cfg["XBotInterface"]) {
        x_bot_interface = root_cfg["XBotInterface"];
    }
    else {
        Logger::error() << "ERROR in " << __func__ << " : YAML file  " << path_to_cfg << "  does not contain XBotInterface mandatory node!!" << Logger::endl();
        return false;
    }

    // check the urdf_filename
    if(x_bot_interface["urdf_path"]) {
        computeAbsolutePath(x_bot_interface["urdf_path"].as<std::string>(),
                            "/",
                            _urdf_path);
    }
    else {
        Logger::error() << "ERROR in " << __func__ << " : XBotInterface node of  " << path_to_cfg << "  does not contain urdf_path mandatory node!!" << Logger::endl();
        return false;
    }

    // check the srdf_filename
    if(x_bot_interface["srdf_path"]) {
        computeAbsolutePath(x_bot_interface["srdf_path"].as<std::string>(),
                            "/",
                            _srdf_path);
    }
    else {
        Logger::error() << "ERROR in " << __func__ << " : XBotInterface node of  " << path_to_cfg << "  does not contain srdf_path mandatory node!!" << Logger::endl();
        return false;
    }

    // check joint_map_config
    if(x_bot_interface["joint_map_path"]) {
        computeAbsolutePath(x_bot_interface["joint_map_path"].as<std::string>(),
                            "/",
                            _joint_map_config);
    }
    else {
        Logger::error() << "ERROR in " << __func__ << " : XBotInterface node of  " << path_to_cfg << "  does not contain joint_map_path mandatory node!!" << Logger::endl();
        return false;
    }

}

const std::string& XBot::XBotInterface::getPathToConfig() const
{
    return _path_to_cfg;
}

bool XBot::XBotInterface::init(const std::string &path_to_cfg, AnyMapConstPtr any_map)
{
    // store path to config
    _path_to_cfg = path_to_cfg;
    // parse the YAML file to initialize internal variables
    parseYAML(path_to_cfg);
    // initialize the model
    if (!_XBotModel.init(_urdf_path, _srdf_path, _joint_map_config)) {
        Logger::error() << "model initialization failed, please check the urdf_path, srdf_path and joint_map_path in your YAML config file" << Logger::endl();
        return false;
    }
    // generate the robot
    _XBotModel.generate_robot();

    // construct class
    _joint_num = _XBotModel.get_joint_num();
    _urdf_string = _XBotModel.get_urdf_string();
    _srdf_string = _XBotModel.get_srdf_string();
    _XBotModel.get_enabled_joint_ids(_ordered_joint_id);
    _XBotModel.get_enabled_joint_names(_ordered_joint_name);
    _ordered_chain_names = _XBotModel.get_ordered_chain_names();

    // create dynamically the Kinematic Chains, FT and IMU
    for (const std::string & chain_name : _XBotModel.get_chain_names()) {
        XBot::KinematicChain::Ptr actual_chain = std::make_shared<KinematicChain>(chain_name,
                                                                    _XBotModel);
        _chain_map[chain_name] = actual_chain;

        const std::map< std::string, ForceTorqueSensor::Ptr >& ft_map = _chain_map.at(chain_name)->getForceTorqueInternal();
        _ft_map.insert(ft_map.begin(),
                       ft_map.end());

        const std::map< std::string, ImuSensor::Ptr >& imu_map = _chain_map.at(chain_name)->getImuInternal();
        _imu_map.insert(imu_map.begin(),
                       imu_map.end());

        for( int i = 0; i < actual_chain->getJointNum(); i++){
            _joint_vector.push_back(actual_chain->getJointInternal(i));
        }

    }

    // hands
    for( const auto& hand_pair : _XBotModel.get_hands() ) {
        _hand_map[hand_pair.first] =  std::make_shared<XBot::Hand>(hand_pair.second, hand_pair.first);
        _hand_id_map[hand_pair.second] =  _hand_map.at(hand_pair.first);
    }

    // NOTE if you have disabled joint, the URDF should be updated in order to have compatibility btw robot and model
    if (_XBotModel.getDisabledJoints().size() > 0) {
         Logger::warning() << "in " << __func__ << " : disabled joint detected in the specified SRDF. URDF must be updated accordingly" << Logger::endl();
    }

    // call virtual init_internal
    bool success = init_internal(path_to_cfg, any_map);

    // after subclasses have done their work inside init_internal, compute joint number
    _joint_num = 0;
    for( const auto& c : _chain_map ){
        _joint_num += c.second->getJointNum();
    }

    // fill the joint id to eigen id
    _ordered_joint_vector.clear();
    _joint_name_to_eigen_id.clear();
    _ordered_joint_id.clear();
    _ordered_joint_name.clear();
    int eigen_id = 0;

    for( const std::string& chain_name : getModelOrderedChainName() ) {
        for( int i = 0; i < _chain_map.at(chain_name)->getJointNum(); i++) {
            _joint_id_to_eigen_id[_chain_map.at(chain_name)->getJointId(i)] = eigen_id;
            _joint_name_to_eigen_id[_chain_map.at(chain_name)->getJointName(i)] = eigen_id;
            _ordered_joint_vector.push_back(_chain_map.at(chain_name)->getJointInternal(i));
            _ordered_joint_id.push_back(_chain_map.at(chain_name)->getJointId(i));
            _ordered_joint_name.push_back(_chain_map.at(chain_name)->getJointName(i));
            eigen_id++;
        }
    }

    _joint_vector = _ordered_joint_vector;

    // fill FT & IMU id maps
    for( const auto& pair : getForceTorqueInternal() ){
        _ft_id_map[pair.second->getSensorId()] = pair.second;
    }

    for( const auto pair : getImuInternal() ){
        _imu_id_map[pair.second->getSensorId()] = pair.second;
    }

    post_init();

    return success;

}

bool XBot::XBotInterface::getDofIndex ( const std::string& chain_name, std::vector< int >& ids ) const
{
    auto it = _chain_map.find(chain_name);
    if( it != _chain_map.end() ){

        const KinematicChain& chain = *(it->second);
        ids.resize(chain.getJointNum());
        for(int i=0; i< chain.getJointNum(); i++){
            ids[i] = _joint_id_to_eigen_id.at(chain.getJointId(i));
        }

    }
    else{
     std::cerr << "ERROR in " << __func__ << ": requested chain " << chain_name << " is not defined!" << std::endl;
     return false;
    }

    return true;

}

int XBot::XBotInterface::getDofIndex(const std::string& joint_name) const
{
    auto it = _joint_name_to_eigen_id.find(joint_name);
    if( it != _joint_name_to_eigen_id.end() ){
        return it->second;
    }
    else{
        std::cerr << "ERROR in " << __func__ << ": joint " << joint_name << " NOT defined!!!" << std::endl;
        return -1;
    }

}

int XBot::XBotInterface::getDofIndex(int joint_id) const
{
    auto it = _joint_id_to_eigen_id.find(joint_id);
    if( it != _joint_id_to_eigen_id.end() ){
        return it->second;
    }
    else{
        std::cerr << "ERROR in " << __func__ << ": joint " << joint_id << " NOT defined!!!" << std::endl;
        return -1;
    }
}



int XBot::XBotInterface::legs() const
{
    return _XBotModel.get_legs_chain().size();
}


int XBot::XBotInterface::arms() const
{
    return _XBotModel.get_arms_chain().size();
}


bool XBot::XBotInterface::hasJoint(const std::string &joint_name) const
{
    return std::find(_ordered_joint_name.begin(), _ordered_joint_name.end(), joint_name) !=  _ordered_joint_name.end();
}

bool XBot::XBotInterface::hasJoint(int joint_id) const
{
    return std::find(_ordered_joint_id.begin(), _ordered_joint_id.end(), joint_id) !=  _ordered_joint_id.end();
}


XBot::Joint::ConstPtr XBot::XBotInterface::getJointByName(const std::string& joint_name) const
{
    auto it = _joint_name_to_eigen_id.find(joint_name);

    if( it != _joint_name_to_eigen_id.end() ){
        return _ordered_joint_vector[it->second];
    }

    for( const auto& c : _chain_map){
        const XBot::KinematicChain& chain = *c.second;
        if(chain.hasJoint(joint_name)) return chain.getJointByName(joint_name);
    }
    std::cerr << "ERROR in " << __func__ << ". Joint " << joint_name << " is NOT defined!" << std::endl;
    return XBot::Joint::ConstPtr();
}

XBot::Joint::ConstPtr XBot::XBotInterface::getJointByDofIndex(int idx) const
{
    if( idx < getJointNum() ){
        return _ordered_joint_vector[idx];
    }

    std::cerr << "ERROR in " << __func__ << ". Less than " << idx+1 << " joints are defined!" << std::endl;
    return XBot::Joint::ConstPtr();
}

XBot::Joint::ConstPtr XBot::XBotInterface::getJointByID(int joint_id) const
{
    auto it = _joint_id_to_eigen_id.find(joint_id);

    if( it != _joint_id_to_eigen_id.end() ){
        return _ordered_joint_vector[it->second];
    }

    for( const auto& c : _chain_map){
        const XBot::KinematicChain& chain = *c.second;
        if(chain.hasJoint(joint_id)) return chain.getJointById(joint_id);
    }

    std::cerr << "ERROR in " << __func__ << ". Joint " << joint_id << " is NOT defined!" << std::endl;
    return XBot::Joint::ConstPtr();
}




bool XBot::XBotInterface::getJointPosition(Eigen::VectorXd &q) const
{
    q.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        q(i) = _ordered_joint_vector[i]->getJointPosition();
    }

    return true;
}

bool XBot::XBotInterface::getJointPosition(JointNameMap &q) const
{
//     q.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            q[chain.getJointName(j)] = chain.getJointPosition(j);
        }

    }
}

bool XBot::XBotInterface::getJointPosition(JointIdMap &q) const
{
//     q.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            q[chain.getJointId(j)] = chain.getJointPosition(j);
        }

    }
}

bool XBot::XBotInterface::getPositionReference(JointNameMap &q) const
{
//     q.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            q[chain.getJointName(j)] = chain.getPositionReference(j);
        }

    }
}

bool XBot::XBotInterface::getPositionReference(JointIdMap &q) const
{
//     q.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            q[chain.getJointId(j)] = chain.getPositionReference(j);
        }

    }
}

bool XBot::XBotInterface::getPositionReference(Eigen::VectorXd &q) const
{
    q.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        q(i) = _ordered_joint_vector[i]->getPositionReference();
    }

    return true;
}

bool XBot::XBotInterface::getDamping(JointNameMap &D) const
{
//     D.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            D[chain.getJointName(j)] = chain.getDamping(j);
        }

    }
}

bool XBot::XBotInterface::getJointEffort(JointNameMap &tau) const
{
//     tau.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            tau[chain.getJointName(j)] = chain.getJointEffort(j);
        }

    }
}

bool XBot::XBotInterface::getEffortReference(JointNameMap &tau) const
{
//     tau.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            tau[chain.getJointName(j)] = chain.getEffortReference(j);
        }

    }
}

bool XBot::XBotInterface::getJointVelocity(JointNameMap &qdot) const
{
//     qdot.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            qdot[chain.getJointName(j)] = chain.getJointVelocity(j);
        }

    }
}

bool XBot::XBotInterface::getJointAcceleration(JointNameMap& qddot) const
{
    for(int i = 0; i < _joint_num; i++){
        const Joint::Ptr& joint = _ordered_joint_vector[i];
        qddot[joint->getJointName()] = joint->getJointAcceleration();
    }

//     for (const auto & chain_name_ptr_pair : _chain_map) {
//
//         const KinematicChain &chain = *chain_name_ptr_pair.second;
//
//         for (int j = 0; j < chain.getJointNum(); j++) {
//             qddot[chain.getJointName(j)] = chain.getJointAcceleration(j);
//         }
//
//     }
}


bool XBot::XBotInterface::getMotorPosition(JointNameMap &q) const
{
//     q.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            q[chain.getJointName(j)] = chain.getMotorPosition(j);
        }

    }
}

bool XBot::XBotInterface::getStiffness(JointNameMap &K) const
{
//     K.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            K[chain.getJointName(j)] = chain.getStiffness(j);
        }

    }
}

bool XBot::XBotInterface::getTemperature(JointNameMap &temp) const
{
//     temp.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            temp[chain.getJointName(j)] = chain.getTemperature(j);
        }

    }
}

bool XBot::XBotInterface::getVelocityReference(JointNameMap &qdot) const
{
//     qdot.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            qdot[chain.getJointName(j)] = chain.getVelocityReference(j);
        }

    }
}

bool XBot::XBotInterface::getMotorVelocity(JointNameMap &qdot) const
{
//     qdot.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            qdot[chain.getJointName(j)] = chain.getMotorVelocity(j);
        }

    }
}



bool XBot::XBotInterface::getDamping(JointIdMap &D) const
{
//     D.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            D[chain.getJointId(j)] = chain.getDamping(j);
        }

    }
}

bool XBot::XBotInterface::getJointEffort(JointIdMap &tau) const
{
//     tau.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            tau[chain.getJointId(j)] = chain.getJointEffort(j);
        }

    }
}

bool XBot::XBotInterface::getEffortReference(JointIdMap &tau) const
{
//     tau.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            tau[chain.getJointId(j)] = chain.getEffortReference(j);
        }

    }
}


bool XBot::XBotInterface::getVelocityReference(JointIdMap &qdot) const
{
//     qdot.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            qdot[chain.getJointId(j)] = chain.getVelocityReference(j);
        }

    }
}


bool XBot::XBotInterface::getJointVelocity(JointIdMap &qdot) const
{
//     qdot.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            qdot[chain.getJointId(j)] = chain.getJointVelocity(j);
        }

    }
}

bool XBot::XBotInterface::getJointAcceleration(JointIdMap& qddot) const
{

    for(int i = 0; i < _joint_num; i++){
        const Joint::Ptr& joint = _ordered_joint_vector[i];
        qddot[joint->getJointId()] = joint->getJointAcceleration();

    }

//     for (const auto & chain_name_ptr_pair : _chain_map) {
//
//         const KinematicChain &chain = *chain_name_ptr_pair.second;
//
//         for (int j = 0; j < chain.getJointNum(); j++) {
//             qddot[chain.getJointId(j)] = chain.getJointAcceleration(j);
//         }
//
//     }
}


bool XBot::XBotInterface::getMotorPosition(JointIdMap &q) const
{
//     q.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            q[chain.getJointId(j)] = chain.getMotorPosition(j);
        }

    }
}

bool XBot::XBotInterface::getMotorVelocity(JointIdMap &qdot) const
{
//     qdot.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            qdot[chain.getJointId(j)] = chain.getMotorVelocity(j);
        }

    }
}

bool XBot::XBotInterface::getStiffness(JointIdMap &K) const
{
//     K.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            K[chain.getJointId(j)] = chain.getStiffness(j);
        }

    }
}

bool XBot::XBotInterface::getTemperature(JointIdMap &temp) const
{
//     temp.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            temp[chain.getJointId(j)] = chain.getTemperature(j);
        }

    }
}




bool XBot::XBotInterface::getDamping(Eigen::VectorXd &D) const
{
    D.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        D(i) = _ordered_joint_vector[i]->getDamping();
    }

    return true;
}

bool XBot::XBotInterface::getJointEffort(Eigen::VectorXd &tau) const
{
    tau.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        tau(i) = _ordered_joint_vector[i]->getJointEffort();
    }

    return true;
}

bool XBot::XBotInterface::getEffortReference(Eigen::VectorXd &tau) const
{
    tau.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        tau(i) = _ordered_joint_vector[i]->getEffortReference();
    }

    return true;
}

bool XBot::XBotInterface::getJointVelocity(Eigen::VectorXd &qdot) const
{
    qdot.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        qdot(i) = _ordered_joint_vector[i]->getJointVelocity();
    }

    return true;
}

bool XBot::XBotInterface::getJointAcceleration(Eigen::VectorXd& qddot) const
{

    qddot.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        qddot(i) = _ordered_joint_vector[i]->getJointAcceleration();
    }

    return true;
}


bool XBot::XBotInterface::getMotorPosition(Eigen::VectorXd &q) const
{
    q.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        q(i) = _ordered_joint_vector[i]->getMotorPosition();
    }

    return true;
}

bool XBot::XBotInterface::getMotorVelocity(Eigen::VectorXd &qdot) const
{
    qdot.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        qdot(i) = _ordered_joint_vector[i]->getMotorVelocity();
    }

    return true;
}

bool XBot::XBotInterface::getStiffness(Eigen::VectorXd &K) const
{
    K.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        K(i) = _ordered_joint_vector[i]->getStiffness();
    }

    return true;
}

bool XBot::XBotInterface::getTemperature(Eigen::VectorXd &temp) const
{
    temp.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        temp(i) = _ordered_joint_vector[i]->getTemperature();
    }

    return true;
}

bool XBot::XBotInterface::getVelocityReference(Eigen::VectorXd &qdot) const
{
    qdot.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        qdot(i) = _ordered_joint_vector[i]->getVelocityReference();
    }

    return true;
}


bool XBot::XBotInterface::setJointPosition(const JointNameMap &q)
{
    bool success = false;

    for(const auto& pair : q){
        auto it = _joint_name_to_eigen_id.find(pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setJointPosition(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setMotorPosition(const JointNameMap &q)
{
    bool success = false;

    for(const auto& pair : q){
        auto it = _joint_name_to_eigen_id.find(pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setMotorPosition(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setPositionReference(const JointNameMap &q)
{
    bool success = false;

    for(const auto& pair : q){
        auto it = _joint_name_to_eigen_id.find(pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setPositionReference(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setJointEffort(const JointNameMap &tau)
{
    bool success = false;

    for(const auto& pair : tau){
        auto it = _joint_name_to_eigen_id.find(pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setJointEffort(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setEffortReference(const JointNameMap &tau)
{
    bool success = false;

    for(const auto& pair : tau){
        auto it = _joint_name_to_eigen_id.find(pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setEffortReference(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setJointVelocity(const JointNameMap &qdot)
{
    bool success = false;

    for(const auto& pair : qdot){
        auto it = _joint_name_to_eigen_id.find(pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setJointVelocity(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setJointAcceleration(const JointNameMap& qddot)
{
    bool success = false;

    for(const auto& pair : qddot){
        auto it = _joint_name_to_eigen_id.find(pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setJointAcceleration(pair.second);
            success = true;
        }
    }

    return success;
}


bool XBot::XBotInterface::setMotorVelocity(const JointNameMap &qdot)
{
    bool success = false;

    for(const auto& pair : qdot){
        auto it = _joint_name_to_eigen_id.find(pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setMotorVelocity(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setVelocityReference(const JointNameMap &qdot)
{
    bool success = false;

    for(const auto& pair : qdot){
        auto it = _joint_name_to_eigen_id.find(pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setVelocityReference(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setTemperature(const JointNameMap &temp)
{
    bool success = false;

    for(const auto& pair : temp){
        auto it = _joint_name_to_eigen_id.find(pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setVelocityReference(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setStiffness(const JointNameMap &K)
{
    bool success = false;

    for(const auto& pair : K){
        auto it = _joint_name_to_eigen_id.find(pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setStiffness(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setDamping ( const Eigen::VectorXd& D )
{
    if (D.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : argument has wrong size " << D.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->setDamping(D(i));
    }

    return true;
}

bool XBot::XBotInterface::setJointEffort ( const Eigen::VectorXd& tau )
{
     if (tau.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : tau has wrong size " << tau.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->setJointEffort(tau(i));
    }

    return true;
}

bool XBot::XBotInterface::setEffortReference ( const Eigen::VectorXd& tau )
{
     if (tau.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : tau has wrong size " << tau.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->setEffortReference(tau(i));
    }

    return true;
}

bool XBot::XBotInterface::setJointPosition ( const Eigen::VectorXd& q )
{
     if (q.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : q has wrong size " << q.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->setJointPosition(q(i));
    }

    return true;
}

bool XBot::XBotInterface::setJointVelocity ( const Eigen::VectorXd& qdot )
{
     if (qdot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qdot has wrong size " << qdot.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->setJointVelocity(qdot(i));
    }

    return true;
}

bool XBot::XBotInterface::setJointAcceleration(const Eigen::VectorXd& qddot)
{
     if (qddot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qddot has wrong size " << qddot.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->setJointAcceleration(qddot(i));
    }

    return true;
}


bool XBot::XBotInterface::setMotorPosition( const Eigen::VectorXd& q )
{
     if (q.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : q has wrong size " << q.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->setMotorPosition(q(i));
    }

    return true;
}

bool XBot::XBotInterface::setMotorVelocity( const Eigen::VectorXd& qdot )
{
     if (qdot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qdot has wrong size " << qdot.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->setMotorVelocity(qdot(i));
    }

    return true;
}

bool XBot::XBotInterface::setPositionReference ( const Eigen::VectorXd& q )
{
     if (q.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : q has wrong size " << q.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->setPositionReference(q(i));
    }

    return true;
}

bool XBot::XBotInterface::setStiffness ( const Eigen::VectorXd& K )
{
     if (K.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : K has wrong size " << K.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->setStiffness(K(i));
    }

    return true;
}

bool XBot::XBotInterface::setTemperature ( const Eigen::VectorXd& temp )
{
     if (temp.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : temp has wrong size " << temp.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->setTemperature(temp(i));
    }

    return true;
}

bool XBot::XBotInterface::setVelocityReference ( const Eigen::VectorXd& qdot )
{
     if (qdot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qdot has wrong size " << qdot.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->setVelocityReference(qdot(i));
    }

    return true;
}





bool XBot::XBotInterface::setDamping(const JointNameMap &D)
{
    bool success = false;

    for(const auto& pair : D){
        auto it = _joint_name_to_eigen_id.find(pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setDamping(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setDamping ( const JointIdMap& D )
{
    bool success = false;

    for(const auto& pair : D){
        auto it = _joint_id_to_eigen_id.find(pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setDamping(pair.second);
            success = true;
        }
    }

    return success;
}


bool XBot::XBotInterface::setJointEffort ( const JointIdMap& tau )
{
    bool success = false;

    for(const auto& pair : tau){
        auto it = _joint_id_to_eigen_id.find(pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setJointEffort(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setEffortReference ( const JointIdMap& tau )
{
    bool success = false;

    for(const auto& pair : tau){
        auto it = _joint_id_to_eigen_id.find(pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setEffortReference(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setJointPosition ( const JointIdMap& q ) // TBD profile vs unordered maps
{
    bool success = false;

    for(const auto& pair : q){
        auto it = _joint_id_to_eigen_id.find(pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setJointPosition(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setJointVelocity ( const JointIdMap& qdot )
{
    bool success = false;

    for(const auto& pair : qdot){
        auto it = _joint_id_to_eigen_id.find(pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setJointVelocity(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setJointAcceleration(const JointIdMap& qddot)
{
    bool success = false;

    for(const auto& pair : qddot){
        auto it = _joint_id_to_eigen_id.find(pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setJointAcceleration(pair.second);
            success = true;
        }
    }

    return success;
}


bool XBot::XBotInterface::setMotorPosition( const JointIdMap& q )
{
    bool success = false;

    for(const auto& pair : q){
        auto it = _joint_id_to_eigen_id.find(pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setMotorPosition(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setMotorVelocity( const JointIdMap& qdot )
{
    bool success = false;

    for(const auto& pair : qdot){
        auto it = _joint_id_to_eigen_id.find(pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setMotorVelocity(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setPositionReference ( const JointIdMap& q )
{
    bool success = false;

    for(const auto& pair : q ){
        auto it = _joint_id_to_eigen_id.find(pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setPositionReference(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setStiffness ( const JointIdMap& K )
{
    bool success = false;

    for(const auto& pair : K){
        auto it = _joint_id_to_eigen_id.find(pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setStiffness(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setTemperature ( const JointIdMap& temp )
{
    bool success = false;

    for(const auto& pair : temp){
        auto it = _joint_id_to_eigen_id.find(pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setTemperature(pair.second);
            success = true;
        }
    }

    return success;
}

bool XBot::XBotInterface::setVelocityReference ( const JointIdMap& qdot )
{
    bool success = false;

    for(const auto& pair : qdot){
        auto it = _joint_id_to_eigen_id.find(pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            _ordered_joint_vector[it->second]->setVelocityReference(pair.second);
            success = true;
        }
    }

    return success;
}



std::vector< std::string > XBot::XBotInterface::getChainNames() const
{
    std::vector<std::string> chains;
    for(const auto& pair : _chain_map){
        chains.push_back(pair.first);
    }
    return chains;
}

const std::map< std::string, XBot::KinematicChain::Ptr >& XBot::XBotInterface::getChainMap() const
{
    return _chain_map;
}


const srdf_advr::Model& XBot::XBotInterface::getSrdf() const
{
    return _XBotModel;
}

const std::string& XBot::XBotInterface::getSrdfString() const
{
    return _srdf_string;
}

const urdf::ModelInterface& XBot::XBotInterface::getUrdf() const
{
    return *(_XBotModel.get_urdf_model());
}

const std::string& XBot::XBotInterface::getUrdfString() const
{
    return _urdf_string;
}



XBot::XBotInterface::~XBotInterface()
{
}

XBot::XBotInterface &XBot::XBotInterface::operator=(const XBot::XBotInterface &rhs)
{
    XBotInterface tmp(rhs);
    std::swap(_joint_num, tmp._joint_num);
    std::swap(_XBotModel, tmp._XBotModel);
    std::swap(_urdf_string, tmp._urdf_string);
    std::swap(_srdf_string, tmp._srdf_string);
    std::swap(_urdf_path, tmp._urdf_path);
    std::swap(_srdf_path, tmp._srdf_path);
    std::swap(_ordered_joint_name, tmp._ordered_joint_name);
    std::swap(_ordered_joint_id, tmp._ordered_joint_id);
    std::swap(_chain_map, tmp._chain_map);
    std::swap(_joint_id_to_eigen_id, tmp._joint_id_to_eigen_id);
    std::swap(_joint_name_to_eigen_id, tmp._joint_name_to_eigen_id);
    std::swap(_path_to_cfg, tmp._path_to_cfg);
    std::swap(_ordered_joint_vector, tmp._ordered_joint_vector);
    std::swap(_joint_map_config, tmp._joint_map_config);
    std::swap(_ft_map, tmp._ft_map);
    std::swap(_imu_map, tmp._imu_map);
    std::swap(_ordered_chain_names, tmp._ordered_chain_names);
    std::swap(_joint_vector, tmp._joint_vector);

}

bool XBot::XBotInterface::checkEffortLimits(const Eigen::VectorXd& tau) const
{
    int idx = 0;
    for( const std::string& s : getModelOrderedChainName() ){
        const KinematicChain& chain = *_chain_map.at(s);
        if( !chain.checkEffortLimits(tau.segment(idx, chain.getJointNum())) ){
            return false;
        }
        idx += chain.getJointNum();
    }

    return true;
}

bool XBot::XBotInterface::checkEffortLimits(const Eigen::VectorXd& tau,
                                             std::vector< std::string >& violating_joints) const
{
    bool success = true;

    int idx = 0;
    for( const std::string& s : getModelOrderedChainName() ){
        const KinematicChain& chain = *_chain_map.at(s);
        success = chain.checkEffortLimits(tau.segment(idx, chain.getJointNum()), violating_joints) && success;
        idx += chain.getJointNum();
    }

    return success;
}

bool XBot::XBotInterface::checkJointLimits(const Eigen::VectorXd& q) const
{
    int idx = 0;
    for( const std::string& s : getModelOrderedChainName() ){
        const KinematicChain& chain = *_chain_map.at(s);
        if( !chain.checkJointLimits(q.segment(idx, chain.getJointNum())) ){
            return false;
        }
        idx += chain.getJointNum();
    }

    return true;
}

bool XBot::XBotInterface::checkJointLimits(const Eigen::VectorXd& q,
                                            std::vector< std::string >& violating_joints) const
{
    bool success = true;

    int idx = 0;
    for( const std::string& s : getModelOrderedChainName() ){
        const KinematicChain& chain = *_chain_map.at(s);
        success = chain.checkJointLimits(q.segment(idx, chain.getJointNum()), violating_joints) && success;
        idx += chain.getJointNum();
    }

    return success;
}

bool XBot::XBotInterface::checkVelocityLimits(const Eigen::VectorXd& qdot) const
{
    int idx = 0;
    for( const std::string& s : getModelOrderedChainName() ){
        const KinematicChain& chain = *_chain_map.at(s);
        if( !chain.checkVelocityLimits(qdot.segment(idx, chain.getJointNum())) ){
            return false;
        }
        idx += chain.getJointNum();
    }

    return true;
}

bool XBot::XBotInterface::checkVelocityLimits(const Eigen::VectorXd& qdot,
                                               std::vector< std::string >& violating_joints) const
{
    bool success = true;

    int idx = 0;
    for( const std::string& s : getModelOrderedChainName() ){
        const KinematicChain& chain = *_chain_map.at(s);
        success = chain.checkVelocityLimits(qdot.segment(idx, chain.getJointNum()), violating_joints) && success;
        idx += chain.getJointNum();
    }

    return success;
}

void XBot::XBotInterface::getEffortLimits(Eigen::VectorXd& tau_max) const
{
    tau_max.resize(_joint_num);

    int idx = 0;
    for( const std::string& c : getModelOrderedChainName() ){

        const KinematicChain& chain = *_chain_map.at(c);

        for(int i = 0; i < chain.getJointNum(); i++){
            chain.getEffortLimits(i, tau_max(idx));
            idx++;
        }

    }
}

void XBot::XBotInterface::getJointLimits(Eigen::VectorXd& q_min, Eigen::VectorXd& q_max) const
{
    q_min.resize(_joint_num);
    q_max.resize(_joint_num);

    int idx = 0;
    for( const std::string& c : getModelOrderedChainName() ){

        const KinematicChain& chain = *_chain_map.at(c);

        for(int i = 0; i < chain.getJointNum(); i++){
            chain.getJointLimits(i, q_min(idx), q_max(idx));
            idx++;
        }

    }
}

void XBot::XBotInterface::getVelocityLimits(Eigen::VectorXd& qdot_max) const
{
    qdot_max.resize(_joint_num);

    int idx = 0;
    for( const std::string& c : getModelOrderedChainName() ){

        const KinematicChain& chain = *_chain_map.at(c);

        for(int i = 0; i < chain.getJointNum(); i++){
            chain.getVelocityLimits(i, qdot_max(idx));
            idx++;
        }

    }
}



const std::vector< std::string > &XBot::XBotInterface::getEnabledJointNames() const
{
    return _ordered_joint_name;
}

std::ostream& XBot::operator<< ( std::ostream& os, const XBot::XBotInterface& robot )
{
    os << "Robot name: " << std::endl;
    for( const auto& c : robot.getChainMap() ) {
        os << (*c.second) << std::endl;
    }
    return os;
}

XBot::ForceTorqueSensor::ConstPtr XBot::XBotInterface::getForceTorque(const std::string& parent_link_name) const
{

    for( const auto& ft_pair : _ft_map ) {

        ForceTorqueSensor::ConstPtr ft = ft_pair.second;

        if(ft->getParentLinkName() == parent_link_name){
            return ft;
        }

    }

   return ForceTorqueSensor::ConstPtr();
}

std::map< std::string, XBot::ForceTorqueSensor::ConstPtr > XBot::XBotInterface::getForceTorque()
{
   std::map< std::string, XBot::ForceTorqueSensor::ConstPtr > ft_map;

   for( const auto& ft_ptr_pair : _ft_map ){

        ft_map[ft_ptr_pair.first] = ft_ptr_pair.second;
   }

   return ft_map;
}

const std::map< std::string, XBot::ForceTorqueSensor::Ptr >& XBot::XBotInterface::getForceTorqueInternal() const
{
    return _ft_map;
}

XBot::ImuSensor::ConstPtr XBot::XBotInterface::getImu(const std::string& parent_link_name) const
{
    for( const auto& imu_pair : _imu_map ) {

        ImuSensor::ConstPtr imu = imu_pair.second;

        if(imu->getParentLinkName() == parent_link_name){
            return imu;
        }

    }

   return ImuSensor::ConstPtr();
}

std::map< std::string, XBot::ImuSensor::ConstPtr > XBot::XBotInterface::getImu()
{
   std::map< std::string, XBot::ImuSensor::ConstPtr > imu_map;

   for( const auto& imu_ptr_pair : _imu_map ){

        imu_map[imu_ptr_pair.first] = imu_ptr_pair.second;
   }

   return imu_map;
}

const std::map< std::string, XBot::ImuSensor::Ptr >& XBot::XBotInterface::getImuInternal() const
{
    return _imu_map;
}



std::map< std::string, XBot::Hand::Ptr > XBot::XBotInterface::getHand()
{
    return _hand_map;
}

XBot::Hand::Ptr XBot::XBotInterface::getHand ( int hand_id ) const
{
    auto it = _hand_id_map.find(hand_id);

    if( it == _hand_id_map.end() ){
        std::cerr << "ERROR in " << __PRETTY_FUNCTION__ << ", HAND with given ID " << hand_id << " is NOT defined!\n" <<
        "Available HAND are:\n";

        for( const auto& pair : _hand_map ){
            std::cerr << "\tName: " << pair.first << ", ID: " << pair.second->getHandId() << "\n";
        }

        std::cerr << std::endl;

        return XBot::Hand::Ptr();
    }

    return it->second;
}




const std::vector<int>& XBot::XBotInterface::getEnabledJointId() const
{
    return _ordered_joint_id;
}

bool XBot::XBotInterface::getRobotState(const std::string& state_name,
                                        JointNameMap& q) const
{

    bool success = false;
    for( const auto& state : _XBotModel.getGroupStates() ){

        if( state.group_ == "chains" && state.name_ == state_name ){

            success = true;

            for( const auto& value_pair : state.joint_values_ ){

                const std::string& joint_name = value_pair.first;
                const std::vector<double>& joint_value = value_pair.second;

                if( joint_value.size() != 1 ){
                    std::cerr << "ERROR in " << __func__ << ": multi-dof joints not supported!" << std::endl;
                    return false;
                }

                if(!hasJoint(joint_name)){
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is NOT defined!" << std::endl;
                    return false;
                }

                q[joint_name] = joint_value[0];

            }

        }


    }

    if(!success){
        std::cerr << "ERROR in " << __func__ << ": required group state " << state_name << " is NOT defined as a group state for the whole robot. Check in the SRDF that " << state_name << " is defined for the chains group" << std::endl;
    }

    return success;
}

bool XBot::XBotInterface::getRobotState(const std::string& state_name,
                                        JointIdMap& q) const
{

    bool success = false;
    for( const auto& state : _XBotModel.getGroupStates() ){

        if( state.group_ == "chains" && state.name_ == state_name ){

            success = true;

            for( const auto& value_pair : state.joint_values_ ){

                const std::string& joint_name = value_pair.first;
                const std::vector<double>& joint_value = value_pair.second;

                if( joint_value.size() != 1 ){
                    std::cerr << "ERROR in " << __func__ << ": multi-dof joints not supported!" << std::endl;
                    return false;
                }

                if(!hasJoint(joint_name)){
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is NOT defined!" << std::endl;
                    return false;
                }

                int joint_id = getJointByName(joint_name)->getJointId();



                q[joint_id] = joint_value[0];

            }

        }

    }

    if(!success){
        std::cerr << "ERROR in " << __func__ << ": required group state " << state_name << " is NOT defined as a group state for the whole robot. Check in the SRDF that " << state_name << " is defined for the chains group" << std::endl;
    }

    return success;
}

bool XBot::XBotInterface::getRobotState(const std::string& state_name,
                                        Eigen::VectorXd& q) const
{

    q.setZero(getJointNum());

    bool success = false;
    for( const auto& state : _XBotModel.getGroupStates() ){

        if( state.group_ == "chains" && state.name_ == state_name ){

            success = true;

            for( const auto& value_pair : state.joint_values_ ){

                const std::string& joint_name = value_pair.first;
                const std::vector<double>& joint_value = value_pair.second;

                if( joint_value.size() != 1 ){
                    Logger::warning() << "in " << __func__ << ": multi-dof joints not supported!" << Logger::endl();
                    continue;
                }

                if(!hasJoint(joint_name)){
                    Logger::warning() << "in " << __func__ << "! Joint " << joint_name << " is NOT defined!" << Logger::endl();
                    continue;
                }

                int dof_index = getDofIndex(joint_name);
                q[dof_index] = joint_value[0];

            }

        }

    }

    if(!success){
        Logger::error() << "in " << __func__ << ": required group state " << state_name << " is NOT defined as a group state for the whole robot. Check in the SRDF that " << state_name << " is defined for the chains group" << Logger::endl();
    }

    return success;
}

bool XBot::XBotInterface::eigenToMap(const Eigen::VectorXd& vector, XBot::JointIdMap& id_map) const
{
    if( vector.size() != _joint_num ){
        std::cerr << "ERROR in " << __func__ << "! Input vector has " << vector.size() << "!=" << getJointNum() << " elements!" << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        const Joint& j = *_ordered_joint_vector[i];
        id_map[j.getJointId()] = vector(i);
    }

    return true;
}

bool XBot::XBotInterface::eigenToMap(const Eigen::VectorXd& vector, XBot::JointNameMap& name_map) const
{
    if( vector.size() != _joint_num ){
        std::cerr << "ERROR in " << __func__ << "! Input vector has " << vector.size() << "!=" << getJointNum() << " elements!" << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        const Joint& j = *_ordered_joint_vector[i];
        name_map[j.getJointName()] = vector(i);
    }

    return true;
}

bool XBot::XBotInterface::mapToEigen(const XBot::JointIdMap& map, Eigen::VectorXd& vector) const
{
    bool success = true;

    if( vector.size() != getJointNum() ){
        vector.setZero(getJointNum());
    }

    for(const auto& id_pair : map){
        auto it = _joint_id_to_eigen_id.find(id_pair.first);
        if( it != _joint_id_to_eigen_id.end() ){
            success = true;
            vector(it->second) = id_pair.second;
        }
    }

    return success;
}

bool XBot::XBotInterface::mapToEigen(const XBot::JointNameMap& map, Eigen::VectorXd& vector) const
{
    bool success = true;

    if( vector.size() != getJointNum() ){
        vector.setZero(getJointNum());
    }

    for(const auto& id_pair : map){
        auto it = _joint_name_to_eigen_id.find(id_pair.first);
        if( it != _joint_name_to_eigen_id.end() ){
            success = true;
            vector(it->second) = id_pair.second;
        }
    }

    return success;
}

void XBot::XBotInterface::print() const
{
    for( const auto& c : _chain_map ){
        c.second->print();
    }
}


void XBot::XBotInterface::printTracking() const
{
    for( const auto& c : _chain_map ){
        c.second->printTracking();
    }
}

const std::string& XBot::XBotInterface::getUrdfPath() const
{
    return _urdf_path;
}

const std::string& XBot::XBotInterface::getSrdfPath() const
{
    return _srdf_path;
}

bool XBot::XBotInterface::enforceEffortLimit(Eigen::VectorXd& tau) const
{
    if( tau.size() != getJointNum() ){
        std::cerr << "ERROR in " << __func__ << "! Provided vector has " << tau.size() << " elements != " << getJointNum() << " the size of the robot!" << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->enforceEffortLimit(tau(i));
    }

    return true;
}

bool XBot::XBotInterface::enforceJointLimits(Eigen::VectorXd& q) const
{
    if( q.size() != getJointNum() ){
        std::cerr << "ERROR in " << __func__ << "! Provided vector has " << q.size() << " elements != " << getJointNum() << " the size of the robot!" << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->enforceJointLimits(q(i));
    }

    return true;
}

bool XBot::XBotInterface::enforceJointLimits(XBot::JointNameMap& q) const
{
    for( auto& pair : q ){
        getJointByName(pair.first)->enforceJointLimits(pair.second);
    }

    return true;
}

bool XBot::XBotInterface::enforceJointLimits(XBot::JointIdMap& q) const
{
    for( auto& pair : q ){
        getJointByID(pair.first)->enforceJointLimits(pair.second);
    }

    return true;
}


bool XBot::XBotInterface::enforceVelocityLimit(Eigen::VectorXd& qdot) const
{
    if( qdot.size() != getJointNum() ){
        std::cerr << "ERROR in " << __func__ << "! Provided vector has " << qdot.size() << " elements != " << getJointNum() << " the size of the robot!" << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _ordered_joint_vector[i]->enforceVelocityLimit(qdot(i));
    }

    return true;
}



XBot::Joint::Ptr XBot::XBotInterface::getJointByNameInternal(const std::string& joint_name) const
{
    auto it = _joint_name_to_eigen_id.find(joint_name);

    if( it != _joint_name_to_eigen_id.end() ){
        return _ordered_joint_vector[it->second];
    }

    for( const auto& c : _chain_map){
        const XBot::KinematicChain& chain = *c.second;
        if(chain.hasJoint(joint_name)) return chain.getJointInternal(chain.getChainDofIndex(joint_name));
    }
    std::cerr << "ERROR in " << __func__ << ". Joint " << joint_name << " is NOT defined!" << std::endl;
    return XBot::Joint::Ptr();
}

XBot::Joint::Ptr XBot::XBotInterface::getJointByDofIndexInternal(int dof_index) const
{
    if(_ordered_joint_vector.size() > 0){
        if(_ordered_joint_vector.size() > dof_index){
            return _ordered_joint_vector[dof_index];
        }
        else{
            std::cerr << "ERROR in " << __func__ << "! Requesting joint with dof index " << dof_index << " which exceeds the dof number " << getJointNum() << "!" << std::endl;
            return Joint::Ptr();
        }
    }
    else{

        std::cerr << "ERROR in " << __func__ << "! Joint order unavailable!" << std::endl;

        return Joint::Ptr();

    }

}

XBot::Joint::Ptr XBot::XBotInterface::getJointByIdInternal(int joint_id) const
{
    auto it = _joint_id_to_eigen_id.find(joint_id);

    if( it != _joint_id_to_eigen_id.end() ){
        return _ordered_joint_vector[it->second];
    }

    for( const auto& c : _chain_map){
        const XBot::KinematicChain& chain = *c.second;
        if(chain.hasJoint(joint_id)) return chain.getJointInternal(chain.getChainDofIndex(joint_id));
    }
    std::cerr << "ERROR in " << __func__ << ". Joint " << joint_id << " is NOT defined!" << std::endl;
    return XBot::Joint::Ptr();
}


void XBot::XBotInterface::initLog(MatLogger::Ptr logger, int buffer_size, int interleave, std::string prefix)
{

    if( prefix != "" ){
        prefix = prefix + "_";
    }


    logger->createVectorVariable(prefix + "joint_position", getJointNum(), interleave, buffer_size);
    logger->createVectorVariable(prefix + "motor_position", getJointNum(), interleave, buffer_size);

    logger->createVectorVariable(prefix + "joint_velocity", getJointNum(), interleave, buffer_size);
    logger->createVectorVariable(prefix + "motor_velocity", getJointNum(), interleave, buffer_size);

    logger->createVectorVariable(prefix + "joint_acceleration", getJointNum(), interleave, buffer_size);

    logger->createVectorVariable(prefix + "joint_effort", getJointNum(), interleave, buffer_size);

    logger->createVectorVariable(prefix + "impedance_k", getJointNum(), interleave, buffer_size);
    logger->createVectorVariable(prefix + "impedance_d", getJointNum(), interleave, buffer_size);

    logger->createVectorVariable(prefix + "temperature", getJointNum(), interleave, buffer_size);

    logger->createVectorVariable(prefix + "position_reference", getJointNum(), interleave, buffer_size);
    logger->createVectorVariable(prefix + "effort_reference", getJointNum(), interleave, buffer_size);
    logger->createVectorVariable(prefix + "velocity_reference", getJointNum(), interleave, buffer_size);

    logger->createScalarVariable(prefix + "time", interleave, buffer_size);

    for(const auto& pair : _imu_map){
        std::string imu_name = pair.first;
        logger->createVectorVariable(prefix + "IMU_" + imu_name + "_lin_acc", 3, interleave, buffer_size);
        logger->createVectorVariable(prefix + "IMU_" + imu_name + "_ang_vel", 3, interleave, buffer_size);
        logger->createMatrixVariable(prefix + "IMU_" + imu_name + "_orientation", 3, 3, interleave, buffer_size);
    }

    for(const auto& pair : _ft_map){
        std::string ft_name = pair.first;
        logger->createVectorVariable(prefix + "FT_" + ft_name + "_wrench", 6, interleave, buffer_size);
    }

    getJointPosition(_qlink);
    getMotorPosition(_qmot);
    getJointVelocity(_qdotlink);
    getMotorVelocity(_qdotmot);
    getJointEffort(_tau);
    getStiffness(_stiffness);
    getDamping(_damping);
    getTemperature(_temp);



}

void XBot::XBotInterface::log(MatLogger::Ptr logger, double timestamp, const std::string& prefix) const
{
    if(!logger){
        std::cout << "ERROR in " << __PRETTY_FUNCTION__ << "! Provided logger is null!" << std::endl;
        return;
    }

    getJointPosition(_qlink);
    getMotorPosition(_qmot);
    getJointVelocity(_qdotlink);
    getMotorVelocity(_qdotmot);
    getJointEffort(_tau);
    getStiffness(_stiffness);
    getDamping(_damping);
    getTemperature(_temp);

    logger->add( prefix + "joint_position", _qlink);
    logger->add( prefix + "motor_position", _qmot);

    logger->add( prefix + "joint_velocity", _qdotlink);
    logger->add( prefix + "motor_velocity", _qdotmot);

    logger->add( prefix + "joint_effort", _tau);

    logger->add( prefix + "impedance_k", _stiffness);
    logger->add( prefix + "impedance_d", _damping);

    logger->add( prefix + "temperature", _temp);

    logger->add( prefix + "time", timestamp);

    getPositionReference(_qlink);
    getVelocityReference(_qdotlink);
    getEffortReference(_tau);

    logger->add( prefix + "position_reference", _qlink);
    logger->add( prefix + "velocity_reference", _qdotlink);
    logger->add( prefix + "effort_reference", _tau);

    getJointAcceleration(_qlink);
    logger->add( prefix + "joint_acceleration", _qlink);

    for(const auto& pair : _imu_map){
        const std::string& imu_name = pair.first;
        const ImuSensor& imu = *pair.second;
        Eigen::Vector3d w, a;
        Eigen::Quaterniond o;
        imu.getImuData(o, a, w);

        logger->add(prefix + "IMU_" + imu_name + "_lin_acc", a);
        logger->add(prefix + "IMU_" + imu_name + "_ang_vel", w);
        logger->add(prefix + "IMU_" + imu_name + "_orientation", o.matrix());
    }

    for(const auto& pair : _ft_map){
        const std::string& ft_name = pair.first;
        const ForceTorqueSensor& ft = *pair.second;
        Eigen::Vector6d w;
        ft.getWrench(w);

        logger->add(prefix + "FT_" + ft_name + "_wrench", w);
    }

}

XBot::ForceTorqueSensor::ConstPtr XBot::XBotInterface::getForceTorque(int ft_id) const
{
    auto it = _ft_id_map.find(ft_id);

    if( it == _ft_id_map.end() ){
        std::cerr << "ERROR in " << __PRETTY_FUNCTION__ << ", FT sensor with given ID " << ft_id << " is NOT defined!\n" <<
        "Available FT sensor are:\n";

        for( const auto& pair : _ft_map ){
            std::cerr << "\tName: " << pair.first << ", ID: " << pair.second->getSensorId() << "\n";
        }

        std::cerr << std::endl;

        return XBot::ForceTorqueSensor::ConstPtr();
    }

    return it->second;
}

XBot::ImuSensor::ConstPtr XBot::XBotInterface::getImu(int imu_id) const
{
    auto it = _imu_id_map.find(imu_id);

    if( it == _imu_id_map.end() ){
        std::cerr << "ERROR in " << __PRETTY_FUNCTION__ << ", IMU sensor with given ID " << imu_id << " is NOT defined!\n" <<
        "Available IMU sensor are:\n";

        for( const auto& pair : _imu_map ){
            std::cerr << "\tName: " << pair.first << ", ID: " << pair.second->getSensorId() << "\n";
        }

        std::cerr << std::endl;

        return XBot::ImuSensor::ConstPtr();
    }

    return it->second;
}























