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

#include <XBotInterface/IXBotInterface.h>

#define CONFIG_MIDDLE_PATH "/external/XBotInterface/IXBotInterface/configs/"


XBot::IXBotInterface::IXBotInterface()
{

}

XBot::IXBotInterface::IXBotInterface(const XBot::IXBotInterface &other):
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
    _path_to_cfg(other._path_to_cfg)
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

}

int XBot::IXBotInterface::getJointNum() const
{
    return _joint_num;
}


const std::vector< std::string >& XBot::IXBotInterface::getModelOrderedChainName() const
{
    return _XBotModel.get_ordered_chain_names();
}

bool XBot::IXBotInterface::hasChain(const std::string& chain_name) const
{
    return _chain_map.count(chain_name) == 1;
}


bool XBot::IXBotInterface::computeAbsolutePath ( const std::string& input_path,
                                                 const std::string& middle_path,
                                                 std::string& absolute_path, 
                                                 std::string extension)
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
            current_path += extension;
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

bool XBot::IXBotInterface::parseYAML ( const std::string& path_to_cfg )
{
    std::ifstream fin(path_to_cfg);
    if (fin.fail()) {
        printf("Can not open %s\n", path_to_cfg.c_str());    //TBD change it
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
   
    // check the urdf_filename
    if(x_bot_interface["urdf_filename"]) {
        computeAbsolutePath(x_bot_interface["urdf_filename"].as<std::string>(), 
                            CONFIG_MIDDLE_PATH,
                            _urdf_path); 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain urdf_filename mandatory node!!" << std::endl;
        return false;
    }
    
    // check the srdf_filename
    if(x_bot_interface["srdf_filename"]) {
        computeAbsolutePath(x_bot_interface["srdf_filename"].as<std::string>(), 
                            CONFIG_MIDDLE_PATH,
                            _srdf_path); 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain srdf_filename mandatory node!!" << std::endl;
        return false;
    }
    
    // check joint_map_config
    if(x_bot_interface["joint_map_config"]) {
        computeAbsolutePath(x_bot_interface["joint_map_config"].as<std::string>(), 
                            CONFIG_MIDDLE_PATH,
                            _joint_map_config); 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : x_bot_interface node of  " << path_to_cfg << "  does not contain joint_map_config mandatory node!!" << std::endl;
        return false;
    }

}

const std::string& XBot::IXBotInterface::getPathToConfig() const
{
    return _path_to_cfg;
}

bool XBot::IXBotInterface::init(const std::string &path_to_cfg)
{
    // store path to config
    _path_to_cfg = path_to_cfg;
    // parse the YAML file to initialize internal variables
    parseYAML(path_to_cfg);
    // initialize the model
    if (!_XBotModel.init(_urdf_path, _srdf_path, _joint_map_config)) {
        printf("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n"); 
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

    // create dynamically the Kinematic Chains
    for (const std::string & chain_name : _XBotModel.get_chain_names()) {
        XBot::KinematicChain::Ptr actual_chain = std::make_shared<KinematicChain>(chain_name,
                                                                    _XBotModel);
        _chain_map[chain_name] = actual_chain;
    }
    
    // NOTE if you have disabled joint, the URDF should be updated in order to have compatibility btw robot and model
    if (_XBotModel.getDisabledJoints().size() > 0) {
         std::cerr << "WARNING in " << __func__ << " : disabled joint detected in the specified SRDF. URDF must be updated accordingly (we hope you did it)." << std::endl;
    }

    // call virtual init_internal
    bool success = init_internal(path_to_cfg);
    
    // after subclasses have done their work inside init_internal, compute joint number
    _joint_num = 0;
    for( const auto& c : _chain_map ){
        _joint_num += c.second->getJointNum();
    }
    
    // fill the joint id to eigen id
    int eigen_id = 0;
    for( const std::string& chain_name : getModelOrderedChainName() ) {
        for( int i = 0; i < _chain_map.at(chain_name)->getJointNum(); i++) {
            _joint_id_to_eigen_id[_chain_map.at(chain_name)->jointId(i)] = eigen_id;
            _joint_name_to_eigen_id[_chain_map.at(chain_name)->jointName(i)] = eigen_id;
            _ordered_joint_vector.push_back(_chain_map.at(chain_name)->getJoint(i));
            eigen_id++;
        }
    }
    
    return success;
    
}

bool XBot::IXBotInterface::getEigenID ( const std::string& chain_name, std::vector< int >& ids ) const  //TBD don't replicate it in ModelInterface
{
    if(_chain_map.count(chain_name)){
        
        const KinematicChain& chain = *_chain_map.at(chain_name);
        ids.resize(chain.getJointNum());
        for(int i=0; i< chain.getJointNum(); i++){
            ids[i] = _joint_id_to_eigen_id.at(chain.jointId(i));
        }
        
    }
    else{
     std::cerr << "ERROR in " << __func__ << ": requested chain " << chain_name << " is not defined!" << std::endl;
     return false;
    }
    
    return true;
    
}

int XBot::IXBotInterface::getEigenID(const std::string& joint_name) const
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



int XBot::IXBotInterface::legs() const
{
    return _XBotModel.get_legs_chain().size();
}



int XBot::IXBotInterface::arms() const
{
    return _XBotModel.get_arms_chain().size();
}

bool XBot::IXBotInterface::sync_internal(const XBot::IXBotInterface &other)
{
    bool success = true;
    for (const auto & c : other._chain_map) {
        const std::string &chain_name = c.first;
        const KinematicChain &chain = *c.second;
        if (_chain_map.count(chain_name)) {
            _chain_map.at(chain_name)->syncFrom(chain);
            
        } else {
            if(!chain.isVirtual()){
                std::cerr << "ERROR " << __func__ << " : you are trying to synchronize IXBotInterfaces with different chains!!" << std::endl;
                success = false;
            }
        }
    }
    return success;
}


bool XBot::IXBotInterface::hasJoint(const std::string &joint_name) const
{
    return std::find(_ordered_joint_name.begin(), _ordered_joint_name.end(), joint_name) !=  _ordered_joint_name.end();
}


XBot::Joint::ConstPtr XBot::IXBotInterface::getJointByName(const std::string& joint_name) const
{
    for(const auto& c : _chain_map){
     
        const XBot::KinematicChain& chain = *c.second;
        if(chain.hasJoint(joint_name)) return chain.getJointByName(joint_name);
        
    }
    
    std::cerr << "ERROR in " << __func__ << ". Joint " << joint_name << " is NOT defined!" << std::endl;
    return XBot::Joint::ConstPtr();
}



bool XBot::IXBotInterface::getJointPosition(Eigen::VectorXd &q) const
{
    if (q.rows() != _joint_num) {
        q.resize(_joint_num);
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            q[q_index++] = chain.getJointPosition(i);
        }
    }
    return true;
}

bool XBot::IXBotInterface::getJointPosition(std::map< std::string, double > &q) const
{
//     q.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            q[chain.jointName(j)] = chain.getJointPosition(j);
        }

    }
}

bool XBot::IXBotInterface::getJointPosition(std::map< int, double > &q) const
{
//     q.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            q[chain.jointId(j)] = chain.getJointPosition(j);
        }

    }
}

bool XBot::IXBotInterface::getPositionReference(std::map< std::string, double > &q) const
{
//     q.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            q[chain.jointName(j)] = chain.getPositionReference(j);
        }

    }
}

bool XBot::IXBotInterface::getPositionReference(std::map< int, double > &q) const
{
//     q.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            q[chain.jointId(j)] = chain.getPositionReference(j);
        }

    }
}

bool XBot::IXBotInterface::getPositionReference(Eigen::VectorXd &q) const
{
    if (q.rows() != _joint_num) {
        q.resize(_joint_num);
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            q[q_index++] = chain.getPositionReference(i);
        }
    }
    return true;
}

bool XBot::IXBotInterface::getDamping(std::map< std::string, double > &D) const
{
//     D.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            D[chain.jointName(j)] = chain.getDamping(j);
        }

    }
}

bool XBot::IXBotInterface::getJointEffort(std::map< std::string, double > &tau) const
{
//     tau.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            tau[chain.jointName(j)] = chain.getJointEffort(j);
        }

    }
}

bool XBot::IXBotInterface::getEffortReference(std::map< std::string, double > &tau) const
{
//     tau.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            tau[chain.jointName(j)] = chain.getEffortReference(j);
        }

    }
}

bool XBot::IXBotInterface::getJointVelocity(std::map< std::string, double > &qdot) const
{
//     qdot.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            qdot[chain.jointName(j)] = chain.getJointVelocity(j);
        }

    }
}

bool XBot::IXBotInterface::getMotorPosition(std::map< std::string, double > &q) const
{
//     q.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            q[chain.jointName(j)] = chain.getMotorPosition(j);
        }

    }
}

bool XBot::IXBotInterface::getStiffness(std::map< std::string, double > &K) const
{
//     K.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            K[chain.jointName(j)] = chain.getStiffness(j);
        }

    }
}

bool XBot::IXBotInterface::getTemperature(std::map< std::string, double > &temp) const
{
//     temp.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            temp[chain.jointName(j)] = chain.getTemperature(j);
        }

    }
}

bool XBot::IXBotInterface::getVelocityReference(std::map< std::string, double > &qdot) const
{
//     qdot.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            qdot[chain.jointName(j)] = chain.getVelocityReference(j);
        }

    }
}

bool XBot::IXBotInterface::getMotorVelocity(std::map< std::string, double > &qdot) const
{
//     qdot.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            qdot[chain.jointName(j)] = chain.getMotorVelocity(j);
        }

    }
}



bool XBot::IXBotInterface::getDamping(std::map< int, double > &D) const
{
//     D.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            D[chain.jointId(j)] = chain.getDamping(j);
        }

    }
}

bool XBot::IXBotInterface::getJointEffort(std::map< int, double > &tau) const
{
//     tau.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            tau[chain.jointId(j)] = chain.getJointEffort(j);
        }

    }
}

bool XBot::IXBotInterface::getEffortReference(std::map< int, double > &tau) const
{
//     tau.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            tau[chain.jointId(j)] = chain.getEffortReference(j);
        }

    }
}


bool XBot::IXBotInterface::getVelocityReference(std::map< int, double > &qdot) const
{
//     qdot.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            qdot[chain.jointId(j)] = chain.getVelocityReference(j);
        }

    }
}


bool XBot::IXBotInterface::getJointVelocity(std::map< int, double > &qdot) const
{
//     qdot.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            qdot[chain.jointId(j)] = chain.getJointVelocity(j);
        }

    }
}

bool XBot::IXBotInterface::getMotorPosition(std::map< int, double > &q) const
{
//     q.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            q[chain.jointId(j)] = chain.getMotorPosition(j);
        }

    }
}

bool XBot::IXBotInterface::getMotorVelocity(std::map< int, double > &qdot) const
{
//     qdot.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            qdot[chain.jointId(j)] = chain.getMotorVelocity(j);
        }

    }
}

bool XBot::IXBotInterface::getStiffness(std::map< int, double > &K) const
{
//     K.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            K[chain.jointId(j)] = chain.getStiffness(j);
        }

    }
}

bool XBot::IXBotInterface::getTemperature(std::map< int, double > &temp) const
{
//     temp.clear();

    for (const auto & chain_name_ptr_pair : _chain_map) {

        const KinematicChain &chain = *chain_name_ptr_pair.second;

        for (int j = 0; j < chain.getJointNum(); j++) {
            temp[chain.jointId(j)] = chain.getTemperature(j);
        }

    }
}




bool XBot::IXBotInterface::getDamping(Eigen::VectorXd &D) const
{
    if (D.rows() != _joint_num) {
        D.resize(_joint_num);
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            D[q_index++] = chain.getDamping(i);
        }
    }
    return true;
}

bool XBot::IXBotInterface::getJointEffort(Eigen::VectorXd &tau) const
{
    if (tau.rows() != _joint_num) {
        tau.resize(_joint_num);
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            tau[q_index++] = chain.getJointEffort(i);
        }
    }
    return true;
}

bool XBot::IXBotInterface::getEffortReference(Eigen::VectorXd &tau) const
{
    if (tau.rows() != _joint_num) {
        tau.resize(_joint_num);
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            tau[q_index++] = chain.getEffortReference(i);
        }
    }
    return true;
}

bool XBot::IXBotInterface::getJointVelocity(Eigen::VectorXd &qdot) const
{
    if (qdot.rows() != _joint_num) {
        qdot.resize(_joint_num);
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            qdot[q_index++] = chain.getJointVelocity(i);
        }
    }
    return true;
}

bool XBot::IXBotInterface::getMotorPosition(Eigen::VectorXd &q) const
{
    if (q.rows() != _joint_num) {
        q.resize(_joint_num);
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            q[q_index++] = chain.getMotorPosition(i);
        }
    }
    return true;
}

bool XBot::IXBotInterface::getMotorVelocity(Eigen::VectorXd &qdot) const
{
    if (qdot.rows() != _joint_num) {
        qdot.resize(_joint_num);
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            qdot[q_index++] = chain.getMotorVelocity(i);
        }
    }
    return true;
}

bool XBot::IXBotInterface::getStiffness(Eigen::VectorXd &K) const
{
    if (K.rows() != _joint_num) {
        K.resize(_joint_num);
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            K[q_index++] = chain.getStiffness(i);
        }
    }
    return true;
}

bool XBot::IXBotInterface::getTemperature(Eigen::VectorXd &temp) const
{
    if (temp.rows() != _joint_num) {
        temp.resize(_joint_num);
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            temp[q_index++] = chain.getTemperature(i);
        }
    }
    return true;
}

bool XBot::IXBotInterface::getVelocityReference(Eigen::VectorXd &qdot) const
{
    if (qdot.rows() != _joint_num) {
        qdot.resize(_joint_num);
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        const XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            qdot[q_index++] = chain.getVelocityReference(i);
        }
    }
    return true;
}


bool XBot::IXBotInterface::setJointPosition(const std::map< std::string, double > &q)
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;
        for (int i = 0; i < chain.getJointNum(); i++) {
            const std::string &joint_name = chain.jointName(i);
            if (q.count(joint_name)) {
                chain.setJointPosition(i, q.at(joint_name));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }
    }
    return success;
}

bool XBot::IXBotInterface::setMotorPosition(const std::map< std::string, double > &q)
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            const std::string &joint_name = chain.jointName(i);
            if (q.count(joint_name)) {
                chain.setMotorPosition(i, q.at(joint_name));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setPositionReference(const std::map< std::string, double > &q)
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            const std::string &joint_name = chain.jointName(i);
            if (q.count(joint_name)) {
                chain.setPositionReference(i, q.at(joint_name));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setJointEffort(const std::map< std::string, double > &tau)
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            const std::string &joint_name = chain.jointName(i);
            if (tau.count(joint_name)) {
                chain.setJointEffort(i, tau.at(joint_name));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setEffortReference(const std::map< std::string, double > &tau)
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            const std::string &joint_name = chain.jointName(i);
            if (tau.count(joint_name)) {
                chain.setEffortReference(i, tau.at(joint_name));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setJointVelocity(const std::map< std::string, double > &qdot)
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            const std::string &joint_name = chain.jointName(i);
            if (qdot.count(joint_name)) {
                chain.setJointVelocity(i, qdot.at(joint_name));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setMotorVelocity(const std::map< std::string, double > &qdot)
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            const std::string &joint_name = chain.jointName(i);
            if (qdot.count(joint_name)) {
                chain.setMotorVelocity(i, qdot.at(joint_name));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setVelocityReference(const std::map< std::string, double > &qdot)
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            const std::string &joint_name = chain.jointName(i);
            if (qdot.count(joint_name)) {
                chain.setVelocityReference(i, qdot.at(joint_name));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setTemperature(const std::map< std::string, double > &temp)
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            const std::string &joint_name = chain.jointName(i);
            if (temp.count(joint_name)) {
                chain.setTemperature(i, temp.at(joint_name));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setStiffness(const std::map< std::string, double > &K)
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            const std::string &joint_name = chain.jointName(i);
            if (K.count(joint_name)) {
                chain.setStiffness(i, K.at(joint_name));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setDamping ( const Eigen::VectorXd& D )
{
     if (D.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : D has wrong size " << D.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName() ) {
        
        XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            chain.setDamping(i, D[q_index++]);
        }
    }
    return true;
}

bool XBot::IXBotInterface::setJointEffort ( const Eigen::VectorXd& tau )
{
     if (tau.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : tau has wrong size " << tau.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            chain.setJointEffort(i, tau[q_index++]);
        }
    }
    return true;
}

bool XBot::IXBotInterface::setEffortReference ( const Eigen::VectorXd& tau )
{
     if (tau.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : tau has wrong size " << tau.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            chain.setEffortReference(i, tau[q_index++]);
        }
    }
    return true;
}

bool XBot::IXBotInterface::setJointPosition ( const Eigen::VectorXd& q )
{
     if (q.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : q has wrong size " << q.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            chain.setJointPosition(i, q[q_index++]);
        }
    }
    return true;
}

bool XBot::IXBotInterface::setJointVelocity ( const Eigen::VectorXd& qdot )
{
     if (qdot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qdot has wrong size " << qdot.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            chain.setJointVelocity(i, qdot[q_index++]);
        }
    }
    return true;
}

bool XBot::IXBotInterface::setMotorPosition( const Eigen::VectorXd& q )
{
     if (q.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : q has wrong size " << q.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            chain.setMotorPosition(i, q[q_index++]);
        }
    }
    return true;
}

bool XBot::IXBotInterface::setMotorVelocity( const Eigen::VectorXd& qdot )
{
     if (qdot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qdot has wrong size " << qdot.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            chain.setMotorVelocity(i, qdot[q_index++]);
        }
    }
    return true;
}

bool XBot::IXBotInterface::setPositionReference ( const Eigen::VectorXd& q )
{
     if (q.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : q has wrong size " << q.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            chain.setPositionReference(i, q[q_index++]);
        }
    }
    return true;
}

bool XBot::IXBotInterface::setStiffness ( const Eigen::VectorXd& K )
{
     if (K.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : K has wrong size " << K.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            chain.setStiffness(i, K[q_index++]);
        }
    }
    return true;
}

bool XBot::IXBotInterface::setTemperature ( const Eigen::VectorXd& temp )
{
     if (temp.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : temp has wrong size " << temp.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            chain.setTemperature(i, temp[q_index++]);
        }
    }
    return true;
}

bool XBot::IXBotInterface::setVelocityReference ( const Eigen::VectorXd& qdot )
{
     if (qdot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qdot has wrong size " << qdot.rows() << " != robot joint number " << _joint_num << std::endl;
        return false;
    }
    int q_index = 0;
    for (const std::string & chain_name : getModelOrderedChainName()) {
        
        XBot::KinematicChain& chain = *_chain_map.at(chain_name);
        int chain_joint_num = chain.getJointNum();
        
        for (int i = 0; i < chain_joint_num; i++) {
            chain.setVelocityReference(i, qdot[q_index++]);
        }
    }
    return true;
}





bool XBot::IXBotInterface::setDamping(const std::map< std::string, double > &D)
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            const std::string &joint_name = chain.jointName(i);
            if (D.count(joint_name)) {
                chain.setDamping(i, D.at(joint_name));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setDamping ( const std::map< int, double >& D )
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            int joint_id = chain.jointId(i);
            if (D.count(joint_id)) {
                chain.setDamping(i, D.at(joint_id));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_id << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}


bool XBot::IXBotInterface::setJointEffort ( const std::map< int, double >& tau )
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            int joint_id = chain.jointId(i);
            if (tau.count(joint_id)) {
                chain.setJointEffort(i, tau.at(joint_id));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_id << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setEffortReference ( const std::map< int, double >& tau )
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            int joint_id = chain.jointId(i);
            if (tau.count(joint_id)) {
                chain.setEffortReference(i, tau.at(joint_id));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_id << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setJointPosition ( const std::map< int, double >& q )
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            int joint_id = chain.jointId(i);
            if (q.count(joint_id)) {
                chain.setJointPosition(i, q.at(joint_id));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_id << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setJointVelocity ( const std::map< int, double >& qdot )
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            int joint_id = chain.jointId(i);
            if (qdot.count(joint_id)) {
                chain.setJointVelocity(i, qdot.at(joint_id));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_id << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setMotorPosition( const std::map< int, double >& q )
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            int joint_id = chain.jointId(i);
            if (q.count(joint_id)) {
                chain.setMotorPosition(i, q.at(joint_id));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_id << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setMotorVelocity( const std::map< int, double >& qdot )
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            int joint_id = chain.jointId(i);
            if (qdot.count(joint_id)) {
                chain.setMotorVelocity(i, qdot.at(joint_id));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_id << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setPositionReference ( const std::map< int, double >& q )
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            int joint_id = chain.jointId(i);
            if (q.count(joint_id)) {
                chain.setPositionReference(i, q.at(joint_id));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_id << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setStiffness ( const std::map< int, double >& K )
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            int joint_id = chain.jointId(i);
            if (K.count(joint_id)) {
                chain.setStiffness(i, K.at(joint_id));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_id << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setTemperature ( const std::map< int, double >& temp )
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            int joint_id = chain.jointId(i);
            if (temp.count(joint_id)) {
                chain.setTemperature(i, temp.at(joint_id));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_id << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}

bool XBot::IXBotInterface::setVelocityReference ( const std::map< int, double >& qdot )
{
    bool success = true;
    for (const auto & chainname_ptr_pair : _chain_map) {

        KinematicChain &chain = *chainname_ptr_pair.second;

        for (int i = 0; i < chain.getJointNum(); i++) {
            int joint_id = chain.jointId(i);
            if (qdot.count(joint_id)) {
                chain.setVelocityReference(i, qdot.at(joint_id));
            } else {
                
                if(!chain.isVirtual()){
                
                    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_id << " is not defined!" << std::endl;
                    success = false;
                
                }
            }
        }

    }
    return success;
}



std::vector< std::string > XBot::IXBotInterface::getChainNames() const
{
    return _XBotModel.get_chain_names();
}

const std::map< std::string, XBot::KinematicChain::Ptr >& XBot::IXBotInterface::getChainMap() const
{
    return _chain_map;
}


const srdf::Model& XBot::IXBotInterface::getSrdf() const
{
    return _XBotModel;
}

const std::string& XBot::IXBotInterface::getSrdfString() const
{
    return _srdf_string;
}

const urdf::ModelInterface& XBot::IXBotInterface::getUrdf() const
{
    return *(_XBotModel.get_urdf_model());
}

const std::string& XBot::IXBotInterface::getUrdfString() const
{
    return _urdf_string;
}



XBot::IXBotInterface::~IXBotInterface()
{
}

XBot::IXBotInterface &XBot::IXBotInterface::operator=(const XBot::IXBotInterface &rhs)
{
    IXBotInterface tmp(rhs);
    std::swap(_joint_num, tmp._joint_num);
    std::swap(_XBotModel, tmp._XBotModel);
    std::swap(_urdf_string, tmp._urdf_string);
    std::swap(_srdf_string, tmp._srdf_string);
    std::swap(_ordered_joint_name, tmp._ordered_joint_name);
    std::swap(_ordered_joint_id, tmp._ordered_joint_id);
    std::swap(_chain_map, tmp._chain_map);
    std::swap(_joint_id_to_eigen_id, tmp._joint_id_to_eigen_id);
    std::swap(_joint_name_to_eigen_id, tmp._joint_name_to_eigen_id);
    std::swap(_path_to_cfg, tmp._path_to_cfg);
    std::swap(_ordered_joint_vector, tmp._ordered_joint_vector);
    std::swap(_joint_map_config, tmp._joint_map_config);

}

bool XBot::IXBotInterface::checkEffortLimits(const Eigen::VectorXd& tau) const
{
    int idx = 0;
    for( const std::string& s : getModelOrderedChainName() ){
        const KinematicChain& chain = *_chain_map.at(s);
        if( !chain.checkEffortLimits(tau.segment(idx, chain.getJointNum())) ){
            idx += chain.getJointNum();
            return false;
        }
    }
    
    return true;
}

bool XBot::IXBotInterface::checkEffortLimits(const Eigen::VectorXd& tau, 
                                             std::vector< std::string >& violating_joints) const
{
    bool success = true;
    
    int idx = 0;
    for( const std::string& s : getModelOrderedChainName() ){
        const KinematicChain& chain = *_chain_map.at(s);
        success = success && chain.checkEffortLimits(tau.segment(idx, chain.getJointNum()), violating_joints);
        idx += chain.getJointNum();
    }
    
    return success;
}

bool XBot::IXBotInterface::checkJointLimits(const Eigen::VectorXd& q) const
{
    int idx = 0;
    for( const std::string& s : getModelOrderedChainName() ){
        const KinematicChain& chain = *_chain_map.at(s);
        if( !chain.checkJointLimits(q.segment(idx, chain.getJointNum())) ){
            idx += chain.getJointNum();
            return false;
        }
    }
    
    return true;
}

bool XBot::IXBotInterface::checkJointLimits(const Eigen::VectorXd& q, 
                                            std::vector< std::string >& violating_joints) const
{
    bool success = true;
    
    int idx = 0;
    for( const std::string& s : getModelOrderedChainName() ){
        const KinematicChain& chain = *_chain_map.at(s);
        success = success && chain.checkJointLimits(q.segment(idx, chain.getJointNum()), violating_joints);
        idx += chain.getJointNum();
    }
    
    return success;
}

bool XBot::IXBotInterface::checkVelocityLimits(const Eigen::VectorXd& qdot) const
{
    int idx = 0;
    for( const std::string& s : getModelOrderedChainName() ){
        const KinematicChain& chain = *_chain_map.at(s);
        if( !chain.checkVelocityLimits(qdot.segment(idx, chain.getJointNum())) ){
            idx += chain.getJointNum();
            return false;
        }
    }
    
    return true;
}

bool XBot::IXBotInterface::checkVelocityLimits(const Eigen::VectorXd& qdot, 
                                               std::vector< std::string >& violating_joints) const
{
    bool success = true;
    
    int idx = 0;
    for( const std::string& s : getModelOrderedChainName() ){
        const KinematicChain& chain = *_chain_map.at(s);
        success = success && chain.checkVelocityLimits(qdot.segment(idx, chain.getJointNum()), violating_joints);
        idx += chain.getJointNum();
    }
    
    return success;
}

void XBot::IXBotInterface::getEffortLimits(Eigen::VectorXd& tau_max) const
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

void XBot::IXBotInterface::getJointLimits(Eigen::VectorXd& q_min, Eigen::VectorXd& q_max) const
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

void XBot::IXBotInterface::getVelocityLimits(Eigen::VectorXd& qdot_max) const
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



const std::vector< std::string > &XBot::IXBotInterface::getEnabledJointNames() const
{
    return _ordered_joint_name;
}

std::ostream& XBot::operator<< ( std::ostream& os, const XBot::IXBotInterface& robot )
{
    os << "Robot name: " << std::endl;
    for( const auto& c : robot.getChainMap() ) {
        os << (*c.second) << std::endl;
    }
    return os;
}



