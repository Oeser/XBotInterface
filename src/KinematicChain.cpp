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

#include <XBotInterface/KinematicChain.h>
#include <eigen3/Eigen/Dense>

#include <XBotInterface/RtLog.hpp>


namespace XBot
{

KinematicChain::KinematicChain() :
    _chain_name("dummy_chain"),
    _joint_num(0),
    _is_virtual(false)
{

}

KinematicChain::KinematicChain(const std::string &chain_name,
                               const XBot::XBotCoreModel &XBotModel) :
    _chain_name(chain_name),
    _XBotModel(XBotModel),
    _is_virtual(false),
    _joint_num(0)
{
    _joint_num = _XBotModel.get_joint_num(chain_name);
    _XBotModel.get_enabled_joints_in_chain(chain_name, _ordered_joint_name);
    _XBotModel.get_enabled_joint_ids_in_chain(chain_name, _ordered_joint_id);

    // Get urdf model
    const urdf::ModelInterface &robot_urdf = *XBotModel.get_urdf_model();

    // resize joint vector
    _joint_vector.resize(_joint_num);
    // initialize the joint maps and vector
    for (int i = 0; i < _joint_num; i++) {
        std::string actual_joint_name = _ordered_joint_name[i];
        int actual_joint_id = _ordered_joint_id[i];
        XBot::Joint::Ptr actual_joint = std::make_shared<Joint>(actual_joint_name,
                                                                actual_joint_id,
                                                                robot_urdf.getJoint(actual_joint_name),
                                                                _chain_name);
        _joint_name_map[actual_joint_name] = actual_joint;
        _joint_id_map[actual_joint_id] = actual_joint;
        _joint_vector[i] = actual_joint;
    }


    // Fill _urdf_joints and _urdf_links
    // NOTE: is it the correct way to handle links? What happens if some joint
    // along the chain is disabled/fixed/...?
    for (const std::string & joint_name : _ordered_joint_name) {

        _urdf_joints.push_back(robot_urdf.getJoint(joint_name));

        const std::string &parent_link_name = robot_urdf.getJoint(joint_name)->parent_link_name;
        _urdf_links.push_back(robot_urdf.getLink(parent_link_name));

    }

    // Add last link to _urdf_links
    _urdf_links.push_back(robot_urdf.getLink(getChildLinkName(getJointNum() - 1)));

    // Add FT
    for( const auto& ft_name_id : _XBotModel.get_ft_sensors() ){

        const std::string& ft_joint_name = ft_name_id.first;
        const int ft_id = ft_name_id.second;
        std::string ft_link_name = robot_urdf.getJoint(ft_joint_name)->child_link_name;
        const std::string& ft_parent_link_name = robot_urdf.getJoint(ft_joint_name)->parent_link_name;

        // check the FT on this chain
        for( const auto& link_in_chain : _urdf_links) {
            if(link_in_chain->name == ft_parent_link_name) {
                ForceTorqueSensor::Ptr ft_ptr = std::make_shared<ForceTorqueSensor>(robot_urdf.getLink(ft_link_name), ft_id);
                _ft_vector.push_back(ft_ptr);
                _ft_map[ft_ptr->getSensorName()] = ft_ptr;
                Logger::info() << "Found FT " << ft_link_name << " on chain " << getChainName() << Logger::endl();
            }
        }


    }

    // Add IMU
    for( const auto& imu_name_id : _XBotModel.get_imu_sensors() ){

        const int imu_id = imu_name_id.second;
        std::string imu_link_name = imu_name_id.first;
        std::string imu_parent_link_name =
                robot_urdf.getLink(imu_link_name)->parent_joint->parent_link_name;

        // check the if IMU is on this chain
        for( const auto& link_in_chain : _urdf_links) {
            if(link_in_chain->name == imu_parent_link_name) {
                ImuSensor::Ptr imu_ptr = std::make_shared<ImuSensor>(robot_urdf.getLink(imu_link_name), imu_id);
                _imu_vector.push_back(imu_ptr);
                _imu_map[imu_ptr->getSensorName()] = imu_ptr;
                Logger::info() << "Found IMU " << imu_link_name << " on chain " << getChainName() << Logger::endl();
            }
        }


    }

}

KinematicChain::KinematicChain(const std::string& chain_name):
_chain_name(chain_name),
_is_virtual(false),
_joint_num(0)
{

}


KinematicChain::KinematicChain(const KinematicChain &other):
// Default copy of all non-ptr members
    _urdf_joints(other._urdf_joints),
    _urdf_links(other._urdf_links),
    _ordered_joint_name(other._ordered_joint_name),
    _ordered_joint_id(other._ordered_joint_id),
    _XBotModel(other._XBotModel),
    _chain_name(other._chain_name),
    _joint_num(other._joint_num),
    _is_virtual(other._is_virtual)
{


    for (const Joint::Ptr & other_jptr : other._joint_vector) {

        Joint::Ptr jptr = std::make_shared<Joint>();
        *jptr = *other_jptr;

        _joint_vector.push_back(jptr);
        _joint_name_map[jptr->getJointName()] = jptr;
        _joint_id_map[jptr->getJointId()] = jptr;

    }

    for (const ForceTorqueSensor::Ptr & other_ftptr : other._ft_vector) {

        ForceTorqueSensor::Ptr ftptr = std::make_shared<ForceTorqueSensor>();
        *ftptr = *other_ftptr;

        _ft_vector.push_back(ftptr);
        _ft_map[ftptr->getSensorName()] = ftptr;

    }

    for (const ImuSensor::Ptr & other_imuptr : other._imu_vector) {

        ImuSensor::Ptr imuptr = std::make_shared<ImuSensor>();
        *imuptr = *other_imuptr;

        _imu_vector.push_back(imuptr);
        _imu_map[imuptr->getSensorName()] = imuptr;

    }

}


KinematicChain &KinematicChain::operator=(const KinematicChain &rhs)
{

    // First, make a copy of rhs exploiting the custom (deep) copy constructor
    KinematicChain tmp(rhs);

    // Swap all elements
    std::swap(_urdf_joints, tmp._urdf_joints);
    std::swap(_urdf_links, tmp._urdf_links);
    std::swap(_ordered_joint_name, tmp._ordered_joint_name);
    std::swap(_ordered_joint_id, tmp._ordered_joint_id);
    std::swap(_XBotModel, tmp._XBotModel);
    std::swap(_chain_name, tmp._chain_name);
    std::swap(_joint_num, tmp._joint_num);
    std::swap(_joint_vector, tmp._joint_vector);
    std::swap(_joint_name_map, tmp._joint_name_map);
    std::swap(_joint_id_map, tmp._joint_id_map);
    std::swap(_is_virtual, tmp._is_virtual);
    std::swap(_ft_vector, tmp._ft_vector);
    std::swap(_ft_map, tmp._ft_map);
    std::swap(_imu_vector, tmp._imu_vector);
    std::swap(_imu_map, tmp._imu_map);

    // Return this
    return *this;

}

void XBot::KinematicChain::shallowCopy(const KinematicChain& chain)
{
    *this = chain; // deep copy

    // Shallow copy of joint pointers
    _joint_vector = chain._joint_vector;
    _joint_name_map = chain._joint_name_map;
    _joint_id_map = chain._joint_id_map;
    _ft_vector = chain._ft_vector;
    _ft_map = chain._ft_map;
    _imu_vector = chain._imu_vector;
    _imu_map = chain._imu_map;
//     _hand_map = chain._hand_map;
}


bool XBot::KinematicChain::isVirtual() const
{
    return _is_virtual;
}


void XBot::KinematicChain::pushBackJoint ( Joint::Ptr joint )
{
    _joint_name_map[joint->getJointName()] = joint;
    _joint_id_map[joint->getJointId()] = joint;
    _joint_vector.push_back(joint);
    _urdf_joints.push_back(joint->getUrdfJoint());
    _ordered_joint_name.push_back(joint->getJointName());
    _joint_num = _joint_vector.size();
    if(joint->getJointId() < 0) _is_virtual = true;
}

void XBot::KinematicChain::removeJoint(int i)
{
    _joint_name_map.erase(getJointName(i));
    _joint_id_map.erase(getJointId(i));
    _joint_vector.erase(_joint_vector.begin() + i);
    _ordered_joint_name.erase(_ordered_joint_name.begin() + i);
    _joint_num--;
}


const std::vector< urdf::JointConstSharedPtr > &KinematicChain::getUrdfJoints() const
{
    return _urdf_joints;
}

const std::vector< urdf::LinkConstSharedPtr > &KinematicChain::getUrdfLinks() const
{
    return _urdf_links;
}

int KinematicChain::getJointNum() const
{
    return _joint_vector.size();
}

const std::string &KinematicChain::getBaseLinkName() const
{
    return _urdf_links[0]->name;
}

const std::string &KinematicChain::getTipLinkName() const
{
    return _urdf_links[_urdf_links.size() - 1]->name;
}

const std::string &KinematicChain::getChainName() const
{
    return _chain_name;
}

int KinematicChain::getJointId(int i) const
{
    return _joint_vector[i]->getJointId();
}


const std::string &KinematicChain::getChildLinkName(int id) const
{
    return _urdf_joints[id]->child_link_name;
}

const std::string &KinematicChain::getParentLinkName(int id) const
{
    return _urdf_joints[id]->parent_link_name;
}

const std::string &KinematicChain::getJointName(int id) const
{
    return _ordered_joint_name[id];
}

double KinematicChain::getJointPosition(int index) const
{
    return _joint_vector[index]->getJointPosition();
}


bool KinematicChain::getJointPosition(Eigen::VectorXd &q) const
{
    if (q.rows() != _joint_num) {
        q.resize(_joint_num);
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        q[pos++] = j->getJointPosition();
    }
    return true;
}

bool KinematicChain::getJointPosition(JointNameMap &q) const
{
//     q.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        q[joint_name] = joint.getJointPosition();

    }

}

bool KinematicChain::getDamping(JointNameMap &D) const
{
//     D.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        D[joint_name] = joint.getDamping();

    }
}

bool KinematicChain::getJointEffort(JointNameMap &tau) const
{
//     tau.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        tau[joint_name] = joint.getJointEffort();

    }
}

bool KinematicChain::getEffortReference(JointNameMap &tau) const
{
//     tau.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        tau[joint_name] = joint.getEffortReference();

    }
}

bool KinematicChain::getJointVelocity(JointNameMap &qdot) const
{
//     qdot.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        qdot[joint_name] = joint.getJointVelocity();

    }
}

bool XBot::KinematicChain::getJointAcceleration(JointNameMap& qddot) const
{
    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        qddot[joint_name] = joint.getJointAcceleration();

    }
}


bool KinematicChain::getMotorPosition(JointNameMap &q) const
{
//     q.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        q[joint_name] = joint.getMotorPosition();

    }
}

bool KinematicChain::getMotorVelocity(JointNameMap &qdot) const
{
//     qdot.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        qdot[joint_name] = joint.getMotorVelocity();

    }
}

bool KinematicChain::getStiffness(JointNameMap &K) const
{
//     K.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        K[joint_name] = joint.getStiffness();

    }
}

bool KinematicChain::getTemperature(JointNameMap &temp) const
{
//     temp.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        temp[joint_name] = joint.getTemperature();

    }
}

bool KinematicChain::getVelocityReference(JointNameMap &qdot) const
{
//     qdot.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        qdot[joint_name] = joint.getVelocityReference();

    }
}




bool KinematicChain::getJointPosition(JointIdMap &q) const
{
//     q.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        q[joint_id] = joint.getJointPosition();

    }

}

bool KinematicChain::getPositionReference(Eigen::VectorXd &q) const
{
    if (q.rows() != _joint_num) {
        q.resize(_joint_num);
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        q[pos++] = j->getPositionReference();
    }
    return true;
}

bool KinematicChain::getPositionReference(JointNameMap &q) const
{
//     q.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        q[joint_name] = joint.getPositionReference();

    }
}

bool KinematicChain::getPositionReference(JointIdMap &q) const
{
//     q.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        q[joint_id] = joint.getPositionReference();

    }
}

bool KinematicChain::getDamping(JointIdMap &D) const
{
//     D.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        D[joint_id] = joint.getDamping();

    }
}

bool KinematicChain::getJointEffort(JointIdMap &tau) const
{
//     tau.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        tau[joint_id] = joint.getJointEffort();

    }
}

bool KinematicChain::getEffortReference(JointIdMap &tau) const
{
//     tau.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        tau[joint_id] = joint.getEffortReference();

    }
}

bool KinematicChain::getJointVelocity(JointIdMap &qdot) const
{
//     qdot.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        qdot[joint_id] = joint.getJointVelocity();

    }
}

bool XBot::KinematicChain::getJointAcceleration(JointIdMap& qddot) const
{
    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        qddot[joint_id] = joint.getJointAcceleration();

    }
}


bool KinematicChain::getMotorPosition(JointIdMap &q) const
{
//     q.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        q[joint_id] = joint.getMotorPosition();

    }
}

bool KinematicChain::getMotorVelocity(JointIdMap &qdot) const
{
//     qdot.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        qdot[joint_id] = joint.getMotorVelocity();

    }
}

bool KinematicChain::getStiffness(JointIdMap &K) const
{
//     K.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        K[joint_id] = joint.getStiffness();

    }
}

bool KinematicChain::getTemperature(JointIdMap &temp) const
{
//     temp.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        temp[joint_id] = joint.getTemperature();

    }
}

bool KinematicChain::getVelocityReference(JointIdMap &qdot) const
{
//     qdot.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        qdot[joint_id] = joint.getVelocityReference();

    }
}

bool KinematicChain::getDamping(Eigen::VectorXd &D) const
{
    if (D.rows() != _joint_num) {
        D.resize(_joint_num);
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        D[pos++] = j->getDamping();
    }
    return true;
}

bool KinematicChain::getJointEffort(Eigen::VectorXd &tau) const
{
    if (tau.rows() != _joint_num) {
        tau.resize(_joint_num);
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        tau[pos++] = j->getJointEffort();
    }
    return true;
}

bool KinematicChain::getEffortReference(Eigen::VectorXd &tau) const
{
    if (tau.rows() != _joint_num) {
        tau.resize(_joint_num);
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        tau[pos++] = j->getEffortReference();
    }
    return true;
}

bool KinematicChain::getJointVelocity(Eigen::VectorXd &qdot) const
{
    if (qdot.rows() != _joint_num) {
        qdot.resize(_joint_num);
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        qdot[pos++] = j->getJointVelocity();
    }
    return true;
}

bool XBot::KinematicChain::getJointAcceleration(Eigen::VectorXd& qddot) const
{
    if (qddot.rows() != _joint_num) {
        qddot.resize(_joint_num);
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        qddot[pos++] = j->getJointAcceleration();
    }
    return true;
}


bool KinematicChain::getMotorPosition(Eigen::VectorXd &q) const
{
    if (q.rows() != _joint_num) {
        q.resize(_joint_num);
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        q[pos++] = j->getMotorPosition();
    }
    return true;
}

bool KinematicChain::getMotorVelocity(Eigen::VectorXd &qdot) const
{
    if (qdot.rows() != _joint_num) {
        qdot.resize(_joint_num);
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        qdot[pos++] = j->getMotorVelocity();
    }
    return true;
}

bool KinematicChain::getStiffness(Eigen::VectorXd &K) const
{
    if (K.rows() != _joint_num) {
        K.resize(_joint_num);
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        K[pos++] = j->getStiffness();
    }
    return true;
}

bool KinematicChain::getTemperature(Eigen::VectorXd &temp) const
{
    if (temp.rows() != _joint_num) {
        temp.resize(_joint_num);
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        temp[pos++] = j->getTemperature();
    }
    return true;
}

bool KinematicChain::getVelocityReference(Eigen::VectorXd &qdot) const
{
    if (qdot.rows() != _joint_num) {
        qdot.resize(_joint_num);
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        qdot[pos++] = j->getVelocityReference();
    }
    return true;
}




double KinematicChain::getPositionReference(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getPositionReference();
}

double KinematicChain::getJointEffort(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getJointEffort();
}

double KinematicChain::getEffortReference(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getEffortReference();
}

double KinematicChain::getJointVelocity(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getJointVelocity();
}

double XBot::KinematicChain::getJointAcceleration(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getJointAcceleration();
}


double KinematicChain::getMotorPosition(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getMotorPosition();
}

double KinematicChain::getMotorVelocity(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getMotorVelocity();
}

double KinematicChain::getStiffness(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getStiffness();
}

double KinematicChain::getTemperature(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getTemperature();
}

double KinematicChain::getVelocityReference(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getVelocityReference();
}

double KinematicChain::getDamping(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getDamping();
}




bool KinematicChain::setJointPosition(const Eigen::VectorXd &q)
{
    if (q.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : q has wrong size " << q.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        j->setJointPosition(q[pos++]);
    }
    return true;
}


bool KinematicChain::setJointEffort(const Eigen::VectorXd &tau)
{
    if (tau.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : tau has wrong size " << tau.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        j->setJointEffort(tau[pos++]);
    }
    return true;
}

bool KinematicChain::setEffortReference(const Eigen::VectorXd &tau)
{
    if (tau.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : tau has wrong size " << tau.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        j->setEffortReference(tau[pos++]);
    }
    return true;
}

bool KinematicChain::setDamping(const Eigen::VectorXd &D)
{
    if (D.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : D has wrong size " << D.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        j->setDamping(D[pos++]);
    }
    return true;
}

bool KinematicChain::setJointVelocity(const Eigen::VectorXd &qdot)
{
    if (qdot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qdot has wrong size " << qdot.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        j->setJointVelocity(qdot[pos++]);
    }
    return true;
}

bool XBot::KinematicChain::setJointAcceleration(const Eigen::VectorXd& qddot)
{
    if (qddot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qddot has wrong size " << qddot.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        j->setJointAcceleration(qddot[pos++]);
    }
    return true;
}


bool KinematicChain::setMotorPosition(const Eigen::VectorXd &q)
{
    if (q.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : q has wrong size " << q.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        j->setMotorPosition(q[pos++]);
    }
    return true;
}

bool KinematicChain::setMotorVelocity(const Eigen::VectorXd &qdot)
{
    if (qdot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qdot has wrong size " << qdot.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        j->setMotorVelocity(qdot[pos++]);
    }
    return true;
}

bool KinematicChain::setPositionReference(const Eigen::VectorXd &q)
{
    if (q.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : q has wrong size " << q.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        j->setPositionReference(q[pos++]);
    }
    return true;
}

bool KinematicChain::setStiffness(const Eigen::VectorXd &K)
{
    if (K.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : K has wrong size " << K.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        j->setStiffness(K[pos++]);
    }
    return true;
}

bool KinematicChain::setTemperature(const Eigen::VectorXd &temp)
{
    if (temp.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : temp has wrong size " << temp.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        j->setTemperature(temp[pos++]);
    }
    return true;
}

bool KinematicChain::setVelocityReference(const Eigen::VectorXd &qdot)
{
    if (qdot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qdot has wrong size " << qdot.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for (const XBot::Joint::Ptr & j : _joint_vector) {
        j->setVelocityReference(qdot[pos++]);
    }
    return true;
}






bool KinematicChain::setJointPosition(int i, double q)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setJointPosition(q);
}

bool KinematicChain::setJointEffort(int i, double tau)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setJointEffort(tau);
}

bool KinematicChain::setJointVelocity(int i, double qdot)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setJointVelocity(qdot);
}

bool XBot::KinematicChain::setJointAcceleration(int i, double qddot)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setJointAcceleration(qddot);
}


bool KinematicChain::setMotorPosition(int i, double q)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setMotorPosition(q);
}

bool KinematicChain::setMotorVelocity(int i, double qdot)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setMotorVelocity(qdot);
}

bool KinematicChain::setTemperature(int i, double temp)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setTemperature(temp);
}



bool KinematicChain::setDamping(const JointNameMap &D)
{
    bool success = false;

    for( const auto& joint_pair : _joint_name_map ){
        const std::string& joint_name = joint_pair.first;
        auto it = D.find(joint_name);
        if( it != D.end() ){
            joint_pair.second->setDamping(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setJointEffort(const JointNameMap &tau)
{
    bool success = false;

    for( const auto& joint_pair : _joint_name_map ){
        const std::string& joint_name = joint_pair.first;
        auto it = tau.find(joint_name);
        if( it != tau.end() ){
            joint_pair.second->setJointEffort(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setEffortReference(const JointNameMap &tau)
{
    bool success = false;

    for( const auto& joint_pair : _joint_name_map ){
        const std::string& joint_name = joint_pair.first;
        auto it = tau.find(joint_name);
        if( it != tau.end() ){
            joint_pair.second->setEffortReference(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setJointPosition(const JointNameMap &q)
{
    bool success = false;

    for( const auto& joint_pair : _joint_name_map ){
        const std::string& joint_name = joint_pair.first;
        auto it = q.find(joint_name);
        if( it != q.end() ){
            joint_pair.second->setJointPosition(it->second);
            success = true;
        }
    }

    return success;

}

bool KinematicChain::setJointVelocity(const JointNameMap &qdot)
{
    bool success = false;

    for( const auto& joint_pair : _joint_name_map ){
        const std::string& joint_name = joint_pair.first;
        auto it = qdot.find(joint_name);
        if( it != qdot.end() ){
            joint_pair.second->setJointVelocity(it->second);
            success = true;
        }
    }

    return success;
}

bool XBot::KinematicChain::setJointAcceleration(const JointNameMap& qddot)
{
    bool success = false;

    for( const auto& joint_pair : _joint_name_map ){
        const std::string& joint_name = joint_pair.first;
        auto it = qddot.find(joint_name);
        if( it != qddot.end() ){
            joint_pair.second->setJointAcceleration(it->second);
            success = true;
        }
    }

    return success;
}


bool KinematicChain::setMotorPosition(const JointNameMap &q)
{
    bool success = false;

    for( const auto& joint_pair : _joint_name_map ){
        const std::string& joint_name = joint_pair.first;
        auto it = q.find(joint_name);
        if( it != q.end() ){
            joint_pair.second->setMotorPosition(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setMotorVelocity(const JointNameMap &qdot)
{
    bool success = false;

    for( const auto& joint_pair : _joint_name_map ){
        const std::string& joint_name = joint_pair.first;
        auto it = qdot.find(joint_name);
        if( it != qdot.end() ){
            joint_pair.second->setMotorVelocity(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setPositionReference(const JointNameMap &q)
{
    bool success = false;

    for( const auto& joint_pair : _joint_name_map ){
        const std::string& joint_name = joint_pair.first;
        auto it = q.find(joint_name);
        if( it != q.end() ){
            joint_pair.second->setPositionReference(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setStiffness(const JointNameMap &K)
{
    bool success = false;

    for( const auto& joint_pair : _joint_name_map ){
        const std::string& joint_name = joint_pair.first;
        auto it = K.find(joint_name);
        if( it != K.end() ){
            joint_pair.second->setStiffness(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setTemperature(const JointNameMap &temp)
{
    bool success = false;

    for( const auto& joint_pair : _joint_name_map ){
        const std::string& joint_name = joint_pair.first;
        auto it = temp.find(joint_name);
        if( it != temp.end() ){
            joint_pair.second->setTemperature(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setVelocityReference(const JointNameMap &qdot)
{
    bool success = false;

    for( const auto& joint_pair : _joint_name_map ){
        const std::string& joint_name = joint_pair.first;
        auto it = qdot.find(joint_name);
        if( it != qdot.end() ){
            joint_pair.second->setVelocityReference(it->second);
            success = true;
        }
    }

    return success;}

bool KinematicChain::setDamping(const JointIdMap &D)
{
    bool success = false;

    for( const auto& joint_pair : _joint_id_map ){
        auto it = D.find(joint_pair.first);
        if( it != D.end() ){
            joint_pair.second->setDamping(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setJointEffort(const JointIdMap &tau)
{
    bool success = false;

    for( const auto& joint_pair : _joint_id_map ){
        auto it = tau.find(joint_pair.first);
        if( it != tau.end() ){
            joint_pair.second->setJointEffort(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setEffortReference(const JointIdMap &tau)
{
    bool success = false;

    for( const auto& joint_pair : _joint_id_map ){
        auto it = tau.find(joint_pair.first);
        if( it != tau.end() ){
            joint_pair.second->setEffortReference(it->second);
            success = true;
        }
    }

    return success;
}


bool KinematicChain::setJointPosition(const JointIdMap &q)
{
    bool success = false;

    for( const auto& joint_pair : _joint_id_map ){
        auto it = q.find(joint_pair.first);
        if( it != q.end() ){
            joint_pair.second->setJointPosition(it->second);
            success = true;
        }
    }

    return success;
}


bool KinematicChain::setJointVelocity(const JointIdMap &qdot)
{
    bool success = false;

    for( const auto& joint_pair : _joint_id_map ){
        auto it = qdot.find(joint_pair.first);
        if( it != qdot.end() ){
            joint_pair.second->setJointVelocity(it->second);
            success = true;
        }
    }

    return success;
}

bool XBot::KinematicChain::setJointAcceleration(const JointIdMap& qddot)
{
    bool success = false;

    for( const auto& joint_pair : _joint_id_map ){
        auto it = qddot.find(joint_pair.first);
        if( it != qddot.end() ){
            joint_pair.second->setJointAcceleration(it->second);
            success = true;
        }
    }

    return success;
}


bool KinematicChain::setMotorPosition(const JointIdMap &q)
{
    bool success = false;

    for( const auto& joint_pair : _joint_id_map ){
        auto it = q.find(joint_pair.first);
        if( it != q.end() ){
            joint_pair.second->setMotorPosition(it->second);
            success = true;
        }
    }

    return success;
}



bool KinematicChain::setMotorVelocity(const JointIdMap &qdot)
{
    bool success = false;

    for( const auto& joint_pair : _joint_id_map ){
        auto it = qdot.find(joint_pair.first);
        if( it != qdot.end() ){
            joint_pair.second->setMotorVelocity(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setPositionReference(const JointIdMap &q)
{
    bool success = false;

    for( const auto& joint_pair : _joint_id_map ){
        auto it = q.find(joint_pair.first);
        if( it != q.end() ){
            joint_pair.second->setPositionReference(it->second);
            success = true;
        }
    }

    return success;
}


bool KinematicChain::setStiffness(const JointIdMap &K)
{
    bool success = false;

    for( const auto& joint_pair : _joint_id_map ){
        auto it = K.find(joint_pair.first);
        if( it != K.end() ){
            joint_pair.second->setStiffness(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setTemperature(const JointIdMap &temp)
{
    bool success = false;

    for( const auto& joint_pair : _joint_id_map ){
        auto it = temp.find(joint_pair.first);
        if( it != temp.end() ){
            joint_pair.second->setTemperature(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setVelocityReference(const JointIdMap &qdot)
{
    bool success = false;

    for( const auto& joint_pair : _joint_id_map ){
        auto it = qdot.find(joint_pair.first);
        if( it != qdot.end() ){
            joint_pair.second->setVelocityReference(it->second);
            success = true;
        }
    }

    return success;
}

bool KinematicChain::setPositionReference(int i, double q)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setPositionReference(q);
}


bool KinematicChain::setVelocityReference(int i, double qdot)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setVelocityReference(qdot);
}


bool KinematicChain::setEffortReference(int i, double tau)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setEffortReference(tau);
}

bool KinematicChain::setStiffness(int i, double K)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setStiffness(K);
}

bool KinematicChain::setDamping(int i, double D)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << getChainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setDamping(D);
}


std::ostream& operator<< ( std::ostream& os, const KinematicChain& c )
{
    os << "Chain name: " << c._chain_name << std::endl;
    for( const auto& j : c._joint_vector ) {
        os << *j << std::endl;
    }
    return os;
}

const std::vector< int >& XBot::KinematicChain::getJointIds() const
{
    return _ordered_joint_id;
}

const std::vector< std::string >& XBot::KinematicChain::getJointNames() const
{
    return _ordered_joint_name;
}

XBot::Joint::ConstPtr XBot::KinematicChain::getJointById(int id) const
{
    auto it = _joint_id_map.find(id);
    if( it != _joint_id_map.end() ){
        return it->second;
    }
    else{
        std::cerr << "ERROR chain " << _chain_name << " does not contain joint ID " << id << "!" << std::endl;
        return XBot::Joint::ConstPtr();
    }
}

XBot::Joint::Ptr XBot::KinematicChain::getJointByName(const std::string& joint_name)
{
    auto it = _joint_name_map.find(joint_name);
    if( it != _joint_name_map.end() ){
        return it->second;
    }
    else{
        std::cerr << "ERROR chain " << _chain_name << " does not contain joint " << joint_name << "!" << std::endl;
        return XBot::Joint::Ptr();
    }
}

XBot::Joint::ConstPtr XBot::KinematicChain::getJointByName(const std::string& joint_name) const
{
    auto it = _joint_name_map.find(joint_name);
    if( it != _joint_name_map.end() ){
        return it->second;
    }
    else{
        std::cerr << "ERROR chain " << _chain_name << " does not contain joint " << joint_name << "!" << std::endl;
        return XBot::Joint::ConstPtr();
    }
}

bool XBot::KinematicChain::hasJoint(const std::string& joint_name) const
{
    return _joint_name_map.count(joint_name) > 0;
}

bool XBot::KinematicChain::hasJoint(int joint_id) const
{
    return _joint_id_map.count(joint_id) > 0;
}

void KinematicChain::getEffortLimits(Eigen::VectorXd& tau_max) const
{
    if(isVirtual()){
        tau_max.setConstant(_joint_num, std::numeric_limits<double>::max());
        return;
    }

    tau_max.resize(getJointNum());

    for(int i = 0; i < _joint_num; i++){
        tau_max(i) = _urdf_joints[i]->limits->effort;
    }
}


void KinematicChain::getJointLimits(Eigen::VectorXd& q_min, Eigen::VectorXd& q_max) const
{
    if(isVirtual()){
        q_min.setConstant(_joint_num, -std::numeric_limits<double>::max());
        q_max.setConstant(_joint_num, std::numeric_limits<double>::max());
        return;
    }


    q_min.resize(_joint_num);
    q_max.resize(_joint_num);

    for(int i = 0; i < _joint_num; i++){
        q_min(i) = _urdf_joints[i]->limits->lower;
        q_max(i) = _urdf_joints[i]->limits->upper;
    }
}

void KinematicChain::getVelocityLimits(Eigen::VectorXd& qdot_max) const
{
    if(isVirtual()){
        qdot_max.setConstant(_joint_num, std::numeric_limits<double>::max());
        return;
    }

    qdot_max.resize(getJointNum());

    for(int i = 0; i < _joint_num; i++){
        qdot_max(i) = _urdf_joints[i]->limits->velocity;
    }
}


bool KinematicChain::checkEffortLimits(const Eigen::VectorXd& tau) const
{
    if(isVirtual()) return true;

    for(int i = 0; i < _joint_num; i++){
        if( std::abs(tau(i)) > _urdf_joints[i]->limits->effort ){
            return false;
        }
    }

    return true;
}

bool KinematicChain::checkEffortLimits(const Eigen::VectorXd& tau,
                                       std::vector< std::string >& violating_joints) const
{
    if(isVirtual()) return true;

    bool success = true;
//     violating_joints.clear();

    for(int i = 0; i < _joint_num; i++){
        if( std::abs(tau(i)) > _urdf_joints[i]->limits->effort ){
            success = false;
            violating_joints.push_back(getJointName(i));
        }
    }

    return success;
}

bool KinematicChain::checkJointLimits(const Eigen::VectorXd& q) const
{
    if(isVirtual()) return true;

    for(int i = 0; i < _joint_num; i++){
        if( q(i) > _urdf_joints[i]->limits->upper || q(i) < _urdf_joints[i]->limits->lower ){
            return false;
        }
    }

    return true;
}

bool KinematicChain::checkJointLimits(const Eigen::VectorXd& q,
                                      std::vector< std::string >& violating_joints) const
{
    if(isVirtual()) return true;

    bool success = true;
//     violating_joints.clear();
    for(int i = 0; i < _joint_num; i++){
        if( q(i) > _urdf_joints[i]->limits->upper || q(i) < _urdf_joints[i]->limits->lower ){
            success = false;
            violating_joints.push_back(getJointName(i));
        }
    }

    return success;
}

bool KinematicChain::checkVelocityLimits(const Eigen::VectorXd& qdot) const
{
    if(isVirtual()) return true;

    for(int i = 0; i < _joint_num; i++){
        if( std::abs(qdot(i)) > _urdf_joints[i]->limits->velocity ){
            return false;
        }
    }

    return true;
}

bool KinematicChain::checkVelocityLimits(const Eigen::VectorXd& qdot,
                                         std::vector< std::string >& violating_joints) const
{
    if(isVirtual()) return true;

    bool success = true;
//     violating_joints.clear();

    for(int i = 0; i < _joint_num; i++){
        if( std::abs(qdot(i)) > _urdf_joints[i]->limits->velocity ){
            success = false;
            violating_joints.push_back(getJointName(i));
        }
    }

    return success;
}

void KinematicChain::getEffortLimits(int i, double& tau_max) const
{
    tau_max = _urdf_joints[i]->limits->effort;
}

void KinematicChain::getJointLimits(int i, double& q_min, double& q_max) const
{
    q_min = _urdf_joints[i]->limits->lower;
    q_max = _urdf_joints[i]->limits->upper;
}

void KinematicChain::getVelocityLimits(int i, double& qdot_max) const
{
    qdot_max = _urdf_joints[i]->limits->velocity;
}

Joint::Ptr XBot::KinematicChain::getJointInternal(int i) const
{
    if(_joint_vector.size() <= i){
        std::cerr << "ERROR in " << __func__ << "chain " << getChainName() << "has less than " << i+1 << " joints!" << std::endl;
        return Joint::Ptr();
    }

    return _joint_vector[i];
}

Joint::ConstPtr KinematicChain::getJoint ( int i ) const
{
    return getJointInternal(i);
}


ForceTorqueSensor::ConstPtr XBot::KinematicChain::getForceTorque(const std::string& link_name) const
{
    bool success = false;

    for( const ForceTorqueSensor::Ptr& ftptr : _ft_vector ){
        if( ftptr->getParentLinkName() == link_name ){
            success = true;
            return ftptr;
        }
    }

    if(!success){
        std::cerr << "ERROR in " << __func__ << " " << link_name << " is either undefined or does not contain any FT" << std::endl;
    }

    return ForceTorqueSensor::ConstPtr();
}

std::map< std::string, ForceTorqueSensor::ConstPtr > XBot::KinematicChain::getForceTorque() const
{
    std::map< std::string, ForceTorqueSensor::ConstPtr > ft_constptr_map;
    for( const ForceTorqueSensor::Ptr& ftptr : _ft_vector ){
        ft_constptr_map[ftptr->getSensorName()] = ftptr;
    }
    return ft_constptr_map;
}

std::map< std::string, ForceTorqueSensor::Ptr > KinematicChain::getForceTorqueInternal() const
{
    return _ft_map;
}


ImuSensor::ConstPtr XBot::KinematicChain::getImu(const std::string& link_name) const
{
    bool success = false;

    for( const ImuSensor::Ptr& imuptr : _imu_vector ){
        if( imuptr->getParentLinkName() == link_name ){
            success = true;
            return imuptr;
        }
    }

    if(!success){
        std::cerr << "ERROR in " << __func__ << " " << link_name << " is either undefined or does not contain any IMU" << std::endl;
    }

    return ImuSensor::ConstPtr();
}

std::map< std::string, ImuSensor::ConstPtr > XBot::KinematicChain::getImu() const
{
    std::map< std::string, ImuSensor::ConstPtr > imu_constptr_map;
    for( const ImuSensor::Ptr& imuptr : _imu_vector ){
        imu_constptr_map[imuptr->getSensorName()] = imuptr;
    }
    return imu_constptr_map;
}

std::map< std::string, ImuSensor::Ptr > KinematicChain::getImuInternal() const
{
    return _imu_map;
}


bool XBot::KinematicChain::getChainState(const std::string& state_name,
                                         Eigen::VectorXd& q) const
{

    q.resize(getJointNum());

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

                int dof_index = getChainDofIndex(joint_name);
                q[dof_index] = joint_value[0];

            }

        }

    }

    if(!success){
        std::cerr << "ERROR in " << __func__ << ": required group state " << state_name << " is NOT defined as a group state for the whole robot. Check in the SRDF that " << state_name << " is defined for the chains group" << std::endl;
    }

    return success;
}

bool XBot::KinematicChain::getChainState(const std::string& state_name,
                                         JointNameMap& q) const
{

    bool success = false;
    for( const auto& state : _XBotModel.getGroupStates() ){

        if( state.group_ == getChainName() && state.name_ == state_name ){

            success = true;

            for( const auto& value_pair : state.joint_values_ ){

                const std::string& joint_name = value_pair.first;
                const std::vector<double>& joint_value = value_pair.second;

                if( joint_value.size() != 1 ){
                    std::cerr << "ERROR in " << __func__ << ": multi-dof joints not supported!" << std::endl;
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

bool XBot::KinematicChain::getChainState(const std::string& state_name,
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

int XBot::KinematicChain::getChainDofIndex(const std::string& joint_name) const
{
    for(int i = 0; i < getJointNum(); i++){
        if( _ordered_joint_name[i] == joint_name ) return i;
    }

    std::cerr << "ERROR in " << __func__ << "! Joint " << joint_name << " NOT defined in chain " << getChainName() << "!" << std::endl;

    return -1;
}

int XBot::KinematicChain::getChainDofIndex(int joint_id) const
{
    for(int i = 0; i < getJointNum(); i++){
        if( _ordered_joint_id[i] == joint_id ) return i;
    }

    std::cerr << "ERROR in " << __func__ << "! Joint ID" << joint_id << " NOT defined in chain " << getChainName() << "!" << std::endl;

    return -1;
}

void XBot::KinematicChain::print() const
{
    bprinter::TablePrinter tp(&std::cout);
    tp.AddColumn("Name", 10);
    tp.AddColumn("ID", 4);
    tp.AddColumn("Link pos", 8);
    tp.AddColumn("Mot pos", 7);
    tp.AddColumn("Link vel", 8);
    tp.AddColumn("Mot vel", 7);
    tp.AddColumn("Effort", 6);
    tp.AddColumn("Temp", 4);
    tp.AddColumn("Pos ref", 7);
    tp.AddColumn("Vel ref", 7);
    tp.AddColumn("Eff ref", 7);
    tp.AddColumn("K", 5);
    tp.AddColumn("D", 5);

    tp.PrintHeader();
    for(int i = 0; i < _joint_num; i++){
        tp << getJointName(i) << getJointId(i) << getJointPosition(i) << getMotorPosition(i) << getJointVelocity(i) << getMotorVelocity(i) << getJointEffort(i) << getTemperature(i) << getPositionReference(i) << getVelocityReference(i) << getEffortReference(i) << getStiffness(i) << getDamping(i);
    }
    tp.PrintFooter();
}

void XBot::KinematicChain::printTracking() const
{
    bprinter::TablePrinter tp(&std::cout);
    tp.AddColumn("Name", 10);
    tp.AddColumn("ID", 4);
    tp.AddColumn("Link pos", 8);
    tp.AddColumn("Pos ref", 7);
    tp.AddColumn("Pos err", 7);
    tp.AddColumn("Pos err%", 8);
    tp.AddColumn("Effort", 8);
    tp.AddColumn("Tau ref", 7);
    tp.AddColumn("Tau err", 7);
    tp.AddColumn("Tau err%", 8);
    tp.AddColumn("Link vel", 8);
    tp.AddColumn("Vel ref", 7);
    tp.AddColumn("Vel err", 7);
    tp.AddColumn("Vel err%", 8);

    tp.PrintHeader();
    for( int i = 0; i < _joint_num; i++ ){

        double pos_err = getPositionReference(i) - getJointPosition(i);
        double pos_err_rel = pos_err / getPositionReference(i);
        double tau_err = getEffortReference(i) - getJointEffort(i);
        double tau_err_rel = tau_err / getEffortReference(i);
        double vel_err = getVelocityReference(i) - getJointVelocity(i);
        double vel_err_rel = vel_err / getVelocityReference(i);


        tp << getJointName(i) << getJointId(i) << getJointPosition(i) << getPositionReference(i) << pos_err << pos_err_rel*100 << getJointEffort(i) << getEffortReference(i) << tau_err << tau_err_rel*100 << getJointVelocity(i) << getVelocityReference(i) << vel_err << vel_err_rel*100;

    }
    tp.PrintFooter();
}

bool XBot::KinematicChain::enforceEffortLimit(Eigen::VectorXd& tau) const
{
    if( tau.size() != getJointNum() ){
        std::cerr << "ERROR in " << __func__ << "! Provided vector has " << tau.size() << " elements != " << getJointNum() << " the size of the chain!" << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _joint_vector[i]->enforceEffortLimit(tau(i));
    }

    return true;
}

bool XBot::KinematicChain::enforceJointLimits(Eigen::VectorXd& q) const
{
    if( q.size() != getJointNum() ){
        std::cerr << "ERROR in " << __func__ << "! Provided vector has " << q.size() << " elements != " << getJointNum() << " the size of the chain!" << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _joint_vector[i]->enforceJointLimits(q(i));
    }

    return true;
}

bool XBot::KinematicChain::enforceVelocityLimit(Eigen::VectorXd& qdot) const
{
    if( qdot.size() != getJointNum() ){
        std::cerr << "ERROR in " << __func__ << "! Provided vector has " << qdot.size() << " elements != " << getJointNum() << " the size of the chain!" << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        _joint_vector[i]->enforceVelocityLimit(qdot(i));
    }

    return true;
}

bool XBot::KinematicChain::eigenToMap(const Eigen::VectorXd& eigen_vector, JointNameMap& namemap) const
{
    if( eigen_vector.size() != _joint_num ){
        std::cerr << "ERROR in " << __func__ << "! Input vector has " << eigen_vector.size() << "!=" << getJointNum() << " elements!" << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        const Joint& j = *_joint_vector[i];
        namemap[j.getJointName()] = eigen_vector(i);
    }

    return true;
}

bool XBot::KinematicChain::eigenToMap(const Eigen::VectorXd& eigen_vector, JointIdMap& idmap) const
{
    if( eigen_vector.size() != _joint_num ){
        std::cerr << "ERROR in " << __func__ << "! Input vector has " << eigen_vector.size() << "!=" << getJointNum() << " elements!" << std::endl;
        return false;
    }

    for(int i = 0; i < _joint_num; i++){
        const Joint& j = *_joint_vector[i];
        idmap[j.getJointId()] = eigen_vector(i);
    }

    return true;
}

bool XBot::KinematicChain::mapToEigen(const JointNameMap& namemap, Eigen::VectorXd& eigen_vector) const
{
    bool success = true;

    if( eigen_vector.size() != getJointNum() ){
        eigen_vector.setZero(getJointNum());
    }

    for(const auto& id_pair : namemap){
        auto it = _joint_name_map.find(id_pair.first);
        if( it != _joint_name_map.end() ){
            success = true;
            eigen_vector(getChainDofIndex(it->first)) = id_pair.second;
        }
    }

    return success;
}

bool XBot::KinematicChain::mapToEigen(const JointIdMap& idmap, Eigen::VectorXd& eigen_vector) const
{
    bool success = true;

    if( eigen_vector.size() != getJointNum() ){
        eigen_vector.setZero(getJointNum());
    }

    for(const auto& id_pair : idmap){
        auto it = _joint_id_map.find(id_pair.first);
        if( it != _joint_id_map.end() ){
            success = true;
            eigen_vector(getChainDofIndex(it->first)) = id_pair.second;
        }
    }

    return success;
}



} // end namespace XBot
