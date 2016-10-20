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

namespace XBot
{

KinematicChain::KinematicChain() :
    _chain_name("dummy_chain"),
    _joint_num(-1),
    _is_virtual(false)
{

}

KinematicChain::KinematicChain(const std::string &chain_name,
                               const XBot::XBotCoreModel &XBotModel) :
    _chain_name(chain_name),
    _XBotModel(XBotModel),
    _is_virtual(false)
{
    _joint_num = _XBotModel.get_joint_num(chain_name);
    _XBotModel.get_enabled_joints_in_chain(chain_name, _ordered_joint_name);
    _XBotModel.get_enabled_joint_ids_in_chain(chain_name, _ordered_joint_id);
    // resize joint vector
    _joint_vector.resize(_joint_num);
    // initialize the joint maps and vector
    for (int i = 0; i < _joint_num; i++) {
        std::string actual_joint_name = _ordered_joint_name[i];
        int actual_joint_id = _ordered_joint_id[i];
        XBot::Joint::Ptr actual_joint = std::make_shared<Joint>(actual_joint_name,
                                                                actual_joint_id,
                                                                _chain_name);
        _joint_name_map[actual_joint_name] = actual_joint;
        _joint_id_map[actual_joint_id] = actual_joint;
        _joint_vector[i] = actual_joint;
    }

    // Get urdf model
    const urdf::ModelInterface &robot_urdf = *XBotModel.get_urdf_model();

    // Fill _urdf_joints and _urdf_links
    // NOTE: is it the correct way to handle links? What happens if some joint
    // along the chain is disabled/fixed/...?
    for (const std::string & joint_name : _ordered_joint_name) {

        _urdf_joints.push_back(robot_urdf.getJoint(joint_name));

        const std::string &parent_link_name = robot_urdf.getJoint(joint_name)->parent_link_name;
        _urdf_links.push_back(robot_urdf.getLink(parent_link_name));

    }

    // Add last link to _urdf_links
    _urdf_links.push_back(robot_urdf.getLink(childLinkName(getJointNum() - 1)));

}

KinematicChain::KinematicChain(const std::string& chain_name):
_chain_name(chain_name),
_is_virtual(false)
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
    _is_virtual(false)
{


    for (const Joint::Ptr & other_jptr : other._joint_vector) {

        Joint::Ptr jptr = std::make_shared<Joint>();
        *jptr = *other_jptr;

        _joint_vector.push_back(jptr);
        _joint_name_map[jptr->getJointName()] = jptr;
        _joint_id_map[jptr->getJointId()] = jptr;

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

    // Return this
    return *this;

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
    if(joint->getJointId() < 0) _is_virtual = true;
}

const std::vector< XBot::JointConstSharedPtr > &KinematicChain::getJoints() const
{
    return _urdf_joints;
}

const std::vector< XBot::LinkConstSharedPtr > &KinematicChain::getLinks() const
{
    return _urdf_links;
}

int KinematicChain::getJointNum() const
{
    return _joint_vector.size();
}

const std::string &KinematicChain::baseLinkName() const
{
    return _urdf_links[0]->name;
}

const std::string &KinematicChain::tipLinkName() const
{
    return _urdf_links[_urdf_links.size() - 1]->name;
}

const std::string &KinematicChain::chainName() const
{
    return _chain_name;
}

int KinematicChain::jointId(int i) const
{
    return _joint_vector[i]->getJointId();
}



const std::string &KinematicChain::childLinkName(int id) const
{
    return _urdf_joints[id]->child_link_name;
}

const std::string &KinematicChain::parentLinkName(int id) const
{
    return _urdf_joints[id]->parent_link_name;
}

const std::string &KinematicChain::jointName(int id) const
{
    return _urdf_joints[id]->name;
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

bool KinematicChain::getJointPosition(std::map< std::string, double > &q) const
{
//     q.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        q[joint_name] = joint.getJointPosition();

    }

}

bool KinematicChain::getDamping(std::map< std::string, double > &D) const
{
//     D.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        D[joint_name] = joint.getDamping();

    }
}

bool KinematicChain::getJointEffort(std::map< std::string, double > &tau) const
{
//     tau.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        tau[joint_name] = joint.getJointEffort();

    }
}

bool KinematicChain::getEffortReference(std::map< std::string, double > &tau) const
{
//     tau.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        tau[joint_name] = joint.getEffortReference();

    }
}

bool KinematicChain::getJointVelocity(std::map< std::string, double > &qdot) const
{
//     qdot.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        qdot[joint_name] = joint.getJointVelocity();

    }
}

bool KinematicChain::getMotorPosition(std::map< std::string, double > &q) const
{
//     q.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        q[joint_name] = joint.getMotorPosition();

    }
}

bool KinematicChain::getMotorVelocity(std::map< std::string, double > &qdot) const
{
//     qdot.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        qdot[joint_name] = joint.getMotorVelocity();

    }
}

bool KinematicChain::getStiffness(std::map< std::string, double > &K) const
{
//     K.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        K[joint_name] = joint.getStiffness();

    }
}

bool KinematicChain::getTemperature(std::map< std::string, double > &temp) const
{
//     temp.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        temp[joint_name] = joint.getTemperature();

    }
}

bool KinematicChain::getVelocityReference(std::map< std::string, double > &qdot) const
{
//     qdot.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        qdot[joint_name] = joint.getVelocityReference();

    }
}




bool KinematicChain::getJointPosition(std::map< int, double > &q) const
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

bool KinematicChain::getPositionReference(std::map< std::string, double > &q) const
{
//     q.clear();

    for (const auto & name_joint_pair : _joint_name_map) {

        const std::string &joint_name = name_joint_pair.first;
        const Joint &joint = *name_joint_pair.second;

        q[joint_name] = joint.getPositionReference();

    }
}

bool KinematicChain::getPositionReference(std::map< int, double > &q) const
{
//     q.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        q[joint_id] = joint.getPositionReference();

    }
}

bool KinematicChain::getDamping(std::map< int, double > &D) const
{
//     D.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        D[joint_id] = joint.getDamping();

    }
}

bool KinematicChain::getJointEffort(std::map< int, double > &tau) const
{
//     tau.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        tau[joint_id] = joint.getJointEffort();

    }
}

bool KinematicChain::getEffortReference(std::map< int, double > &tau) const
{
//     tau.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        tau[joint_id] = joint.getEffortReference();

    }
}

bool KinematicChain::getJointVelocity(std::map< int, double > &qdot) const
{
//     qdot.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        qdot[joint_id] = joint.getJointVelocity();

    }
}

bool KinematicChain::getMotorPosition(std::map< int, double > &q) const
{
//     q.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        q[joint_id] = joint.getMotorPosition();

    }
}

bool KinematicChain::getMotorVelocity(std::map< int, double > &qdot) const
{
//     qdot.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        qdot[joint_id] = joint.getMotorVelocity();

    }
}

bool KinematicChain::getStiffness(std::map< int, double > &K) const
{
//     K.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        K[joint_id] = joint.getStiffness();

    }
}

bool KinematicChain::getTemperature(std::map< int, double > &temp) const
{
//     temp.clear();

    for (const auto & id_joint_pair : _joint_id_map) {

        int joint_id = id_joint_pair.first;
        const Joint &joint = *id_joint_pair.second;

        temp[joint_id] = joint.getTemperature();

    }
}

bool KinematicChain::getVelocityReference(std::map< int, double > &qdot) const
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
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getPositionReference();
}

double KinematicChain::getJointEffort(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getJointEffort();
}

double KinematicChain::getEffortReference(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getEffortReference();
}

double KinematicChain::getJointVelocity(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getJointVelocity();
}

double KinematicChain::getMotorPosition(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getMotorPosition();
}

double KinematicChain::getMotorVelocity(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getMotorVelocity();
}

double KinematicChain::getStiffness(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getStiffness();
}

double KinematicChain::getTemperature(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getTemperature();
}

double KinematicChain::getVelocityReference(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index + 1 << " joints!" << std::endl;
    }
    return _joint_vector[index]->getVelocityReference();
}

double KinematicChain::getDamping(int index) const
{
    if (index >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index + 1 << " joints!" << std::endl;
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
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setJointPosition(q);
}

bool KinematicChain::setJointEffort(int i, double tau)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setJointEffort(tau);
}

bool KinematicChain::setJointVelocity(int i, double qdot)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setJointVelocity(qdot);
}

bool KinematicChain::setMotorPosition(int i, double q)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setMotorPosition(q);
}

bool KinematicChain::setMotorVelocity(int i, double qdot)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setMotorVelocity(qdot);
}

bool KinematicChain::setTemperature(int i, double temp)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setTemperature(temp);
}




bool KinematicChain::sync(const KinematicChain &other)
{
    // TBD: check that chains are indeed omologues??
    int pos = 0;
    for (const XBot::Joint::Ptr & j : other._joint_vector) {
        _joint_vector[pos++]->syncFrom(*j);
    }
}


bool KinematicChain::setDamping(const std::map< std::string, double > &D)
{
    bool success = true;

    for (const auto & jointname_value_pair : D) {
        const std::string &joint_name = jointname_value_pair.first;
        if (_joint_name_map.count(joint_name)) {
            _joint_name_map.at(joint_name)->setDamping(jointname_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_name << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setJointEffort(const std::map< std::string, double > &tau)
{
    bool success = true;

    for (const auto & jointname_value_pair : tau) {
        const std::string &joint_name = jointname_value_pair.first;
        if (_joint_name_map.count(joint_name)) {
            _joint_name_map.at(joint_name)->setJointEffort(jointname_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_name << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setEffortReference(const std::map< std::string, double > &tau)
{
    bool success = true;

    for (const auto & jointname_value_pair : tau) {
        const std::string &joint_name = jointname_value_pair.first;
        if (_joint_name_map.count(joint_name)) {
            _joint_name_map.at(joint_name)->setEffortReference(jointname_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_name << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setJointPosition(const std::map< std::string, double > &q)
{
    bool success = true;

    for (const auto & jointname_value_pair : q) {
        const std::string &joint_name = jointname_value_pair.first;
        if (_joint_name_map.count(joint_name)) {
            _joint_name_map.at(joint_name)->setJointPosition(jointname_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_name << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setJointVelocity(const std::map< std::string, double > &qdot)
{
    bool success = true;

    for (const auto & jointname_value_pair : qdot) {
        const std::string &joint_name = jointname_value_pair.first;
        if (_joint_name_map.count(joint_name)) {
            _joint_name_map.at(joint_name)->setJointVelocity(jointname_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_name << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setMotorPosition(const std::map< std::string, double > &q)
{
    bool success = true;

    for (const auto & jointname_value_pair : q) {
        const std::string &joint_name = jointname_value_pair.first;
        if (_joint_name_map.count(joint_name)) {
            _joint_name_map.at(joint_name)->setMotorPosition(jointname_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_name << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setMotorVelocity(const std::map< std::string, double > &qdot)
{
    bool success = true;

    for (const auto & jointname_value_pair : qdot) {
        const std::string &joint_name = jointname_value_pair.first;
        if (_joint_name_map.count(joint_name)) {
            _joint_name_map.at(joint_name)->setMotorVelocity(jointname_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_name << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setPositionReference(const std::map< std::string, double > &q)
{
    bool success = true;

    for (const auto & jointname_value_pair : q) {
        const std::string &joint_name = jointname_value_pair.first;
        if (_joint_name_map.count(joint_name)) {
            _joint_name_map.at(joint_name)->setPositionReference(jointname_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_name << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setStiffness(const std::map< std::string, double > &K)
{
    bool success = true;

    for (const auto & jointname_value_pair : K) {
        const std::string &joint_name = jointname_value_pair.first;
        if (_joint_name_map.count(joint_name)) {
            _joint_name_map.at(joint_name)->setStiffness(jointname_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_name << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setTemperature(const std::map< std::string, double > &temp)
{
    bool success = true;

    for (const auto & jointname_value_pair : temp) {
        const std::string &joint_name = jointname_value_pair.first;
        if (_joint_name_map.count(joint_name)) {
            _joint_name_map.at(joint_name)->setTemperature(jointname_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_name << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setVelocityReference(const std::map< std::string, double > &qdot)
{
    bool success = true;

    for (const auto & jointname_value_pair : qdot) {
        const std::string &joint_name = jointname_value_pair.first;
        if (_joint_name_map.count(joint_name)) {
            _joint_name_map.at(joint_name)->setVelocityReference(jointname_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_name << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setDamping(const std::map< int, double > &D)
{
    bool success = true;

    for (const auto & jointid_value_pair : D) {
        int joint_id = jointid_value_pair.first;
        if (_joint_id_map.count(joint_id)) {
            _joint_id_map.at(joint_id)->setDamping(jointid_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint with ID " << joint_id << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setJointEffort(const std::map< int, double > &tau)
{
    bool success = true;

    for (const auto & jointid_value_pair : tau) {
        int joint_id = jointid_value_pair.first;
        if (_joint_id_map.count(joint_id)) {
            _joint_id_map.at(joint_id)->setJointEffort(jointid_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_id << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setEffortReference(const std::map< int, double > &tau)
{
    bool success = true;

    for (const auto & jointid_value_pair : tau) {
        int joint_id = jointid_value_pair.first;
        if (_joint_id_map.count(joint_id)) {
            _joint_id_map.at(joint_id)->setEffortReference(jointid_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_id << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}


bool KinematicChain::setJointPosition(const std::map< int, double > &q)
{
    bool success = true;

    for (const auto & jointid_value_pair : q) {
        int joint_id = jointid_value_pair.first;
        if (_joint_id_map.count(joint_id)) {
            _joint_id_map.at(joint_id)->setJointPosition(jointid_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_id << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}


bool KinematicChain::setJointVelocity(const std::map< int, double > &qdot)
{
    bool success = true;

    for (const auto & jointid_value_pair : qdot) {
        int joint_id = jointid_value_pair.first;
        if (_joint_id_map.count(joint_id)) {
            _joint_id_map.at(joint_id)->setJointVelocity(jointid_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_id << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setMotorPosition(const std::map< int, double > &q)
{
    bool success = true;

    for (const auto & jointid_value_pair : q) {
        int joint_id = jointid_value_pair.first;
        if (_joint_id_map.count(joint_id)) {
            _joint_id_map.at(joint_id)->setMotorPosition(jointid_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_id << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}



bool KinematicChain::setMotorVelocity(const std::map< int, double > &qdot)
{
    bool success = true;

    for (const auto & jointid_value_pair : qdot) {
        int joint_id = jointid_value_pair.first;
        if (_joint_id_map.count(joint_id)) {
            _joint_id_map.at(joint_id)->setMotorVelocity(jointid_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_id << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setPositionReference(const std::map< int, double > &q)
{
    bool success = true;

    for (const auto & jointid_value_pair : q) {
        int joint_id = jointid_value_pair.first;
        if (_joint_id_map.count(joint_id)) {
            _joint_id_map.at(joint_id)->setPositionReference(jointid_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_id << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}


bool KinematicChain::setStiffness(const std::map< int, double > &K)
{
    bool success = true;

    for (const auto & jointid_value_pair : K) {
        int joint_id = jointid_value_pair.first;
        if (_joint_id_map.count(joint_id)) {
            _joint_id_map.at(joint_id)->setStiffness(jointid_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_id << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setTemperature(const std::map< int, double > &temp)
{
    bool success = true;

    for (const auto & jointid_value_pair : temp) {
        int joint_id = jointid_value_pair.first;
        if (_joint_id_map.count(joint_id)) {
            _joint_id_map.at(joint_id)->setTemperature(jointid_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_id << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setVelocityReference(const std::map< int, double > &qdot)
{
    bool success = true;

    for (const auto & jointid_value_pair : qdot) {
        int joint_id = jointid_value_pair.first;
        if (_joint_id_map.count(joint_id)) {
            _joint_id_map.at(joint_id)->setVelocityReference(jointid_value_pair.second);
        } else {
            success = false;
            std::cerr << "ERROR in function " << __func__ << "! Joint " << joint_id << " is NOT defined!!" << std::endl;
        }

    }

    return success;
}

bool KinematicChain::setPositionReference(int i, double q)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setPositionReference(q);
}


bool KinematicChain::setVelocityReference(int i, double qdot)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setVelocityReference(qdot);
}


bool KinematicChain::setEffortReference(int i, double tau)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setEffortReference(tau);
}

bool KinematicChain::setStiffness(int i, double K)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i + 1 << " joints!" << std::endl;
        return false;
    }
    _joint_vector[i]->setStiffness(K);
}

bool KinematicChain::setDamping(int i, double D)
{
    if (i >= this->getJointNum()) {
        std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i + 1 << " joints!" << std::endl;
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

const std::vector< int >& XBot::KinematicChain::jointIds() const
{
    return _ordered_joint_id;
}

const std::vector< std::string >& XBot::KinematicChain::jointNames() const
{
    return _ordered_joint_name;
}

XBot::Joint::ConstPtr XBot::KinematicChain::getJointById(int id) const
{
    if(_joint_id_map.count(id)){
        return _joint_id_map.at(id);
    }
    else{
        std::cerr << "ERROR chain " << _chain_name << " does not contain joint ID " << id << "!" << std::endl;
        return XBot::Joint::ConstPtr();
    }
}

XBot::Joint::ConstPtr XBot::KinematicChain::getJointByName(const std::string& joint_name) const
{
    if(_joint_name_map.count(joint_name)){
        return _joint_name_map.at(joint_name);
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

bool XBot::KinematicChain::hasJoint(int id) const
{
    return _joint_id_map.count(id) > 0;
}



} // end namespace XBot
