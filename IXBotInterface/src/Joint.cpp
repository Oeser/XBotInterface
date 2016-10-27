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

#include <XBotInterface/Joint.h>

XBot::Joint::Joint() :
    _joint_name(""),
    _joint_id(-1),
    _chain_name("")
{
    init();
}

XBot::Joint::Joint ( const std::string& joint_name, 
                     int joint_id, 
                     urdf::JointConstSharedPtr urdf_joint,
                     const std::string& chain_name ) :
    _joint_name(joint_name),
    _joint_id(joint_id),
    _urdf_joint(urdf_joint),
    _chain_name(chain_name)
{
    init();
}

void XBot::Joint::init()
{
    ///////////////////
    // RX from board //
    ///////////////////

    _link_pos = 0;
    _motor_pos = 0;
    _link_vel = 0;
    _motor_vel = 0;
    _effort = 0;
    _temperature = 0;

    /////////////////
    // TX to board //
    /////////////////

    _pos_ref = 0;
    _vel_ref = 0;
    _effort_ref = 0;
    _stiffness = 0;
    _damping = 0;
}

bool XBot::Joint::isVirtualJoint()
{
    return (_joint_id < 0);
}


const std::string &XBot::Joint::getJointName() const
{
    return _joint_name;
}

int XBot::Joint::getJointId() const
{
    return _joint_id;
}

const std::string& XBot::Joint::getChainName() const
{
    return _chain_name;
}


void XBot::Joint::setJointName(const std::string &joint_name)
{
    _joint_name = joint_name;
}

void XBot::Joint::setJointId(int joint_id)
{
    _joint_id = joint_id;
}

double XBot::Joint::getJointPosition() const
{
    return _link_pos;
}

double XBot::Joint::getMotorPosition() const
{
    return _motor_pos;
}

double XBot::Joint::getJointVelocity() const
{
    return _link_vel;
}

double XBot::Joint::getMotorVelocity() const
{
    return _motor_vel;
}

double XBot::Joint::getJointEffort() const
{
    return _effort;
}

double XBot::Joint::getTemperature() const
{
    return _temperature;
}


void XBot::Joint::setJointPosition(double link_pos)
{
    _link_pos = link_pos;
}

void XBot::Joint::setMotorPosition(double motor_pos)
{
    _motor_pos = motor_pos;
}

void XBot::Joint::setJointVelocity(double link_vel)
{
    _link_vel = link_vel;
}

void XBot::Joint::setMotorVelocity(double motor_vel)
{
    _motor_vel = motor_vel;
}

void XBot::Joint::setJointEffort(double effort)
{
    _effort = effort;
}

void XBot::Joint::setTemperature(double temperature)
{
    _temperature = temperature;
}

double XBot::Joint::getPositionReference() const
{
    return _pos_ref;
}

double XBot::Joint::getVelocityReference() const
{
    return _vel_ref;
}

double XBot::Joint::getEffortReference() const
{
    return _effort_ref;
}

double XBot::Joint::getStiffness() const
{
    return _stiffness;
}

double XBot::Joint::getDamping() const
{
    return _damping;
}

void XBot::Joint::setPositionReference(double pos_ref)
{
    _pos_ref = pos_ref;
}

void XBot::Joint::setVelocityReference(double vel_ref)
{
    _vel_ref = vel_ref;
}

void XBot::Joint::setEffortReference(double effort_ref)
{
    _effort_ref = effort_ref;
}

void XBot::Joint::setStiffness(double stiffness)
{
    _stiffness = stiffness;
}

void XBot::Joint::setDamping(double damping)
{
    _damping = damping;
}

bool XBot::Joint::syncFrom(const XBot::Joint &other)
{

    if(_joint_name != other._joint_name || _joint_id != other._joint_id){
        std::cerr << "ERROR in " << __func__ << "! Attempt to synchronize joints with different names/ids!" << std::endl;
        return false;
    }

    ///////////////////
    // RX from board //
    ///////////////////

    _link_pos = other._link_pos;
    _motor_pos = other._motor_pos;
    _link_vel = other._link_vel;
    _motor_vel = other._motor_vel;
    _effort = other._effort;
    _temperature = other._temperature;

    /////////////////
    // TX to board //
    /////////////////

    _pos_ref = other._pos_ref;
    _vel_ref = other._vel_ref;
    _effort_ref = other._effort_ref;
    _stiffness = other._stiffness;
    _damping = other._damping;
}

bool XBot::Joint::setReferenceFrom(const XBot::Joint& other)
{
    if(_joint_name != other._joint_name || _joint_id != other._joint_id){
        std::cerr << "ERROR in " << __func__ << "! Attempt to set reference from joint with different name/id!" << std::endl;
        return false;
    }


    /////////////////
    // TX to board //
    /////////////////

    _pos_ref = other._link_pos;
    _vel_ref = other._link_vel;
    _effort_ref = other._effort;
    _stiffness = other._stiffness;
    _damping = other._damping;
}


const XBot::Joint& XBot::Joint::operator<< ( const XBot::Joint& from )
{
    this->syncFrom(from);
    return *this;
}


std::ostream& XBot::operator<< ( std::ostream& os, const XBot::Joint& j ) 
{
    os << "Joint id: " << j.getJointId() << std::endl;
    os << "Joint name: " << j.getJointName() << std::endl;
    os << "RX values ###########" << std::endl;
    os << "\tLink position: " << j.getJointPosition() << std::endl;
    os << "\tMotor position: " << j.getMotorPosition() << std::endl;
    os << "\tLink velocity: " << j.getJointVelocity() << std::endl;
    os << "\tMotor velocity: " << j.getJointVelocity() << std::endl;
    os << "\tEffort: " << j.getJointEffort() << std::endl;
    os << "\tTemperature: " << j.getTemperature() << std::endl;
    
    os << "TX values ###########" << std::endl;
    os << "\tLink position ref: " << j.getPositionReference() << std::endl;
    os << "\tLink velocity ref: " << j.getVelocityReference() << std::endl;
    os << "\tEffort ref: " << j.getEffortReference() << std::endl;
    os << "\tStiffness: " << j.getStiffness() << std::endl;
    os << "\tDamping: " << j.getDamping() << std::endl;
    
    return os;
}


