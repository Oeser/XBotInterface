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
    _chain_name(""),
    _control_mode(ControlMode::Idle())
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
    _chain_name(chain_name),
    _control_mode(ControlMode::Idle())
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
    _link_acc = 0;
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

bool XBot::Joint::isVirtualJoint() const
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

double XBot::Joint::getJointAcceleration() const
{
    return _link_acc;
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

void XBot::Joint::setJointAcceleration(double link_acc)
{
    _link_acc = link_acc;
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


bool XBot::Joint::checkEffortLimit(double tau) const
{
    return std::abs(tau) <= _urdf_joint->limits->effort;
}

bool XBot::Joint::checkJointLimits(double q) const
{
    return (q <= _urdf_joint->limits->upper) && (q >= _urdf_joint->limits->lower);
}

bool XBot::Joint::checkVelocityLimit(double qdot) const
{
    return std::abs(qdot) <= _urdf_joint->limits->velocity;
}


void XBot::Joint::getEffortLimit(double& tau_max) const
{
    tau_max = _urdf_joint->limits->effort;
}

void XBot::Joint::getJointLimits(double& qmin, double& qmax) const
{
    qmin = _urdf_joint->limits->lower;
    qmax = _urdf_joint->limits->upper;
}

void XBot::Joint::getVelocityLimit(double& qdot_max) const
{
    qdot_max = _urdf_joint->limits->velocity;
}


const XBot::Joint& XBot::Joint::operator<< ( const XBot::Joint& from )
{
    this->syncFrom(from);
    return *this;
}


void XBot::Joint::print() const
{
    bprinter::TablePrinter tp(&std::cout);
    tp.AddColumn("Name", 7);
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
    tp << getJointName() << getJointId() << getJointPosition() << getMotorPosition() << getJointVelocity() << getMotorVelocity() << getJointEffort() << getTemperature() << getPositionReference() << getVelocityReference() << getEffortReference() << getStiffness() << getDamping();
    tp.PrintFooter();
}

void XBot::Joint::printTracking() const
{
    bprinter::TablePrinter tp(&std::cout);
    tp.AddColumn("Name", 7);
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

    double pos_err = getPositionReference() - getJointPosition();
    double pos_err_rel = pos_err / getPositionReference();
    double tau_err = getEffortReference() - getJointEffort();
    double tau_err_rel = pos_err / getEffortReference();
    double vel_err = getVelocityReference() - getJointVelocity();
    double vel_err_rel = pos_err / getVelocityReference();

    tp.PrintHeader();
    tp << getJointName() << getJointId() << getJointPosition() << getPositionReference() << pos_err << pos_err_rel*100 << getJointEffort() << getEffortReference() << tau_err << tau_err_rel*100 << getJointVelocity() << getVelocityReference() << vel_err << vel_err_rel*100;
    tp.PrintFooter();
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

void XBot::Joint::enforceEffortLimit(double& tau) const
{
    if( tau < -_urdf_joint->limits->effort ){
        tau = -_urdf_joint->limits->effort;
        return;
    }

    if( tau > _urdf_joint->limits->effort ){
        tau = _urdf_joint->limits->effort;
        return;
    }
}

void XBot::Joint::enforceJointLimits(double& q) const
{
    if( q < _urdf_joint->limits->lower ){
        q = _urdf_joint->limits->lower;
        return;
    }

    if( q > _urdf_joint->limits->upper ){
        q = _urdf_joint->limits->upper;
        return;
    }
}

void XBot::Joint::enforceVelocityLimit(double& qdot) const
{
    if( qdot < -_urdf_joint->limits->velocity ){
        qdot = -_urdf_joint->limits->velocity;
        return;
    }

    if( qdot > _urdf_joint->limits->velocity ){
        qdot = _urdf_joint->limits->velocity;
        return;
    }
}

void XBot::Joint::getControlMode(XBot::ControlMode& control_mode) const
{
    control_mode = _control_mode;
}

void XBot::Joint::setControlMode(const XBot::ControlMode& control_mode)
{
    _control_mode = control_mode;
}

bool XBot::Joint::isFixedControlledJoint() const
{
    if( isVirtualJoint() ) return false;

    if ( _urdf_joint->type == urdf::Joint::FIXED ) {
        // NOTE if we arrive here we are 100% sure that the joint is specified in the SRDF controlled_joints group and it is FIXED
        return true;
    }

    return false;
}

urdf::JointConstSharedPtr XBot::Joint::getUrdfJoint() const
{
    return _urdf_joint;
}


