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

#include <XBotInterface/ForceTorqueSensor.h>

namespace XBot {
    
ForceTorqueSensor::ForceTorqueSensor(urdf::LinkConstSharedPtr ft_link):
GenericSensor::GenericSensor(ft_link),
_fx(0), _fy(0), _fz(0), _tx(0), _ty(0), _tz(0)
{

}

ForceTorqueSensor::ForceTorqueSensor():
GenericSensor::GenericSensor(),
_fx(0), _fy(0), _fz(0), _tx(0), _ty(0), _tz(0)
{

}

void ForceTorqueSensor::getForce(KDL::Vector& force) const
{
    force(0) = _fx;
    force(1) = _fy;
    force(2) = _fz;
}

void ForceTorqueSensor::getForce(Eigen::Vector3d& force) const
{
    force.x() = _fx;
    force.y() = _fy;
    force.z() = _fz;
}

void ForceTorqueSensor::getTorque(KDL::Vector& torque) const
{
    torque(0) = _tx;
    torque(1) = _ty;
    torque(2) = _tz;
}

void ForceTorqueSensor::getTorque(Eigen::Vector3d& torque) const
{
    torque.x() = _tx;
    torque.y() = _ty;
    torque.z() = _tz;
}

void ForceTorqueSensor::getWrench(KDL::Wrench& wrench) const
{
    wrench.force(0) = _fx;
    wrench.force(1) = _fy;
    wrench.force(2) = _fz;
    wrench.torque(0) = _tx;
    wrench.torque(1) = _ty;
    wrench.torque(2) = _tz;
}

void ForceTorqueSensor::getWrench(Eigen::Matrix< double, int(6), int(1) >& wrench) const
{
    wrench << _fx, _fy, _fz, _tx, _ty, _tz;
}


void ForceTorqueSensor::setForce(const Eigen::Vector3d& force, double timestamp)
{
    _fx = force.x();
    _fy = force.y();
    _fz = force.z();
    setTimestamp(timestamp);
}

void ForceTorqueSensor::setTorque(const Eigen::Vector3d& torque, double timestamp)
{
    _tx = torque.x();
    _ty = torque.y();
    _tz = torque.z();
    setTimestamp(timestamp);
}

void ForceTorqueSensor::setWrench(const Eigen::Matrix< double, int(6), int(1) >& wrench, double timestamp)
{
    _fx = wrench(0);
    _fy = wrench(1);
    _fz = wrench(2);
    _tx = wrench(3);
    _ty = wrench(4);
    _tz = wrench(5);
    setTimestamp(timestamp);
}

std::ostream& operator<< ( std::ostream& os, const XBot::ForceTorqueSensor& j ) 
{
    os << "FT name: " << j.getSensorName() << std::endl;
    os << "Parent link: " << j.getParentLinkName() << std::endl;
    os << "Sensor frame to parent link frame transform: " << std::endl;
    os << "Orientation: \n" << j.getSensorPose().linear() << std::endl;
    os << "Position: \n" << j.getSensorPose().translation() << std::endl;
    os << "Sensed values ###########" << std::endl;
    os << "\tFx: " << j._fx << std::endl;
    os << "\tFy: " << j._fy << std::endl;
    os << "\tFz: " << j._fz << std::endl;
    os << "\tTx: " << j._tx << std::endl;
    os << "\tTy: " << j._ty << std::endl;
    os << "\tTz: " << j._tz << std::endl;
    
    return os;
}

    
    
    
    
    
    
    
    
    
    
}