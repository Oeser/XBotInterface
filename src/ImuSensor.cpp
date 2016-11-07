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

#include <XBotInterface/ImuSensor.h>

namespace XBot {
    
ImuSensor::ImuSensor():
_lin_acc(0,0,0), _angular_velocity(0,0,0), _orientation(Eigen::Matrix3d::Identity())
{

}

ImuSensor::ImuSensor(urdf::LinkConstSharedPtr sensor_link): 
GenericSensor(sensor_link),
_lin_acc(0,0,0), _angular_velocity(0,0,0), _orientation(Eigen::Matrix3d::Identity())
{

}


void ImuSensor::getAngularVelocity(Eigen::Vector3d& angular_velocity) const
{
    angular_velocity = _angular_velocity;
}

void ImuSensor::getImuData(Eigen::Matrix3d& orientation, 
                           Eigen::Vector3d& acceleration, 
                           Eigen::Vector3d& angular_velocity) const
{
    orientation = _orientation;
    acceleration = _lin_acc;
    angular_velocity = _angular_velocity;
}

void ImuSensor::getImuData(Eigen::Quaterniond& orientation, 
                           Eigen::Vector3d& acceleration, 
                           Eigen::Vector3d& angular_velocity) const
{
    orientation = Eigen::Quaterniond(_orientation);
    acceleration = _lin_acc;
    angular_velocity = _angular_velocity;
}

void ImuSensor::getLinearAcceleration(Eigen::Vector3d& acceleration) const
{
    acceleration = _lin_acc;
}

void ImuSensor::getOrientation(Eigen::Matrix3d& orientation) const
{
    orientation = _orientation;
}

void ImuSensor::getOrientation(Eigen::Quaterniond& orientation) const
{
    orientation = Eigen::Quaterniond(_orientation);
}

void ImuSensor::setImuData(Eigen::Matrix3d& orientation, 
                           Eigen::Vector3d& acceleration,
                           Eigen::Vector3d& angular_velocity)
{
    _orientation = orientation;
    _lin_acc = acceleration;
    _angular_velocity = angular_velocity;
}

void ImuSensor::setImuData(Eigen::Quaterniond& orientation, 
                           Eigen::Vector3d& acceleration,
                           Eigen::Vector3d& angular_velocity)
{
    _orientation = orientation.toRotationMatrix();
    _lin_acc = acceleration;
    _angular_velocity = angular_velocity;
}

    
    
    
    
}