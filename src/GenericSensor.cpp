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

#include <XBotInterface/GenericSensor.h>

namespace XBot {
    
GenericSensor::GenericSensor(urdf::LinkConstSharedPtr sensor_link):
_timestamp(-1)
{
    
    _sensor_name = sensor_link->name;
    _parent_link_name = sensor_link->parent_joint->parent_link_name;
    
    urdf::Rotation rot = sensor_link->parent_joint->parent_to_joint_origin_transform.rotation;
    urdf::Vector3 pos = sensor_link->parent_joint->parent_to_joint_origin_transform.position;
    
    Eigen::Quaterniond quat;
    quat.x() = rot.x;
    quat.y() = rot.y;
    quat.z() = rot.z;
    quat.w() = rot.w;
    
    _parent_link_T_sensor_link.translation() << pos.x, pos.y, pos.z;
    _parent_link_T_sensor_link.linear() = quat.toRotationMatrix();
}

GenericSensor::GenericSensor():
_timestamp(-1)
{
    _parent_link_T_sensor_link.setIdentity();
}

const std::string& GenericSensor::getParentLinkName() const
{
    return _parent_link_name;
}

const std::string& GenericSensor::getSensorName() const
{
    return _sensor_name;
}
/*
KDL::Frame GenericSensor::sensorPose() const
{
    KDL::frame tmp_frame;
    tf::transformEigenToKDL(sensorPose(), tmp_frame);
    return tmp_frame;
}*/

const Eigen::Affine3d& GenericSensor::getSensorPose() const
{
    return _parent_link_T_sensor_link;
}

double XBot::GenericSensor::getTimestamp() const
{
    return _timestamp;
}

void XBot::GenericSensor::setTimestamp(double timestamp)
{
    _timestamp = timestamp;
}


}