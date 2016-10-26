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

#ifndef __XBOT_FORCE_TORQUE_SENSOR_H__
#define __XBOT_FORCE_TORQUE_SENSOR_H__

#include <string>
#include <memory>
#include <iostream>
#include <kdl/kdl.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/Geometry>
#include <srdfdom_advr/model.h>

namespace XBot {

    class KinematicChain;
    
class ForceTorqueSensor {
    
    
public:
    
    typedef std::shared_ptr<ForceTorqueSensor> Ptr;
    typedef std::shared_ptr<const ForceTorqueSensor> ConstPtr;
    
    
    friend XBot::KinematicChain;
    
    ForceTorqueSensor();
    
    ForceTorqueSensor(urdf::LinkConstSharedPtr ft_link);
    
    void getWrench(Eigen::Matrix<double, 6, 1>& wrench) const;
    void getForce(Eigen::Vector3d& force) const;
    void getTorque(Eigen::Vector3d& torque) const;
    
    void getWrench(KDL::Wrench& wrench) const;
    void getForce(KDL::Vector& force) const;
    void getTorque(KDL::Vector& torque) const;
    
    const std::string& parentLinkName() const;
    const std::string& sensorName() const;
    const Eigen::Affine3d& sensorPose() const;
    

protected:
    
    void setWrench(const Eigen::Matrix<double, 6, 1>& wrench);
    void setForce(const Eigen::Vector3d& force);
    void setTorque(const Eigen::Vector3d& torque);


private:
    
    std::string _ft_name;
    std::string _parent_link_name;
    Eigen::Affine3d _parent_link_T_sensor_link;
    double _fx, _fy, _fz, _tx, _ty, _tz;
    
    
};
}
#endif