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
#include <XBotInterface/GenericSensor.h>

namespace XBot {

    class KinematicChain;
    
    /**
     * @brief Force - Torque sensor abstraction: TBD documentation of the single functions
     * 
     */
    class ForceTorqueSensor : public GenericSensor {
    
    
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
    
    friend std::ostream& operator<< ( std::ostream& os, const XBot::ForceTorqueSensor& j );
    
    // NOTE if you are a const ptr you cannot call them ///////
    void setWrench(const Eigen::Matrix<double, 6, 1>& wrench, double timestamp);
    void setForce(const Eigen::Vector3d& force, double timestamp);
    void setTorque(const Eigen::Vector3d& torque, double timestamp);
    //////////////////////////////////////////////////////////

private:
    

    double _fx, _fy, _fz, _tx, _ty, _tz;
    
    
};
}
#endif