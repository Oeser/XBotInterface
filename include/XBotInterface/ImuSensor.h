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

#ifndef __XBOT__IMUSENSOR__
#define __XBOT__IMUSENSOR__

#include <XBotInterface/GenericSensor.h>
#include <Eigen/Geometry>

namespace XBot {

class ImuSensor : public GenericSensor {
    
public:
    
    typedef std::shared_ptr<ImuSensor> Ptr;
    
    typedef std::shared_ptr<ImuSensor> ConstPtr;
    
    ImuSensor(urdf::LinkConstSharedPtr sensor_link);

    ImuSensor();
    
    void getOrientation(Eigen::Quaterniond& orientation) const;
    
    void getOrientation(Eigen::Matrix3d& orientation) const;
    
    void getLinearAcceleration(Eigen::Vector3d& acceleration) const;
    
    void getAngularVelocity(Eigen::Vector3d& angular_velocity) const;
    
    void getImuData(Eigen::Quaterniond& orientation, 
                    Eigen::Vector3d& acceleration,
                    Eigen::Vector3d& angular_velocity) const;
                    
    void getImuData(Eigen::Matrix3d& orientation, 
                    Eigen::Vector3d& acceleration,
                    Eigen::Vector3d& angular_velocity) const;
                    
    void setImuData(Eigen::Quaterniond& orientation, 
                    Eigen::Vector3d& acceleration,
                    Eigen::Vector3d& angular_velocity);
    
    void setImuData(Eigen::Matrix3d& orientation, 
                    Eigen::Vector3d& acceleration,
                    Eigen::Vector3d& angular_velocity);

protected:
    
private:
    
    Eigen::Matrix3d _orientation;
    Eigen::Vector3d _lin_acc, _angular_velocity;
    
    
    
    
};

}

#endif