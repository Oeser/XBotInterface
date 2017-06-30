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
#include <mutex>

namespace XBot {

class ImuSensor : public GenericSensor {
    
public:
    
    typedef std::shared_ptr<ImuSensor> Ptr;
    typedef std::shared_ptr<const ImuSensor> ConstPtr;

    /**
     * @brief Default constructor
     * 
     */
    ImuSensor();
    
     /**
     * @brief Copy constructor
     *
     */
    ImuSensor(const ImuSensor& i);
    
    /**
     * @brief operator =
     *
     */
    const ImuSensor& operator=(const ImuSensor& i);
    
    /**
     * @brief  Constructor with the sensor parent link
     * 
     * @param sensor_link the sensor parent link
     * @param sensor_id the sensor id
     */
    ImuSensor(urdf::LinkConstSharedPtr sensor_link, int sensor_id);
    
    /**
     * @brief Getter for the IMU orientation as a quaternion 
     * 
     * @param orientation the IMU orientation as a quaternion 
     * @return void
     */
    void getOrientation(Eigen::Quaterniond& orientation) const;
    
    /**
     * @brief Getter for the IMU orientation as a rotation matrix 
     * 
     * @param orientation the IMU orientation as a rotation matrix 
     * @return void
     */
    void getOrientation(Eigen::Matrix3d& orientation) const;
    
    /**
     * @brief Getter for the IMU linear acceleration
     * 
     * @param acceleration the IMU linear acceleration
     * @return void
     */
    void getLinearAcceleration(Eigen::Vector3d& acceleration) const;
    
    /**
     * @brief Getter for the IMU angular velocity
     * 
     * @param angular_velocity the IMU angular velocity
     * @return void
     */
    void getAngularVelocity(Eigen::Vector3d& angular_velocity) const;
    
    /**
     * @brief Getter for the IMU data i.e. orientation(as a quaternion), linear acceleration and angular velocity
     * 
     * @param orientation the IMU orientation as a quaternion 
     * @param acceleration the IMU linear acceleration
     * @param angular_velocity the IMU angular velocity
     * @return void
     */
    void getImuData(Eigen::Quaterniond& orientation, 
                    Eigen::Vector3d& acceleration,
                    Eigen::Vector3d& angular_velocity) const;
                    
   /**
    * @brief Getter for the IMU data i.e. orientation(as a rotation matrix), linear acceleration and angular velocity
    * 
    * @param orientation the IMU orientation as a rotation matrix
    * @param accelerationthe the IMU linear acceleration
    * @param angular_velocity the IMU angular velocity
    * @return void
    */
    void getImuData(Eigen::Matrix3d& orientation, 
                    Eigen::Vector3d& acceleration,
                    Eigen::Vector3d& angular_velocity) const;
                    
   /**
    * @brief Setter for the IMU data i.e. orientation(as a quaternion), linear acceleration, angular velocity and a timestamp
    * 
    * @param orientation the IMU orientation as a quaternion to set
    * @param acceleration the IMU linear acceleration to set
    * @param angular_velocity the IMU angular velocity to set
    * @param timestamp IMU data reading timestamp to set
    * @return void
    */
    void setImuData(Eigen::Quaterniond& orientation, 
                    Eigen::Vector3d& acceleration,
                    Eigen::Vector3d& angular_velocity,
                    double timestamp
                   );
    
   /**
    * @brief Setter for the IMU data i.e. orientation(as a rotation matrix), linear acceleration, angular velocity and a timestamp
    * 
    * @param orientation the IMU orientation as a rotation matrix to set
    * @param acceleration the IMU linear acceleration to set
    * @param angular_velocity the IMU angular velocity to set
    * @param timestamp IMU data reading timestamp to set
    * @return void
    */
    void setImuData(Eigen::Matrix3d& orientation, 
                    Eigen::Vector3d& acceleration,
                    Eigen::Vector3d& angular_velocity,
                    double timestamp
                   );

    friend std::ostream& operator<< ( std::ostream& os, const XBot::ImuSensor& j );
    
protected:
    
private:
    
    Eigen::Matrix3d _orientation;
    Eigen::Vector3d _lin_acc, _angular_velocity;
    mutable std::mutex _mutex;
    
};

}

#endif