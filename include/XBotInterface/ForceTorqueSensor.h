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
#include <atomic>

namespace XBot {

    class KinematicChain;
    
    /**
     * @brief Force - Torque sensor abstraction:
     * 
     */
    class ForceTorqueSensor : public GenericSensor {
    
    
public:
    
    typedef std::shared_ptr<ForceTorqueSensor> Ptr;
    typedef std::shared_ptr<const ForceTorqueSensor> ConstPtr;

    friend XBot::KinematicChain;
    
    /**
     * @brief Default Contructor
     * 
     */
    ForceTorqueSensor();
    
    /**
     * @brief Copy constructor
     *
     */
    ForceTorqueSensor(const ForceTorqueSensor& f);
    
    /**
     * @brief operator =
     *
     */
    const ForceTorqueSensor& operator=(const ForceTorqueSensor& f);
    
    /**
     * @brief Constructor with the Force-Torque parent link
     * 
     * @param ft_link the FOrce-Torque parent link
     * @param sensor_id the sensor id
     */
    ForceTorqueSensor(urdf::LinkConstSharedPtr ft_link, int sensor_id);
    
    /**
     * @brief Getter for the Force-Torque measured wrench
     * 
     * @param wrench the measured wrench that will be filled by this function as Eigen 6x1 double Matrix 
     * @return void
     */
    void getWrench(Eigen::Matrix<double, 6, 1>& wrench) const;
    
    /**
     * @brief Getter for the Force-Torque measured force 
     * 
     * @param force the measured force that will be filled by this function as Eigen Vector3d
     * @return void
     */
    void getForce(Eigen::Vector3d& force) const;

    /**
     * @brief Getter for the Force-Torque measured torque 
     * 
     * @param torque the measured torque that will be filled by this function as Eigen Vector3d
     * @return void
     */
    void getTorque(Eigen::Vector3d& torque) const;
    
    /**
     * @brief Getter for the Force-Torque measured wrench
     * 
     * @param wrench the measured wrench that will be filled by this function as KDL Wrench
     * @return void
     */
    void getWrench(KDL::Wrench& wrench) const;
    
    /**
     * @brief Getter for the Force-Torque measured force 
     * 
     * @param force the measured force that will be filled by this function as KDL Vector
     * @return void
     */
    void getForce(KDL::Vector& force) const;
    
    /**
     * @brief Getter for the Force-Torque measured torque 
     * 
     * @param torque the measured torque that will be filled by this function as KDL Vector
     * @return void
     */
    void getTorque(KDL::Vector& torque) const;
    
    friend std::ostream& operator<< ( std::ostream& os, const XBot::ForceTorqueSensor& j );
    
    /**
     * @brief Setter for the Force-Torque wrench.
     *        NOTE if you are a const ptr you cannot call it.
     * @param wrench the wrench to set
     * @param timestamp the timestamp related to the Force-Torque reading
     * @return void
     */
    void setWrench(const Eigen::Matrix<double, 6, 1>& wrench, double timestamp);
    
    /**
     * @brief Setter for the Force-Torque wrench.
     *        NOTE if you are a const ptr you cannot call it.
     * @param force the force to set
     * @param timestamp the timestamp related to the force reading
     * @return void
     */
    void setForce(const Eigen::Vector3d& force, double timestamp);
    
    /**
     * @brief Setter for the Force-Torque torque.
     *        NOTE if you are a const ptr you cannot call it.
     * @param torque the torque to set
     * @param timestamp the timestamp related to the torque reading
     * @return void
     */
    void setTorque(const Eigen::Vector3d& torque, double timestamp);

private:
    

    std::atomic<double> _fx, _fy, _fz, _tx, _ty, _tz;
    
    
};
}
#endif