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

#ifndef __XBOT__GENERIC_SENSOR__
#define __XBOT__GENERIC_SENSOR__

#include <string>
#include <memory>
#include <iostream>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <srdfdom_advr/model.h>

namespace XBot {
 
    /**
     * @brief Generic sensor abstraction
     * 
     */
    class GenericSensor {
    
    public:
        
        typedef std::shared_ptr<GenericSensor> Ptr;
        typedef std::shared_ptr<const GenericSensor> ConstPtr;
        
        /**
         * @brief Default constructor
         * 
         */
        GenericSensor();
        
        /**
         * @brief Constructor with the sensor parent link
         * 
         * @param sensor_link the sensor parent link
         */
        GenericSensor(urdf::LinkConstSharedPtr sensor_link);
        
        /**
         * @brief Getter for the sensor parent link name
         * 
         * @return the parent link name
         */
        const std::string& getParentLinkName() const;
        
        /**
         * @brief Getter for the sensor name
         * 
         * @return the sensor name
         */
        const std::string& getSensorName() const;
        
        /**
         * @brief Getter for the sensor pose w.r.t. the parent link
         * 
         * @return the sensor pose w.r.t. the parent link
         */
        const Eigen::Affine3d& getSensorPose() const;
        
        /**
         * @brief Getter for the timestamp of the last sensor reading
         * 
         * @return the timestamp of the last sensor reading
         */
        double getTimestamp() const;
        
        /**
         * @brief Setter for the timestamp of the sensor reading
         * 
         * @param timestamp the timestamp to set
         * @return void
         */
        void setTimestamp(double timestamp);
        
        
    protected:
        
    private:
        
        std::string _sensor_name;
        std::string _parent_link_name;
        Eigen::Affine3d _parent_link_T_sensor_link;
        double _timestamp;
        
    };
}

#endif