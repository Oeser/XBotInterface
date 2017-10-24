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

#ifndef __XBOT_ENUM__
#define __XBOT_ENUM__

#include <map>
#include <unordered_map>
#include <string>
#include <memory>
#include <boost/any.hpp>
#include <eigen3/Eigen/Dense>

namespace XBot{

    class ImuSensor;
    class ForceTorqueSensor;


    /*////////////
    /* TYPEDEF  */
    ////////////*/

    /**
     * @brief std::map with key representing the joint ID (i.e. its numerical identifier; note that
     * IDs are not required to be consecutive) and value representing a joint state (e.g. position,
     * torque, ...)
     */
    typedef std::unordered_map<int, double> JointIdMap;

    /**
     * @brief std::map with key representing the joint human-readable name and value
     * representing a joint state (e.g. position, torque, ...)
     */
    typedef std::unordered_map<std::string, double> JointNameMap;

    /**
     * @brief std::map with key representing the FT sensor human-readable name and value
     * representing a const shared pointer to the FT itself.
     *
     */
    typedef std::map<std::string, std::shared_ptr<const ForceTorqueSensor> > ForceTorqueMap;

    /**
     * @brief std::map with key representing the IMU sensor human-readable name and value
     * representing a const shared pointer to the IMU itself.
     *
     */
    typedef std::map<std::string, std::shared_ptr<const ImuSensor> > ImuMap;

    /**
     * @brief std::map map with key representing a parameter name and value representing
     * the parameter value. Note that thanks to the use of boost::any it can be of any type.
     *
     */
    typedef std::map<std::string, boost::any> AnyMap;

    /**
     * @brief Shared pointer to AnyMap
     *
     */
    typedef std::shared_ptr<AnyMap> AnyMapPtr;

    /**
     * @brief Shared pointer to AnyMap
     *
     */
    typedef std::shared_ptr<const AnyMap> AnyMapConstPtr;

    //////////
    // ENUM //
    //////////

    /**
     * @brief enum class that represents a set of flags to specify what part of the robot/model state needs to be synchronized.
     * By default (i.e. if this argument is omitted) the whole state is used. Otherwise, an arbitrary number of flags
     * can be specified in order to select a subset of the state. The flags must be of the enum type
     * XBot::Sync, which can take the following values:
     * - Sync::Position,
     * - Sync::Velocity
     * - Sync::Acceleration
     * - Sync::Effort
     * - Sync::Stiffness
     * - Sync::Damping
     * - Sync::Impedance
     * - Sync::All
     * - Sync::Sensors
     * - Sync::Imu
     * - Sync::ForceTorque
     */
    enum class Sync {

        Position ,
        Velocity,
        Effort,
        Acceleration,
        Impedance,
        Stiffness,
        Damping,
        All ,
        Sensors,
        Imu,
        ForceTorque

    };

}

namespace Eigen {
    
    /**
     * @brief Typedef for a 6 element column vector of double.
     *
     */
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    
    /**
     * @brief Typedef for a 6-by-6 matrix of double.
     *
     */
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}

#endif