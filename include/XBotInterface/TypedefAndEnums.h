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
#include <string>

namespace XBot{
    
    /*////////////
    /* TYPEDEF  */
    ////////////*/
    
    /**
     * @brief std::map with key representing the joint ID (i.e. its numerical identifier; note that
     * IDs are not required to be consecutive) and value representing a joint state (e.g. position,
     * torque, ...)
     */
    typedef std::map<int, double> JointIdMap;
    
    /**
     * @brief std::map with key representing the joint human-readable name and value 
     * representing a joint state (e.g. position, torque, ...)
     */
    typedef std::map<std::string, double> JointNameMap;
    
    
    //////////
    // ENUM //
    //////////
 
    enum class Sync {
        
        Position ,
        Velocity,
        Effort,
        Acceleration,
        Impedance,
        Stiffness,
        Damping,
        All 
        
    };

}

#endif