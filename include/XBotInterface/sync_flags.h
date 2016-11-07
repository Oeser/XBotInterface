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

namespace XBot{
 
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
    
    
    inline void parseFlags(bool& pos, 
                           bool& vel, 
                           bool& acc, 
                           bool& eff,
                           bool& k,
                           bool& d,
                           Sync s = Sync::All){
        
        pos = (s == Sync::Position) || (s == Sync::All);
        vel = (s == Sync::Velocity) || (s == Sync::All);
        acc = (s == Sync::Acceleration) || (s == Sync::All);
        eff = (s == Sync::Effort) || (s == Sync::All);
        k = (s == Sync::Stiffness) || (s == Sync::Impedance) || (s == Sync::All);
        d = (s == Sync::Damping) || (s == Sync::Impedance) || (s == Sync::All);
        
    }

    template <typename... Flags>
    inline void parseFlags(bool& pos, 
                           bool& vel, 
                           bool& acc, 
                           bool& eff, 
                           bool& k,
                           bool& d,
                           Sync s, Flags... rest){
        
        parseFlags(pos, vel, acc, eff, k, d, rest...);
        
        pos = pos || (s == Sync::Position);
        vel = vel || (s == Sync::Velocity);
        acc = acc || (s == Sync::Acceleration);
        eff = eff || (s == Sync::Effort);
        k = k || (s == Sync::Stiffness);
        d = d || (s == Sync::Damping);
        
    }
    

    
}

#endif