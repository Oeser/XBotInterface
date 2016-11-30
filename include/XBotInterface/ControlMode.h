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

#ifndef __XBOT_XBOTINTERFACE_CONTROLMODE_H__
#define __XBOT_XBOTINTERFACE_CONTROLMODE_H__

#include <string>

namespace XBot{
 
    class ControlMode {
      
    public:
        
        ControlMode(bool position_enabled = false, 
                    bool velocity_enabled = false,
                    bool effort_enabled = false,
                    bool stiffness_enabled = false,
                    bool damping_enabled = false);
        
        ControlMode(const std::string& name,
                    bool position_enabled = false, 
                    bool velocity_enabled = false,
                    bool effort_enabled = false,
                    bool stiffness_enabled = false,
                    bool damping_enabled = false);
                    
        
        bool isPositionEnabled() const;
        bool isVelocityEnabled() const;
        bool isEffortEnabled() const;
        bool isStiffnessEnabled() const;
        bool isDampingEnabled() const;
        
        bool setPositionEnabled(bool is_enabled);
        bool setVelocityEnabled(bool is_enabled);
        bool setEffortEnabled(bool is_enabled);
        bool setStiffnessEnabled(bool is_enabled);
        bool setDampingEnabled(bool is_enabled);
        
        ControlMode operator+(const ControlMode& ctrl_mode);
        bool operator==(const ControlMode& ctrl_mode);
        
        const std::string& getName() const;
        
        static ControlMode Position();
        static ControlMode Velocity();
        static ControlMode Effort();
        static ControlMode PosImpedance();
        static ControlMode Stiffness();
        static ControlMode Damping();
        static ControlMode Idle();
        
        
        
        
    protected:
        
    private:
        
        std::string _name;
        
        bool _position_enabled;
        bool _velocity_enabled;
        bool _effort_enabled;
        bool _stiffness_enabled;
        bool _damping_enabled;
        
    };
    
    
    
}

#endif
