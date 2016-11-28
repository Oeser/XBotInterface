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

#include <XBotInterface/ControlMode.h>

namespace XBot {
    
    
ControlMode::ControlMode(bool position_enabled, 
                         bool velocity_enabled, 
                         bool effort_enabled, 
                         bool stiffness_enabled, 
                         bool damping_enabled):
_position_enabled(position_enabled),
_velocity_enabled(velocity_enabled),
_effort_enabled(effort_enabled),
_stiffness_enabled(stiffness_enabled),
_damping_enabled(damping_enabled),
_name("")
{

}

ControlMode::ControlMode(const std::string& name, bool position_enabled, bool velocity_enabled, bool effort_enabled, bool stiffness_enabled, bool damping_enabled):
_position_enabled(position_enabled),
_velocity_enabled(velocity_enabled),
_effort_enabled(effort_enabled),
_stiffness_enabled(stiffness_enabled),
_damping_enabled(damping_enabled),
_name(name)
{

}



ControlMode ControlMode::operator+(const ControlMode& ctrl_mode)
{
    if(_name == "IDLE"){
        _name = ctrl_mode._name;
    }
    else if(ctrl_mode._name != "IDLE"){
        _name = _name + std::string("+") + ctrl_mode._name;
    }
    
    _position_enabled = _position_enabled || ctrl_mode._position_enabled;
    _velocity_enabled = _velocity_enabled || ctrl_mode._velocity_enabled;
    _effort_enabled = _effort_enabled || ctrl_mode._effort_enabled;
    _stiffness_enabled = _stiffness_enabled || ctrl_mode._stiffness_enabled;
    _damping_enabled = _damping_enabled || ctrl_mode._damping_enabled;
    
    return *this;
}

bool ControlMode::isDampingEnabled() const
{
    return _damping_enabled;
}

bool ControlMode::isEffortEnabled() const
{
    return _effort_enabled;
}

bool ControlMode::isPositionEnabled() const
{
    return _position_enabled;
}

bool ControlMode::isStiffnessEnabled() const
{
    return _stiffness_enabled;
}

bool ControlMode::isVelocityEnabled() const
{
    return _velocity_enabled;
}

bool ControlMode::setDampingEnabled(bool is_enabled)
{
    _damping_enabled = is_enabled;
}

bool ControlMode::setEffortEnabled(bool is_enabled)
{
    _effort_enabled = is_enabled;
}

bool ControlMode::setPositionEnabled(bool is_enabled)
{
    _position_enabled = is_enabled;
}

bool ControlMode::setStiffnessEnabled(bool is_enabled)
{
    _stiffness_enabled = is_enabled;
}

bool ControlMode::setVelocityEnabled(bool is_enabled)
{
    _velocity_enabled = is_enabled;
}

ControlMode ControlMode::Damping()
{
    ControlMode ctrl_mode("DAMPING");
    ctrl_mode.setDampingEnabled(true);
    return ctrl_mode;
}

ControlMode ControlMode::Effort()
{
    ControlMode ctrl_mode("EFFORT");
    ctrl_mode.setEffortEnabled(true);
    return ctrl_mode;
}

ControlMode ControlMode::PosImpedance()
{
    ControlMode ctrl_mode("POS_IMPEDANCE");
    ctrl_mode.setPositionEnabled(true);
    ctrl_mode.setStiffnessEnabled(true);
    ctrl_mode.setDampingEnabled(true);
    return ctrl_mode;
}

ControlMode ControlMode::Position()
{
    ControlMode ctrl_mode("POSITION");
    ctrl_mode.setPositionEnabled(true);
    return ctrl_mode;
}

ControlMode ControlMode::Stiffness()
{
    ControlMode ctrl_mode("STIFFNESS");
    ctrl_mode.setStiffnessEnabled(true);
    return ctrl_mode;
}

ControlMode ControlMode::Velocity()
{
    ControlMode ctrl_mode("VELOCITY");
    ctrl_mode.setVelocityEnabled(true);
    return ctrl_mode;
}

ControlMode XBot::ControlMode::Idle()
{
    return ControlMode("IDLE");
}


bool XBot::ControlMode::operator==(const ControlMode& ctrl_mode)
{
    return (_position_enabled == ctrl_mode._position_enabled) && (_velocity_enabled == ctrl_mode._velocity_enabled) && (_effort_enabled == ctrl_mode._effort_enabled) && (_stiffness_enabled == ctrl_mode._stiffness_enabled) && (_damping_enabled == ctrl_mode._damping_enabled);
}

const std::string& XBot::ControlMode::getName() const
{
    return _name;
}











    
}