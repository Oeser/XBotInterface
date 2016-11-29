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


#ifndef _XBOT_ROBOTCHAIN_H_
#define _XBOT_ROBOTCHAIN_H_

#include "XBotInterface/KinematicChain.h"
#include "XBotInterface/ModelChain.h"

namespace XBot {

/**
* @brief Kinematic chain useful for a robot abstraction
* 
*/
class RobotChain : public KinematicChain {
    
public:
    
    typedef std::shared_ptr<RobotChain> Ptr;
    
    RobotChain();

    
    // Getters for RX

    using KinematicChain::getJointPosition;
    using KinematicChain::getMotorPosition;
    using KinematicChain::getJointVelocity;
    using KinematicChain::getMotorVelocity;
    using KinematicChain::getJointEffort;
    using KinematicChain::getTemperature;

    
    // Getters for TX

    using KinematicChain::getPositionReference;
    using KinematicChain::getVelocityReference;
    using KinematicChain::getEffortReference;
    using KinematicChain::getStiffness;
    using KinematicChain::getDamping;

    // Setters for TX
    
    using KinematicChain::setPositionReference;
    using KinematicChain::setVelocityReference;
    using KinematicChain::setEffortReference;
    using KinematicChain::setStiffness;
    using KinematicChain::setDamping;
    
    /**
     * @brief Setter for the robot chain references using a model chain
     * 
     * @param model_chain the model chain reference to set on the current robot chain
     * @param flags Flags to specify what part of the state must be synchronized. By default (i.e. if
     * this argument is omitted) the whole state is synchronized. Otherwise, an arbitrary number of flags
     * can be specified in order to select a subset of the state. The flags must be of the enum type
     * XBot::Sync, which can take the following values:
     *  - Sync::Position, 
     *  - Sync::Velocity
     *  - Sync::Acceleration
     *  - Sync::Effort
     *  - Sync::Stiffness 
     *  - Sync::Damping 
     *  - Sync::Impedance
     *  - Sync::All
     * @return true on success, false otherwise
     */
    template <typename... SyncFlags>
    bool setReferenceFrom(const ModelChain& model_chain, SyncFlags... flags);
    
    
protected:
    
private:
    
    using KinematicChain::syncFrom;
    using KinematicChain::pushBackJoint;
    
    // Setters for RX
    
    using KinematicChain::setJointPosition;
    using KinematicChain::setMotorPosition;
    using KinematicChain::setJointVelocity;
    using KinematicChain::setMotorVelocity;
    using KinematicChain::setJointEffort;
    using KinematicChain::setTemperature;
    
    using KinematicChain::setJointAcceleration;
    using KinematicChain::getJointAcceleration;

    
    
};

template <typename... SyncFlags>
bool RobotChain::setReferenceFrom(const ModelChain& other, SyncFlags... flags)
{
    
    int pos = 0;
    for (const XBot::Joint::Ptr & j : other._joint_vector) {
        _joint_vector[pos++]->setReferenceFrom(*j, flags...);
    }
}

} // end namespace XBot
#endif