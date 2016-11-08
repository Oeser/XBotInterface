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


#ifndef _XBOT_MODELCHAIN_H_
#define _XBOT_MODELCHAIN_H_

#include "XBotInterface/KinematicChain.h"

namespace XBot {

class RobotChain;
    
class ModelChain : public KinematicChain {
    
public:
    
    friend XBot::RobotChain;
    
    typedef std::shared_ptr<ModelChain> Ptr;

    // Getters for RX

    using KinematicChain::getJointPosition;
    using KinematicChain::getMotorPosition;
    using KinematicChain::getJointVelocity;
    using KinematicChain::getMotorVelocity;
    using KinematicChain::getJointAcceleration;
    using KinematicChain::getJointEffort;

    // Setters for RX
    
    using KinematicChain::setJointPosition;
    using KinematicChain::setMotorPosition;
    using KinematicChain::setJointVelocity;
    using KinematicChain::setMotorVelocity;
    using KinematicChain::setJointAcceleration;
    using KinematicChain::setJointEffort;

protected:
    
private:
    
    using KinematicChain::getTemperature;
    using KinematicChain::setTemperature;
    
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
    
    using KinematicChain::pushBackJoint;
    
    
};
} // end namespace XBot
#endif