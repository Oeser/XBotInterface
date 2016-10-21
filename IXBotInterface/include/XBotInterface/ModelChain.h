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
    
    
    // Setters for RX
    
    virtual bool setJointPosition(const Eigen::VectorXd &q);
    virtual bool setMotorPosition(const Eigen::VectorXd &q);
    virtual bool setJointVelocity(const Eigen::VectorXd &qdot);
    virtual bool setMotorVelocity(const Eigen::VectorXd &qdot);
    virtual bool setJointEffort(const Eigen::VectorXd &tau);
    virtual bool setTemperature(const Eigen::VectorXd &temp);

    virtual bool setJointPosition(const std::map<int, double> &q);
    virtual bool setMotorPosition(const std::map<int, double> &q);
    virtual bool setJointVelocity(const std::map<int, double> &qdot);
    virtual bool setMotorVelocity(const std::map<int, double> &qdot);
    virtual bool setJointEffort(const std::map<int, double> &tau);
    virtual bool setTemperature(const std::map<int, double> &temp);

    virtual bool setJointPosition(const std::map<std::string, double> &q);
    virtual bool setMotorPosition(const std::map<std::string, double> &q);
    virtual bool setJointVelocity(const std::map<std::string, double> &qdot);
    virtual bool setMotorVelocity(const std::map<std::string, double> &qdot);
    virtual bool setJointEffort(const std::map<std::string, double> &tau);
    virtual bool setTemperature(const std::map<std::string, double> &temp);

    virtual bool setJointPosition(int i, double q);
    virtual bool setMotorPosition(int i, double q);
    virtual bool setJointVelocity(int i, double qdot);
    virtual bool setMotorVelocity(int i, double qdot);
    virtual bool setJointEffort(int i, double tau);
    virtual bool setTemperature(int i, double temp);
    
    virtual bool syncFrom(const KinematicChain& other);

    
    
    
protected:
    
private:
    
    
};
} // end namespace XBot
#endif