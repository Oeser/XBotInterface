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

class RobotChain : public KinematicChain {
    
public:
    
    typedef std::shared_ptr<RobotChain> Ptr;
    
    RobotChain();
    
    // Getters for TX

    virtual bool getPositionReference(Eigen::VectorXd &q) const;
    virtual bool getVelocityReference(Eigen::VectorXd &qdot) const;
    virtual bool getEffortReference(Eigen::VectorXd &tau) const;
    virtual bool getStiffness(Eigen::VectorXd &K) const;
    virtual bool getDamping(Eigen::VectorXd &D) const;

    virtual bool getPositionReference(std::map<int, double> &q) const;
    virtual bool getVelocityReference(std::map<int, double> &qdot) const;
    virtual bool getEffortReference(std::map<int, double> &tau) const;
    virtual bool getStiffness(std::map<int, double> &K) const;
    virtual bool getDamping(std::map<int, double> &D) const;

    virtual bool getPositionReference(std::map<std::string, double> &q) const;
    virtual bool getVelocityReference(std::map<std::string, double> &qdot) const;
    virtual bool getEffortReference(std::map<std::string, double> &tau) const;
    virtual bool getStiffness(std::map<std::string, double> &K) const;
    virtual bool getDamping(std::map<std::string, double> &D) const;

    virtual double getPositionReference(int index) const;
    virtual double getVelocityReference(int index) const;
    virtual double getEffortReference(int index) const;
    virtual double getStiffness(int index) const;
    virtual double getDamping(int index) const;

    // Setters for TX

    virtual bool setPositionReference(const Eigen::VectorXd &q);
    virtual bool setVelocityReference(const Eigen::VectorXd &qdot);
    virtual bool setEffortReference(const Eigen::VectorXd &tau);
    virtual bool setStiffness(const Eigen::VectorXd &K);
    virtual bool setDamping(const Eigen::VectorXd &D);

    virtual bool setPositionReference(const std::map<int, double> &q);
    virtual bool setVelocityReference(const std::map<int, double> &qdot);
    virtual bool setEffortReference(const std::map<int, double> &tau);
    virtual bool setStiffness(const std::map<int, double> &K);
    virtual bool setDamping(const std::map<int, double> &D);

    virtual bool setPositionReference(const std::map<std::string, double> &q);
    virtual bool setVelocityReference(const std::map<std::string, double> &qdot);
    virtual bool setEffortReference(const std::map<std::string, double> &tau);
    virtual bool setStiffness(const std::map<std::string, double> &K);
    virtual bool setDamping(const std::map<std::string, double> &D);

    virtual bool setPositionReference(int i, double q);
    virtual bool setVelocityReference(int i, double qdot);
    virtual bool setEffortReference(int i, double tau);
    virtual bool setStiffness(int i, double K);
    virtual bool setDamping(int i, double D);

    bool setReferenceFrom(const ModelChain& model_chain);
    
    
protected:
    
private:
    
    
};
} // end namespace XBot
#endif