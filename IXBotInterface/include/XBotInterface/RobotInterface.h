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

#ifndef __ROBOT_INTERFACE_H__
#define __ROBOT_INTERFACE_H__

#include <vector>
#include <map>
#include <memory>
#include <SharedLibraryClass.h>

#include <cstdlib>

#include <XBotInterface/IXBotInterface.h>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotChain.h>

namespace XBot
{

class RobotInterface : public IXBotInterface
{

public:

    RobotInterface();

    typedef std::shared_ptr<RobotInterface> Ptr;

    static RobotInterface::Ptr getRobot(const std::string &path_to_cfg, int argc, char **argv);

    bool sense(bool sync_model = true);
    bool move(bool sync_model = true);
    
    RobotChain& operator()(const std::string& chain_name);
    RobotChain& arm(int arm_id);
    RobotChain& leg(int leg_id);
    
    bool setReferenceFrom(const ModelInterface& model);

    virtual bool setControlMode(const std::map<std::string, std::string> &joint_control_mode_map) = 0;
    virtual bool setControlMode(const std::string &robot_control_mode) = 0;
    virtual bool getControlMode(std::map<std::string, std::string> &joint_control_mode_map) = 0;
    
    // Getters for TX

    virtual bool getPositionReference(Eigen::VectorXd& q) const;
    virtual bool getVelocityReference(Eigen::VectorXd& qdot) const;
    virtual bool getEffortReference(Eigen::VectorXd& tau) const;
    virtual bool getStiffness(Eigen::VectorXd& K) const;
    virtual bool getDamping(Eigen::VectorXd& D) const;
    
    virtual bool getPositionReference(std::map<int, double>& q) const;
    virtual bool getVelocityReference(std::map<int, double>& qdot) const;
    virtual bool getEffortReference(std::map<int, double>& tau) const;
    virtual bool getStiffness(std::map<int, double>& K) const;
    virtual bool getDamping(std::map<int, double>& D) const;
    
    virtual bool getPositionReference(std::map<std::string, double>& q) const;
    virtual bool getVelocityReference(std::map<std::string, double>& qdot) const;
    virtual bool getEffortReference(std::map<std::string, double>& tau) const;
    virtual bool getStiffness(std::map<std::string, double>& K) const;
    virtual bool getDamping(std::map<std::string, double>& D) const;
    
    

    // Setters for TX
    
    virtual bool setPositionReference(const Eigen::VectorXd& q);
    virtual bool setVelocityReference(const Eigen::VectorXd& qdot);
    virtual bool setEffortReference(const Eigen::VectorXd& tau);
    virtual bool setStiffness(const Eigen::VectorXd& K);
    virtual bool setDamping(const Eigen::VectorXd& D);
    
    virtual bool setPositionReference(const std::map<int, double>& q);
    virtual bool setVelocityReference(const std::map<int, double>& qdot);
    virtual bool setEffortReference(const std::map<int, double>& tau);
    virtual bool setStiffness(const std::map<int, double>& K);
    virtual bool setDamping(const std::map<int, double>& D);
    
    virtual bool setPositionReference(const std::map<std::string, double>& q);
    virtual bool setVelocityReference(const std::map<std::string, double>& qdot);
    virtual bool setEffortReference(const std::map<std::string, double>& tau);
    virtual bool setStiffness(const std::map<std::string, double>& K);
    virtual bool setDamping(const std::map<std::string, double>& D);        

protected:

    virtual bool sense_internal() = 0;
    virtual bool move_internal() = 0;
    virtual bool init_robot(const std::string& path_to_cfg) = 0;
    
    virtual const std::vector<std::string>& getModelOrderedChainName() final;

private:
    
    virtual bool init_internal(const std::string &path_to_cfg);
    
    std::map<std::string, XBot::KinematicChain::Ptr> _chain_map;
    std::map<std::string, XBot::RobotChain::Ptr> _robot_chain_map;
    XBot::RobotChain _dummy_chain;
    
    static RobotInterface::Ptr _instance_ptr;
    static shlibpp::SharedLibraryClass<RobotInterface> _robot_interface_instance;
    static ModelInterface::Ptr _model;
    
    static std::string _framework;
    static std::string _subclass_name;
    static std::string _path_to_shared_lib;
    static std::string _subclass_factory_name;
            
    std::vector<std::string> _model_ordered_chain_name;
    
    static bool parseYAML(const std::string &path_to_cfg);
    


};
}

#endif // __ROBOT_INTERFACE_H__