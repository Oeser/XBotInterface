/*
 * Copyright (C) 2016 Walkman
 * Author: Luca Muratore
 * email:  luca.muratore@iit.it
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

#ifndef __I_XBOT_INTERFACE_H__
#define __I_XBOT_INTERFACE_H__

#include <vector>
#include <map>
#include <memory>
#include <algorithm>

#include <yaml-cpp/yaml.h>
#include <XBotCoreModel.h>
#include <XBotInterface/KinematicChain.h>

namespace XBot {
    // TBD XBotState
    class IXBotInterface {
    
    public:
        
        typedef std::shared_ptr<IXBotInterface> Ptr;
        
        IXBotInterface();
        
        IXBotInterface(const IXBotInterface& other);
        
        IXBotInterface& operator= (const IXBotInterface& rhs);
        
        virtual ~IXBotInterface();
        
        bool init(const std::string& path_to_cfg);
        
        // TODO: bool hasVelocity(), hasImpedance(), ....
        
        const urdf::ModelInterface& getUrdf() const;
        const srdf::Model& getSrdf() const;
        const std::string& getUrdfString() const;
        const std::string& getSrdfString() const;
        std::vector<std::string> getChainNames() const;
        std::map<std::string, XBot::KinematicChain::Ptr>  getChainMap() const;
        const std::vector<std::string>& getEnabledJointNames() const;
        bool hasJoint(const std::string& joint_name) const;
        
        XBot::Joint::ConstPtr getJointByName(const std::string& joint_name) const;
        
        KinematicChain& operator()(const std::string& chain_name);
        KinematicChain& leg(int id);
        KinematicChain& arm(int id);
        
        int legs() const;
        int arms() const;
        
        bool syncFrom(const IXBotInterface& other);
        
        
        bool getJointPosition(Eigen::VectorXd& q) const;
        bool getMotorPosition(Eigen::VectorXd& q) const;
        bool getJointVelocity(Eigen::VectorXd& qdot) const;
        bool getMotorVelocity(Eigen::VectorXd& qdot) const;
        bool getJointEffort(Eigen::VectorXd& tau) const;
        bool getTemperature(Eigen::VectorXd& temp) const;
        
        bool getJointPosition(std::map<int, double>& q) const;
        bool getMotorPosition(std::map<int, double>& q) const;
        bool getJointVelocity(std::map<int, double>& qdot) const;
        bool getMotorVelocity(std::map<int, double>& qdot) const;
        bool getJointEffort(std::map<int, double>& tau) const;
        bool getTemperature(std::map<int, double>& temp) const;
        
        bool getJointPosition(std::map<std::string, double>& q) const;
        bool getMotorPosition(std::map<std::string, double>& q) const;
        bool getJointVelocity(std::map<std::string, double>& qdot) const;
        bool getMotorVelocity(std::map<std::string, double>& qdot) const;
        bool getJointEffort(std::map<std::string, double>& tau) const;
        bool getTemperature(std::map<std::string, double>& temp) const;
        

        bool getPositionReference(Eigen::VectorXd& q) const;
        bool getVelocityReference(Eigen::VectorXd& qdot) const;
        bool getEffortReference(Eigen::VectorXd& tau) const;
        bool getStiffness(Eigen::VectorXd& K) const;
        bool getDamping(Eigen::VectorXd& D) const;
        
        bool getPositionReference(std::map<int, double>& q) const;
        bool getVelocityReference(std::map<int, double>& qdot) const;
        bool getEffortReference(std::map<int, double>& tau) const;
        bool getStiffness(std::map<int, double>& K) const;
        bool getDamping(std::map<int, double>& D) const;
        
        bool getPositionReference(std::map<std::string, double>& q) const;
        bool getVelocityReference(std::map<std::string, double>& qdot) const;
        bool getEffortReference(std::map<std::string, double>& tau) const;
        bool getStiffness(std::map<std::string, double>& K) const;
        bool getDamping(std::map<std::string, double>& D) const;
        
        
        
        
        
        
        
        bool setPositionReference(const Eigen::VectorXd& q);
        bool setVelocityReference(const Eigen::VectorXd& qdot);
        bool setEffortReference(const Eigen::VectorXd& tau);
        bool setStiffness(const Eigen::VectorXd& K);
        bool setDamping(const Eigen::VectorXd& D);
        
        bool setPositionReference(const std::map<int, double>& q);
        bool setVelocityReference(const std::map<int, double>& qdot);
        bool setEffortReference(const std::map<int, double>& tau);
        bool setStiffness(const std::map<int, double>& K);
        bool setDamping(const std::map<int, double>& D);
        
        bool setPositionReference(const std::map<std::string, double>& q);
        bool setVelocityReference(const std::map<std::string, double>& qdot);
        bool setEffortReference(const std::map<std::string, double>& tau);
        bool setStiffness(const std::map<std::string, double>& K);
        bool setDamping(const std::map<std::string, double>& D);
        
        friend std::ostream& operator<<(std::ostream& os, const XBot::IXBotInterface& robot);

    protected:
        
        virtual bool init_internal(const std::string& path_to_cfg){ return true; }
        
        bool setJointPosition(const Eigen::VectorXd& q);
        bool setMotorPosition(const Eigen::VectorXd& q);
        bool setJointVelocity(const Eigen::VectorXd& qdot);
        bool setMotorVelocity(const Eigen::VectorXd& qdot);
        bool setJointEffort(const Eigen::VectorXd& tau);
        bool setTemperature(const Eigen::VectorXd& temp);
        
        bool setJointPosition(const std::map<int, double>& q);
        bool setMotorPosition(const std::map<int, double>& q);
        bool setJointVelocity(const std::map<int, double>& qdot);
        bool setMotorVelocity(const std::map<int, double>& qdot);
        bool setJointEffort(const std::map<int, double>& tau);
        bool setTemperature(const std::map<int, double>& temp);
        
        bool setJointPosition(const std::map<std::string, double>& q);
        bool setMotorPosition(const std::map<std::string, double>& q);
        bool setJointVelocity(const std::map<std::string, double>& qdot);
        bool setMotorVelocity(const std::map<std::string, double>& qdot);
        bool setJointEffort(const std::map<std::string, double>& tau);
        bool setTemperature(const std::map<std::string, double>& temp);
        

    private:

        int _joint_num;
        XBotCoreModel _XBotModel;
        std::string _urdf_string, _srdf_string;
        
        std::vector<std::string> _ordered_joint_name;
        std::vector<int> _ordered_joint_id;
        
        std::map<std::string, XBot::KinematicChain::Ptr> _chain_map;
        XBot::KinematicChain _dummy_chain;

    };
    
    std::ostream& operator<<(std::ostream& os, const XBot::IXBotInterface& robot);

}

#endif // __I_XBOT_INTERFACE_H__