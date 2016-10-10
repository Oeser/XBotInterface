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
#include <XBotInterface/KinematicChain.h>

namespace XBot {
    // TBD XBotState
    class IXBotInterface {
    
    public:
        
        typedef std::shared_ptr<IXBotInterface> Ptr;
        
        explicit IXBotInterface(const XBotCoreModel& XBotModel);
        
        IXBotInterface(const IXBotInterface& other);
        
        IXBotInterface& operator= (const IXBotInterface& rhs);
        
        virtual ~IXBotInterface();
        
            // TODO: bool hasVelocity(), hasImpedance(), ....
        
        const urdf::ModelInterface& getUrdf() const;
        const srdf::Model& getSrdf() const;
        const std::string& getUrdfString() const;
        const std::string& getSrdfString() const;
        std::vector<std::string> getChainNames() const;
		const std::vector<std::string>& getEnabledJointNames() const;
		bool hasJoint(const std::string& joint_name) const;
        
        KinematicChain& operator()(const std::string& chain_name);
        KinematicChain& leg(int id);
        KinematicChain& arm(int id);
        
        int legs() const;
        int arms() const;
        
        // TBD sync all the joints and sensors information
        bool sync(const IXBotInterface& other);
        
        
        bool getLinkPos(Eigen::VectorXd& q) const;
        bool getMotorPos(Eigen::VectorXd& q) const;
        bool getLinkVel(Eigen::VectorXd& qdot) const;
        bool getMotorVel(Eigen::VectorXd& qdot) const;
        bool getEffort(Eigen::VectorXd& tau) const;
        bool getTemperature(Eigen::VectorXd& temp) const;
        
        bool getLinkPos(std::map<int, double>& q) const;
        bool getMotorPos(std::map<int, double>& q) const;
        bool getLinkVel(std::map<int, double>& qdot) const;
        bool getMotorVel(std::map<int, double>& qdot) const;
        bool getEffort(std::map<int, double>& tau) const;
        bool getTemperature(std::map<int, double>& temp) const;
        
        bool getLinkPos(std::map<std::string, double>& q) const;
        bool getMotorPos(std::map<std::string, double>& q) const;
        bool getLinkVel(std::map<std::string, double>& qdot) const;
        bool getMotorVel(std::map<std::string, double>& qdot) const;
        bool getEffort(std::map<std::string, double>& tau) const;
        bool getTemperature(std::map<std::string, double>& temp) const;
        

        bool getPosRef(Eigen::VectorXd& q) const;
        bool getVelRef(Eigen::VectorXd& qdot) const;
        bool getEffortRef(Eigen::VectorXd& tau) const;
        bool getStiffness(Eigen::VectorXd& K) const;
        bool getDamping(Eigen::VectorXd& D) const;
        
        bool getPosRef(std::map<int, double>& q) const;
        bool getVelRef(std::map<int, double>& qdot) const;
        bool getEffortRef(std::map<int, double>& tau) const;
        bool getStiffness(std::map<int, double>& K) const;
        bool getDamping(std::map<int, double>& D) const;
        
        bool getPosRef(std::map<std::string, double>& q) const;
        bool getVelRef(std::map<std::string, double>& qdot) const;
        bool getEffortRef(std::map<std::string, double>& tau) const;
        bool getStiffness(std::map<std::string, double>& K) const;
        bool getDamping(std::map<std::string, double>& D) const;
        
        
        
        bool setLinkPos(const Eigen::VectorXd& q);
        bool setMotorPos(const Eigen::VectorXd& q);
        bool setLinkVel(const Eigen::VectorXd& qdot);
        bool setMotorVel(const Eigen::VectorXd& qdot);
        bool setEffort(const Eigen::VectorXd& tau);
        bool setTemperature(const Eigen::VectorXd& temp);
        
        bool setLinkPos(const std::map<int, double>& q);
        bool setMotorPos(const std::map<int, double>& q);
        bool setLinkVel(const std::map<int, double>& qdot);
        bool setMotorVel(const std::map<int, double>& qdot);
        bool setEffort(const std::map<int, double>& tau);
        bool setTemperature(const std::map<int, double>& temp);
        
        bool setLinkPos(const std::map<std::string, double>& q);
        bool setMotorPos(const std::map<std::string, double>& q);
        bool setLinkVel(const std::map<std::string, double>& qdot);
        bool setMotorVel(const std::map<std::string, double>& qdot);
        bool setEffort(const std::map<std::string, double>& tau);
        bool setTemperature(const std::map<std::string, double>& temp);
        
        
        
        bool setPosRef(const Eigen::VectorXd& q);
        bool setVelRef(const Eigen::VectorXd& qdot);
        bool setEffortRef(const Eigen::VectorXd& tau);
        bool setStiffness(const Eigen::VectorXd& K);
        bool setDamping(const Eigen::VectorXd& D);
        
        bool setPosRef(const std::map<int, double>& q);
        bool setVelRef(const std::map<int, double>& qdot);
        bool setEffortRef(const std::map<int, double>& tau);
        bool setStiffness(const std::map<int, double>& K);
        bool setDamping(const std::map<int, double>& D);
        
        bool setPosRef(const std::map<std::string, double>& q);
        bool setVelRef(const std::map<std::string, double>& qdot);
        bool setEffortRef(const std::map<std::string, double>& tau);
        bool setStiffness(const std::map<std::string, double>& K);
        bool setDamping(const std::map<std::string, double>& D);
        
        

    protected:
        
        
        

    private:

        int _joint_num;
        XBotCoreModel _XBotModel;
        std::string _urdf_string, _srdf_string;
        
        std::vector<std::string> _ordered_joint_name;
        std::vector<int> _ordered_joint_id;
        
        std::map<std::string, XBot::KinematicChain::Ptr> _chain_map;
        XBot::KinematicChain _dummy_chain;

    };

}

#endif // __I_XBOT_INTERFACE_H__