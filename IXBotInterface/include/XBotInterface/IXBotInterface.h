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

#ifndef __I_XBOT_INTERFACE_H__
#define __I_XBOT_INTERFACE_H__

#include <vector>
#include <map>
#include <memory>
#include <algorithm>

#include <yaml-cpp/yaml.h>
#include <XBotCoreModel.h>
#include <XBotInterface/KinematicChain.h>

#define LIB_MIDDLE_PATH "/build/install/lib/"

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
        
        int legs() const;
        int arms() const;
        

        // Getters for RX
        
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
        
        
        friend std::ostream& operator<<(std::ostream& os, const XBot::IXBotInterface& robot);

    protected:
        
        virtual bool syncFrom(const IXBotInterface& other);
        
        virtual bool init_internal(const std::string& path_to_cfg){ return true; }
        
        const std::string& getPathToConfig() const;
        
        static bool computeAbsolutePath( const std::string& input_path,
                                         const std::string& midlle_path,
                                         std::string& absolute_path,
                                         std::string extension = "");
        
        virtual const std::vector<std::string>& getModelOrderedChainName();

        // Setters for RX
        
        virtual bool setJointPosition(const Eigen::VectorXd& q);
        virtual bool setMotorPosition(const Eigen::VectorXd& q);
        virtual bool setJointVelocity(const Eigen::VectorXd& qdot);
        virtual bool setMotorVelocity(const Eigen::VectorXd& qdot);
        virtual bool setJointEffort(const Eigen::VectorXd& tau);
        virtual bool setTemperature(const Eigen::VectorXd& temp);
        
        virtual bool setJointPosition(const std::map<int, double>& q);
        virtual bool setMotorPosition(const std::map<int, double>& q);
        virtual bool setJointVelocity(const std::map<int, double>& qdot);
        virtual bool setMotorVelocity(const std::map<int, double>& qdot);
        virtual bool setJointEffort(const std::map<int, double>& tau);
        virtual bool setTemperature(const std::map<int, double>& temp);
        
        virtual bool setJointPosition(const std::map<std::string, double>& q);
        virtual bool setMotorPosition(const std::map<std::string, double>& q);
        virtual bool setJointVelocity(const std::map<std::string, double>& qdot);
        virtual bool setMotorVelocity(const std::map<std::string, double>& qdot);
        virtual bool setJointEffort(const std::map<std::string, double>& tau);
        virtual bool setTemperature(const std::map<std::string, double>& temp);
        
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
        
        XBotCoreModel _XBotModel;

    private:

        int _joint_num;
        
        std::string _urdf_string, _srdf_string;
        std::string _urdf_path;
        std::string _srdf_path;
        std::string _joint_map_config;
        
        std::vector<std::string> _ordered_joint_name;
        std::vector<int> _ordered_joint_id;
        
        std::map<std::string, XBot::KinematicChain::Ptr> _chain_map;
        
        
        std::string _path_to_cfg;
        
        bool parseYAML(const std::string &path_to_cfg);


    };
    
    std::ostream& operator<<(std::ostream& os, const XBot::IXBotInterface& robot);

}

#endif // __I_XBOT_INTERFACE_H__