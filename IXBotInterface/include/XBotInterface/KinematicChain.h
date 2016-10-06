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

#ifndef __KINEMATIC_CHAIN_H__
#define __KINEMATIC_CHAIN_H__

#include <string>
#include <vector>
#include <map>
#include <memory>

#include<XBotInterface/Joint.h>
#include<XBotCoreModel.h>

namespace XBot {
    // TBD update liburdf-headers-dev to version >0.4
    typedef boost::shared_ptr<urdf::Joint const> JointConstSharedPtr;
    typedef boost::shared_ptr<urdf::Link const> LinkConstSharedPtr;

class KinematicChain {
  
  public:

    /**
     * @brief Default constructor
     * 
     */
    KinematicChain();
    
    KinematicChain(const std::string& chain_name, 
		   const XBot::XBotCoreModel& XBotModel);
    
    /**
     * @brief Custom copy constructor, which guarantees independence between
     * copies by performing a deep copy
     * 
     */
    KinematicChain(const KinematicChain& other);
    
    
    /**
     * @brief Custom copy assignment, which guarantees independence between
     * copies by performing a deep copy
     * 
     */
    KinematicChain& operator= (const KinematicChain& rhs);
    
    /**
     * @brief Method returning the name of the chain
     * 
     * @return Chain name as const std::string&
     */
    const std::string& chainName() const;
    
    
    /**
     * @brief Method returning the name of the chain base link
     * 
     * @return Base link name as const std::string&
     */
    const std::string& baseLinkName() const;
    
    
    /**
     * @brief Method returning the name of the chain tip link
     * 
     * @return Tip link name as const std::string&
     */
    const std::string& tipLinkName() const;
    
    
    /**
     * @brief Method returning the name of the child link corresponding
     * to the id-th joint of the chain
     * 
     * @param id The id of the joint along the chain (id = 0 refers to the joint attached to the base link)
     * 
     * @return The name of the child link of joint n° id
     */
    const std::string& childLinkName(int id) const;
    
    
    /**
     * @brief Method returning the name of the parent link corresponding
     * to the id-th joint of the chain
     * 
     * @param id The id of the joint along the chain (id = 0 refers to the joint attached to the base link)
     * 
     * @return The name of the parent link of joint n° id
     */
    const std::string& parentLinkName(int id) const;
    
    
    /**
     * @brief Method returning the name of the id-th joint of the chain
     * 
     * @param id The id of the joint whose name is queried (id = 0 refers to the joint attached to the base link)
     * 
     * @return The name of the joint n° id
     */
    const std::string& jointName(int id) const;
    
    
    /**
     * @brief Method returning the number of enabled joints
     * belonging to the chain
     * 
     * @return The number of enabled joints
     * belonging to the chain
     */
    int getJointNum() const;
    
    
    /**
     * @brief Method returning the vector of urdf::Joints corresponding to the chain.
     * 
     * @return const std::vector< XBot::JointConstSharedPtr>&
     */
    const std::vector< XBot::JointConstSharedPtr >& getJoints() const;
    
    /**
     * @brief Method returning the vector of XBot::Links corresponding to the chain.
     * 
     * @return const std::vector< XBot::LinkConstSharedPtr>&
     */
    const std::vector< XBot::LinkConstSharedPtr >& getLinks() const;
    
    
    // TBD  implement it
//     bool setJointPosition(const Eigen::VectorXd& q);
//     bool setJointVelocity(const Eigen::VectorXd& qdot);
//     bool setJointAcceleration(const Eigen::VectorXd& qddot);
//     bool setJointEffort(const Eigen::VectorXd& tau);
//     bool setJointImpedance(const Eigen::VectorXd& K, const Eigen::VectorXd& D);
//     
//     bool getJointPosition(Eigen::VectorXd& q) const;
//     bool getJointVelocity(Eigen::VectorXd& qdot) const;
//     bool getJointAcceleration(Eigen::VectorXd& qddot) const;
//     bool getJointEffort(Eigen::VectorXd& tau) const;
//     bool getJointImpedance(Eigen::VectorXd& K, Eigen::VectorXd& D) const;
//     
//     const Eigen::VectorXd& getJointPosition() const;
//     const Eigen::VectorXd& getJointVelocity() const;
//     const Eigen::VectorXd& getJointAcceleration() const;
//     const Eigen::VectorXd& getJointEffort() const;
    
    
    typedef std::shared_ptr<KinematicChain> Ptr;

  protected:
  
  private:
    
    std::map<std::string, XBot::Joint::Ptr> _joint_name_map;
    std::map<int, XBot::Joint::Ptr> _joint_id_map;
    std::vector<XBot::Joint::Ptr> _joint_vector;
    
    std::vector<XBot::JointConstSharedPtr> _urdf_joints;
    std::vector<XBot::LinkConstSharedPtr> _urdf_links;
    
    std::vector<std::string> _ordered_joint_name;
    std::vector<int> _ordered_joint_id;
    
    XBot::XBotCoreModel _XBotModel;
    
    std::string _chain_name;
    int _joint_num;

};

}

#endif // __KIN_CHAIN_H__