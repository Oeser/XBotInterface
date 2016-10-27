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

#ifndef __KINEMATIC_CHAIN_H__
#define __KINEMATIC_CHAIN_H__

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <iostream>

#include <eigen3/Eigen/Dense>

#include<XBotInterface/Joint.h>
#include<XBotCoreModel.h>
#include<XBotInterface/ForceTorqueSensor.h>


namespace XBot
{
// TBD update liburdf-headers-dev to version >0.4
// typedef boost::shared_ptr<urdf::Joint const> JointConstSharedPtr;
// typedef boost::shared_ptr<urdf::Link const> LinkConstSharedPtr;

// NOTE forward declaration because of friendship
class IXBotInterface;

class KinematicChain
{

public:
    
    friend XBot::IXBotInterface;

    /**
     * @brief Default constructor
     *
     */
    KinematicChain();

    /**
     * @brief Construct a Kinematic Chain using the chain name and the XBotCoreModel
     *
     * @param chain_name the name of the chain
     * @param XBotModel the model built using XBotModel
     */
    KinematicChain(const std::string &chain_name,
                   const XBot::XBotCoreModel &XBotModel);
    
    /**
     * @brief Construct a Kinematic Chain using the chain name
     * 
     * @param chain_name the name of the chain
     */
    explicit KinematicChain(const std::string& chain_name);

    /**
     * @brief Custom copy constructor, which guarantees independence between
     * copies by performing a deep copy
     *
     */
    KinematicChain(const KinematicChain &other);


    /**
     * @brief Custom copy assignment, which guarantees independence between
     * copies by performing a deep copy
     *
     */
    KinematicChain &operator= (const KinematicChain &rhs);

    /**
     * @brief shared pointer to a KinematicChain
     * 
     */
    typedef std::shared_ptr<KinematicChain> Ptr;
    
    /**
     * @brief Getter for the force-torque sensor map 
     * 
     * @return A map whose key is the sensor name (i.e. the name of the sensor link inside the URDF) and
     * whose value is a shared pointer to the force torque sensor.
     */
    std::map<std::string, ForceTorqueSensor::ConstPtr> getForceTorque() const;
    
    /**
     * @brief Returns a force-torque sensor given its parent-link name. 
     * 
     * @param parent_link_name Name of the link to which the sensor is attached
     * @param ft A shared pointer to the force-torque sensor. A dummy FT is returned if either parent_link_name is not defined
     * or it does not contain any FT sensor.
     * @return True if a FT sensor with given parent_link_name was found. False otherwise.
     */
    bool getForceTorque(const std::string& parent_link_name, ForceTorqueSensor::ConstPtr& ft ) const;

    
    void getJointLimits(Eigen::VectorXd& q_min, Eigen::VectorXd& q_max) const;
    void getVelocityLimits(Eigen::VectorXd& qdot_max) const;
    void getEffortLimits(Eigen::VectorXd& tau_max) const;
    void getJointLimits(int i, double& q_min, double& q_max) const;
    void getVelocityLimits(int i, double& qdot_max) const;
    void getEffortLimits(int i, double& tau_max) const;
    bool checkJointLimits(const Eigen::VectorXd& q, 
                          std::vector<std::string>& violating_joints) const;
    bool checkVelocityLimits(const Eigen::VectorXd& qdot, 
                          std::vector<std::string>& violating_joints) const;
    bool checkEffortLimits(const Eigen::VectorXd& tau, 
                          std::vector<std::string>& violating_joints) const;
    bool checkJointLimits(const Eigen::VectorXd& q) const;
    bool checkVelocityLimits(const Eigen::VectorXd& qdot) const;
    bool checkEffortLimits(const Eigen::VectorXd& tau) const;
    
    /**
     * @brief add a joint in the kinematic chain pushing it in the end of the chain
     * 
     * @param joint the joint to add
     * @return void
     */
    void pushBackJoint(Joint::Ptr joint);
    
    /**
     * @brief Method for determining whether a chain is virtual, i.e. contains all virtual joints
     * 
     * @return A boolean set to true if chain is virtual, false otherwise.
     */
    bool isVirtual() const;

    /**
     * @brief Method returning the name of the chain
     *
     * @return Chain name as const std::string&
     */
    const std::string &chainName() const;

    /**
     * @brief Method returning the name of the chain base link
     *
     * @return Base link name as const std::string&
     */
    const std::string &baseLinkName() const;

    /**
     * @brief Method returning the name of the chain tip link
     *
     * @return Tip link name as const std::string&
     */
    const std::string &tipLinkName() const;

    /**
     * @brief Method returning the name of the child link corresponding
     * to the i-th joint of the chain
     *
     * @param i The position of the joint along the chain (i = 0 refers to the joint attached to the base link)
     *
     * @return The name of the child link of joint n° i
     */
    const std::string &childLinkName(int i) const;

    /**
     * @brief Method returning the name of the parent link corresponding
     * to the i-th joint of the chain
     *
     * @param i The position of the joint along the chain (i = 0 refers to the joint attached to the base link)
     *
     * @return The name of the parent link of joint n° i
     */
    const std::string &parentLinkName(int i) const;

    /**
     * @brief Method returning the name of the i-th joint of the chain
     *
     * @param i The position of the joint whose name is queried (i = 0 refers to the joint attached to the base link)
     *
     * @return The name of the joint n° i
     */
    const std::string &jointName(int i) const;
    
    
    /**
     * @brief Returns a vector containing the namess of all joints in the chain, from
     * base link to tip link
     * 
     * @return a const reference to the vector of joint names
     */
    const std::vector<std::string>& jointNames() const;

    /**
     * @brief Method returning the ID of the i-th joint of the chain
     *
     * @param i The position of the joint whose ID is queried (i = 0 refers to the joint attached to the base link)
     *
     * @return The ID of the joint n° i
     */
    int jointId(int i) const;
    
    bool hasJoint(int id) const;
    
    bool hasJoint(const std::string& joint_name) const;
    
    Joint::ConstPtr joint(int idx) const;
    
    /**
     * @brief Returns a vector containing the IDs of all joints in the chain, from
     * base link to tip link
     * 
     * @return a const reference to the vector of joint IDs
     */
    const std::vector<int>& jointIds() const;

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
     * @return const std::vector< urdf::JointConstSharedPtr>&
     */
    const std::vector< urdf::JointConstSharedPtr > &getUrdfJoints() const;

    /**
     * @brief Method returning the vector of urdf::Links corresponding to the chain.
     *
     * @return const std::vector< urdf::LinkConstSharedPtr>&
     */
    const std::vector< urdf::LinkConstSharedPtr > &getUrdfLinks() const;
    
    void shallowCopy(const KinematicChain& chain);

    // Getters for RX
    
    bool getJointPosition(Eigen::VectorXd &q) const;
    bool getMotorPosition(Eigen::VectorXd &q) const;
    bool getJointVelocity(Eigen::VectorXd &qdot) const;
    bool getMotorVelocity(Eigen::VectorXd &qdot) const;
    bool getJointEffort(Eigen::VectorXd &tau) const;
    bool getTemperature(Eigen::VectorXd &temp) const;

    bool getJointPosition(std::map<int, double> &q) const;
    bool getMotorPosition(std::map<int, double> &q) const;
    bool getJointVelocity(std::map<int, double> &qdot) const;
    bool getMotorVelocity(std::map<int, double> &qdot) const;
    bool getJointEffort(std::map<int, double> &tau) const;
    bool getTemperature(std::map<int, double> &temp) const;

    bool getJointPosition(std::map<std::string, double> &q) const;
    bool getMotorPosition(std::map<std::string, double> &q) const;
    bool getJointVelocity(std::map<std::string, double> &qdot) const;
    bool getMotorVelocity(std::map<std::string, double> &qdot) const;
    bool getJointEffort(std::map<std::string, double> &tau) const;
    bool getTemperature(std::map<std::string, double> &temp) const;

    double getJointPosition(int index) const;
    double getMotorPosition(int index) const;
    double getJointVelocity(int index) const;
    double getMotorVelocity(int index) const;
    double getJointEffort(int index) const;
    double getTemperature(int index) const;
    
    // Setters for RX
    
    bool setJointPosition(const Eigen::VectorXd &q);
    bool setMotorPosition(const Eigen::VectorXd &q);
    bool setJointVelocity(const Eigen::VectorXd &qdot);
    bool setMotorVelocity(const Eigen::VectorXd &qdot);
    bool setJointEffort(const Eigen::VectorXd &tau);
    bool setTemperature(const Eigen::VectorXd &temp);

    bool setJointPosition(const std::map<int, double> &q);
    bool setMotorPosition(const std::map<int, double> &q);
    bool setJointVelocity(const std::map<int, double> &qdot);
    bool setMotorVelocity(const std::map<int, double> &qdot);
    bool setJointEffort(const std::map<int, double> &tau);
    bool setTemperature(const std::map<int, double> &temp);

    bool setJointPosition(const std::map<std::string, double> &q);
    bool setMotorPosition(const std::map<std::string, double> &q);
    bool setJointVelocity(const std::map<std::string, double> &qdot);
    bool setMotorVelocity(const std::map<std::string, double> &qdot);
    bool setJointEffort(const std::map<std::string, double> &tau);
    bool setTemperature(const std::map<std::string, double> &temp);

    bool setJointPosition(int i, double q);
    bool setMotorPosition(int i, double q);
    bool setJointVelocity(int i, double qdot);
    bool setMotorVelocity(int i, double qdot);
    bool setJointEffort(int i, double tau);
    bool setTemperature(int i, double temp);

    // Getters for TX

    bool getPositionReference(Eigen::VectorXd &q) const;
    bool getVelocityReference(Eigen::VectorXd &qdot) const;
    bool getEffortReference(Eigen::VectorXd &tau) const;
    bool getStiffness(Eigen::VectorXd &K) const;
    bool getDamping(Eigen::VectorXd &D) const;

    bool getPositionReference(std::map<int, double> &q) const;
    bool getVelocityReference(std::map<int, double> &qdot) const;
    bool getEffortReference(std::map<int, double> &tau) const;
    bool getStiffness(std::map<int, double> &K) const;
    bool getDamping(std::map<int, double> &D) const;

    bool getPositionReference(std::map<std::string, double> &q) const;
    bool getVelocityReference(std::map<std::string, double> &qdot) const;
    bool getEffortReference(std::map<std::string, double> &tau) const;
    bool getStiffness(std::map<std::string, double> &K) const;
    bool getDamping(std::map<std::string, double> &D) const;

    double getPositionReference(int index) const;
    double getVelocityReference(int index) const;
    double getEffortReference(int index) const;
    double getStiffness(int index) const;
    double getDamping(int index) const;

    // Setters for TX

    bool setPositionReference(const Eigen::VectorXd &q);
    bool setVelocityReference(const Eigen::VectorXd &qdot);
    bool setEffortReference(const Eigen::VectorXd &tau);
    bool setStiffness(const Eigen::VectorXd &K);
    bool setDamping(const Eigen::VectorXd &D);

    bool setPositionReference(const std::map<int, double> &q);
    bool setVelocityReference(const std::map<int, double> &qdot);
    bool setEffortReference(const std::map<int, double> &tau);
    bool setStiffness(const std::map<int, double> &K);
    bool setDamping(const std::map<int, double> &D);

    bool setPositionReference(const std::map<std::string, double> &q);
    bool setVelocityReference(const std::map<std::string, double> &qdot);
    bool setEffortReference(const std::map<std::string, double> &tau);
    bool setStiffness(const std::map<std::string, double> &K);
    bool setDamping(const std::map<std::string, double> &D);

    bool setPositionReference(int i, double q);
    bool setVelocityReference(int i, double qdot);
    bool setEffortReference(int i, double tau);
    bool setStiffness(int i, double K);
    bool setDamping(int i, double D);

    bool syncFrom(const KinematicChain &other);

    friend std::ostream& operator<<(std::ostream& os, const XBot::KinematicChain& c);

protected:
    
    std::map<std::string, XBot::Joint::Ptr> _joint_name_map;
    std::map<int, XBot::Joint::Ptr> _joint_id_map;
    std::vector<XBot::Joint::Ptr> _joint_vector;
    
    std::vector<ForceTorqueSensor::Ptr> _ft_vector;
    std::map<std::string, ForceTorqueSensor::Ptr> _ft_map;
    
    
    
    
    /**
     * @brief Getter for the i-th Joint Ptr 
     * 
     * @param i index inside the chain (i=0 is the child joint of base link)
     * @return A shared pointer to the requested joint
     */
    Joint::Ptr getJoint(int i) const;
    
    std::map< std::string, ForceTorqueSensor::Ptr > getForceTorqueInternal() const;
    
    
private:


    std::vector<urdf::JointConstSharedPtr> _urdf_joints;
    std::vector<urdf::LinkConstSharedPtr> _urdf_links;

    std::vector<std::string> _ordered_joint_name;
    std::vector<int> _ordered_joint_id;

    XBot::XBotCoreModel _XBotModel;

    std::string _chain_name;
    int _joint_num;
    
    bool _is_virtual;
    
    XBot::Joint::ConstPtr getJointByName(const std::string& joint_name) const;
    XBot::Joint::ConstPtr getJointById(int joint_id) const;


};

    std::ostream& operator<<(std::ostream& os, const XBot::KinematicChain& c);
}

#endif // __KINEMATIC_CHAIN_H__
