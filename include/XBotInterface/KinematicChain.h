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

#include <XBotInterface/Joint.h>
#include <XBotCoreModel.h>
#include <XBotInterface/ForceTorqueSensor.h>
#include <XBotInterface/ImuSensor.h>


namespace XBot
{
    
// NOTE forward declaration because of friendship
class XBotInterface;

/**
 * @brief Kinematic chain abstraction as a set of joints and sensors.
 * 
 */
class KinematicChain
{

public:

    typedef std::shared_ptr<KinematicChain> Ptr;
    typedef std::shared_ptr<const KinematicChain> ConstPtr;
    
    friend XBot::XBotInterface;

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
     * @brief Gets the chain joints group state configuration as specified in the robot SRDF.
     * 
     * @param state_name The name of the requested group state.
     * @param q The chain joint configuration as a map with key representing the joint ID (i.e. numerical name of the joint) and values representing joint positions. This is an output parameter; it will not be cleared before being filled.
     * @return True if the state_name exists in the SRDF, false otherwise.
     */
    bool getChainState(const std::string& state_name, JointIdMap& q) const;
    
    /**
     * @brief Gets the chain joints group state configuration as specified in the robot SRDF.
     * 
     * @param state_name The name of the requested group state.
     * @param q The chain joint configuration as a map with key representing joint names and values representing joint positions. This is an output parameter; it will not be cleared before being filled.
     * @return True if the state_name exists in the SRDF, false otherwise.
     */
    bool getChainState(const std::string& state_name, JointNameMap& q) const;
    
    
    /**
     * @brief Gets the chain joints group state configuration as specified in the robot SRDF.
     * 
     * 
     * @param state_name The name of the requested group state.
     * @param q The chain joint configuration as Eigen vector of joint positions,
     *          This is an output parameter; any contents present in the vector will be overwritten in this function.
     * @return True if the state_name exists in the SRDF, false otherwise.
     */
    bool getChainState(const std::string& state_name, Eigen::VectorXd& q) const;
    
    /**
     * @brief Getter for the force-torque sensor map pertaining to the chain. 
     * 
     * @return A map whose key is the sensor name (i.e. the name of the sensor link inside the URDF) and
     * whose value is a shared pointer to the force torque sensor.
     */
    std::map<std::string, ForceTorqueSensor::ConstPtr> getForceTorque() const;
    
    /**
     * @brief Getter for the IMU sensor map pertaining to the chain. 
     * 
     * @return A map whose key is the sensor name (i.e. the name of the sensor link inside the URDF) and
     * whose value is a shared pointer to the IMU sensor.
     */
    std::map<std::string, ImuSensor::ConstPtr> getImu() const;
    
    /**
     * @brief Returns a force-torque sensor given its parent-link name. 
     * 
     * @param parent_link_name Name of the link to which the sensor is attached
     * @return A shared pointer to the required force-torque sensor. The returned pointer is null if
     * the chain either does not contain a link named "parent_link_name" or such a link does not contain
     * any FT.
     */
    ForceTorqueSensor::ConstPtr getForceTorque(const std::string& parent_link_name) const;
    
    /**
     * @brief Returns an IMU sensor given its parent-link name. 
     * 
     * @param parent_link_name Name of the link to which the sensor is attached
     * @return A shared pointer to the required IMU sensor. The returned pointer is null if
     * the chain either does not contain a link named "parent_link_name" or such a link does not contain
     * any IMU.
     */
    ImuSensor::ConstPtr getImu(const std::string& parent_link_name) const;

    
    /**
     * @brief Gets a vector of the chain joint limits, as specified in the URDF file. The vector is ordered from
     * the chain base link to its tip link.
     * 
     * @param q_min The output vector of the chain joints lower limits.
     * @param q_max The output vector of the chain joints upper limits.
     * @return void
     */
    void getJointLimits(Eigen::VectorXd& q_min, Eigen::VectorXd& q_max) const;
    
    /**
     * @brief Gets a vector of the chain joint velocity limits, as specified in the URDF file. The vector is ordered from
     * the chain base link to its tip link.
     * 
     * @param qdot_max The output vector of the chain joints velocity limits
     * @return void
     */
    void getVelocityLimits(Eigen::VectorXd& qdot_max) const;
    /**
     * 
     * @brief Gets a vector of the chain joint effort limits, as specified in the URDF file. The vector is ordered from
     * the chain base link to its tip link.
     * 
     * @param qdot_max The output vector of the chain joints effort limits
     * @return void
     */    
    void getEffortLimits(Eigen::VectorXd& tau_max) const;
    
    /**
     * @brief Gets the lower and upper joint limits of the i-th joint in the chain (from base link to
     * tip link)
     * 
     * @param i The index of the joint along the chain for which the joint limits are requested.
     * @param q_min The required joint lower limit.
     * @param q_max The required joint upper limit.
     * @return void
     */
    void getJointLimits(int i, double& q_min, double& q_max) const;
    
    /**
     * @brief Gets the velocity limit of the i-th joint in the chain (from base link to
     * tip link)
     * 
     * @param i  The index of the joint along the chain for which the joint limit is requested.
     * @param qdot_max The requested joint velocity limit.
     * @return void
     */
    void getVelocityLimits(int i, double& qdot_max) const;
    
    /**
     * @brief Gets the effort limit of the i-th joint in the chain (from base link to
     * tip link)
     * 
     * @param i  The index of the joint along the chain for which the joint limit is requested.
     * @param tau_max The requested joint effort limit.
     * @return void
     */
    void getEffortLimits(int i, double& tau_max) const;
    
    /**
     * @brief Check the input joint position vector against joint limits. The names of joints violating the
     * limits are pushed into the violating_joints vector (which is not cleared before being filled)
     * 
     * @param q A joint position vector to be checked against joint limits, ordered from base link to tip link.
     * @param violating_joints The vector of the names of joints violating the limits. Note that this is not cleared before use.
     * @return True if all joints are within their limits.
     */
    bool checkJointLimits(const Eigen::VectorXd& q, 
                          std::vector<std::string>& violating_joints) const;

    /**
     * @brief Check the input joint velocity vector against joint limits. The names of joints violating the
     * limits are pushed into the violating_joints vector (which is not cleared before being filled)
     * 
     * @param qdot A joint velocity vector to be checked against joint limits, ordered from base link to tip link.
     * @param violating_joints The vector of the names of joints violating the limits. Note that this is not cleared before use.
     * @return True if all joints are within their limits.
     */                      
    bool checkVelocityLimits(const Eigen::VectorXd& qdot, 
                          std::vector<std::string>& violating_joints) const;

    /**
     * @brief Check the input joint effort vector against joint limits. The names of joints violating the
     * limits are pushed into the violating_joints vector (which is not cleared before being filled)
     * 
     * @param tau A joint effort vector to be checked against joint limits, ordered from base link to tip link.
     * @param violating_joints The vector of the names of joints violating the limits. Note that this is not cleared before use.
     * @return True if all joints are within their limits.
     */
    bool checkEffortLimits(const Eigen::VectorXd& tau, 
                          std::vector<std::string>& violating_joints) const;
                          
    /**
     * @brief Check the input joint position vector against joint limits. 
     * 
     * @param q A joint position vector to be checked against joint limits, ordered from base link to tip link.
     * @return True if all joints are within their limits.
     */
    bool checkJointLimits(const Eigen::VectorXd& q) const;
    
    /**
     * @brief Check the input joint velocity vector against joint limits. 
     * 
     * @param qdot A joint velocity vector to be checked against joint limits, ordered from base link to tip link.
     * @return True if all joints are within their limits.
     */
    bool checkVelocityLimits(const Eigen::VectorXd& qdot) const;
    
    /**
     * @brief Check the input joint effort vector against joint limits. 
     * 
     * @param tau A joint effort vector to be checked against joint limits, ordered from base link to tip link.
     * @return True if all joints are within their limits.
     */
    bool checkEffortLimits(const Eigen::VectorXd& tau) const;
    
    bool enforceJointLimits(Eigen::VectorXd& q) const;
    
    bool enforceVelocityLimit(Eigen::VectorXd& qdot) const;
    
    bool enforceEffortLimit(Eigen::VectorXd& tau) const;
    
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
    const std::string &getChainName() const;

    /**
     * @brief Method returning the name of the chain base link
     *
     * @return Base link name as const std::string&
     */
    const std::string &getBaseLinkName() const;

    /**
     * @brief Method returning the name of the chain tip link
     *
     * @return Tip link name as const std::string&
     */
    const std::string &getTipLinkName() const;

    /**
     * @brief Method returning the name of the child link corresponding
     * to the i-th joint of the chain
     *
     * @param i The position of the joint along the chain (i = 0 refers to the joint attached to the base link)
     *
     * @return The name of the child link of joint n° i
     */
    const std::string &getChildLinkName(int i) const;

    /**
     * @brief Method returning the name of the parent link corresponding
     * to the i-th joint of the chain
     *
     * @param i The position of the joint along the chain (i = 0 refers to the joint attached to the base link)
     *
     * @return The name of the parent link of joint n° i
     */
    const std::string &getParentLinkName(int i) const;

    /**
     * @brief Method returning the name of the i-th joint of the chain
     *
     * @param i The position of the joint whose name is queried (i = 0 refers to the joint attached to the base link)
     *
     * @return The name of the joint n° i
     */
    const std::string& getJointName(int i) const;
    
    
    /**
     * @brief Returns a vector containing the namess of all joints in the chain, from
     * base link to tip link
     * 
     * @return a const reference to the vector of joint names
     */
    const std::vector<std::string>& getJointNames() const;

    /**
     * @brief Method returning the ID of the i-th joint of the chain
     *
     * @param i The position of the joint whose ID is queried (i = 0 refers to the joint attached to the base link)
     *
     * @return The ID of the joint n° i
     */
    int getJointId(int i) const;
    
    /**
     * @brief Getter for the chain dof index (local index of a joint inside a chain, this means 
     *        that the base link of the chain has dof index = 0, tip link of the chain has dof index = getJointNum()-1 )
     *        of a joint with the requested joint id.
     * 
     * @param joint_id the requested joint id
     * @return the chain dof index of a joint with the requested joint id
     */
    int getChainDofIndex(int joint_id) const;
    
    /**
     * @brief Getter for the chain dof index (local index of a joint inside a chain, this means 
     *        that the base link of the chain has dof index = 0, tip link of the chain has dof index = getJointNum()-1 )
     *        of a joint with the requested joint name.
     * 
     * @param joint_name the requested joint name
     * @return the chain dof index of a joint with the requested joint name
     */
    int getChainDofIndex(const std::string& joint_name) const;
    
    /**
     * @brief check if the chain has a joint with a certain id
     * 
     * @param joint_id the requested joint id
     * @return true if the joint with with_id is present in the chain
     */
    bool hasJoint(int joint_id) const;
    
    /**
     * @brief check if the chain has a joint with a certain name
     * 
     * @param joint_name the requested joint name
     * @return true if the joint with joint_name is present in the chain
     */
    bool hasJoint(const std::string& joint_name) const;
    
    /**
     * @brief Returns a vector containing the IDs of all joints in the chain, from
     * base link to tip link
     * 
     * @return a const reference to the vector of joint IDs
     */
    const std::vector<int>& getJointIds() const;

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
    
    
    /**
     * @brief Updates the current object (this) by performing a shallow copy with the chain passed as argument
     * 
     * @param chain the chain to shallow copy
     * @return void
     */
    void shallowCopy(const KinematicChain& chain);
    
    /**
     * @brief Getter for the i-th Joint Ptr 
     * 
     * @param i index inside the chain (i=0 is the child joint of base link)
     * @return A shared pointer to the requested joint
     */
    Joint::ConstPtr getJoint(int i) const;

    // Getters for RX
    
    bool getJointPosition(Eigen::VectorXd &q) const;
    bool getMotorPosition(Eigen::VectorXd &q) const;
    bool getJointVelocity(Eigen::VectorXd &qdot) const;
    bool getMotorVelocity(Eigen::VectorXd &qdot) const;
    bool getJointAcceleration(Eigen::VectorXd &qddot) const;
    bool getJointEffort(Eigen::VectorXd &tau) const;
    bool getTemperature(Eigen::VectorXd &temp) const;

    bool getJointPosition(JointIdMap &q) const;
    bool getMotorPosition(JointIdMap &q) const;
    bool getJointVelocity(JointIdMap &qdot) const;
    bool getMotorVelocity(JointIdMap &qdot) const;
    bool getJointAcceleration(JointIdMap &qddot) const;
    bool getJointEffort(JointIdMap &tau) const;
    bool getTemperature(JointIdMap &temp) const;

    bool getJointPosition(JointNameMap &q) const;
    bool getMotorPosition(JointNameMap &q) const;
    bool getJointVelocity(JointNameMap &qdot) const;
    bool getMotorVelocity(JointNameMap &qdot) const;
    bool getJointAcceleration(JointNameMap &qddot) const;
    bool getJointEffort(JointNameMap &tau) const;
    bool getTemperature(JointNameMap &temp) const;

    double getJointPosition(int index) const;
    double getMotorPosition(int index) const;
    double getJointVelocity(int index) const;
    double getMotorVelocity(int index) const;
    double getJointAcceleration(int index) const;
    double getJointEffort(int index) const;
    double getTemperature(int index) const;
    
    // Setters for RX
    
    bool setJointPosition(const Eigen::VectorXd &q);
    bool setMotorPosition(const Eigen::VectorXd &q);
    bool setJointVelocity(const Eigen::VectorXd &qdot);
    bool setMotorVelocity(const Eigen::VectorXd &qdot);
    bool setJointAcceleration(const Eigen::VectorXd &qddot);
    bool setJointEffort(const Eigen::VectorXd &tau);
    bool setTemperature(const Eigen::VectorXd &temp);

    bool setJointPosition(const JointIdMap &q);
    bool setMotorPosition(const JointIdMap &q);
    bool setJointVelocity(const JointIdMap &qdot);
    bool setMotorVelocity(const JointIdMap &qdot);
    bool setJointAcceleration(const JointIdMap &qddot);
    bool setJointEffort(const JointIdMap &tau);
    bool setTemperature(const JointIdMap &temp);

    bool setJointPosition(const JointNameMap &q);
    bool setMotorPosition(const JointNameMap &q);
    bool setJointVelocity(const JointNameMap &qdot);
    bool setMotorVelocity(const JointNameMap &qdot);
    bool setJointAcceleration(const JointNameMap &qddot);
    bool setJointEffort(const JointNameMap &tau);
    bool setTemperature(const JointNameMap &temp);

    bool setJointPosition(int i, double q);
    bool setMotorPosition(int i, double q);
    bool setJointVelocity(int i, double qdot);
    bool setMotorVelocity(int i, double qdot);
    bool setJointAcceleration(int i, double qddot);
    bool setJointEffort(int i, double tau);
    bool setTemperature(int i, double temp);

    // Getters for TX

    bool getPositionReference(Eigen::VectorXd &q) const;
    bool getVelocityReference(Eigen::VectorXd &qdot) const;
    bool getEffortReference(Eigen::VectorXd &tau) const;
    bool getStiffness(Eigen::VectorXd &K) const;
    bool getDamping(Eigen::VectorXd &D) const;

    bool getPositionReference(JointIdMap &q) const;
    bool getVelocityReference(JointIdMap &qdot) const;
    bool getEffortReference(JointIdMap &tau) const;
    bool getStiffness(JointIdMap &K) const;
    bool getDamping(JointIdMap &D) const;

    bool getPositionReference(JointNameMap &q) const;
    bool getVelocityReference(JointNameMap &qdot) const;
    bool getEffortReference(JointNameMap &tau) const;
    bool getStiffness(JointNameMap &K) const;
    bool getDamping(JointNameMap &D) const;

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

    bool setPositionReference(const JointIdMap &q);
    bool setVelocityReference(const JointIdMap &qdot);
    bool setEffortReference(const JointIdMap &tau);
    bool setStiffness(const JointIdMap &K);
    bool setDamping(const JointIdMap &D);

    bool setPositionReference(const JointNameMap &q);
    bool setVelocityReference(const JointNameMap &qdot);
    bool setEffortReference(const JointNameMap &tau);
    bool setStiffness(const JointNameMap &K);
    bool setDamping(const JointNameMap &D);

    bool setPositionReference(int i, double q);
    bool setVelocityReference(int i, double qdot);
    bool setEffortReference(int i, double tau);
    bool setStiffness(int i, double K);
    bool setDamping(int i, double D);
    
    /**
     * @brief Synchronize the current KinematicChain with another KinematicChain object
     * 
     * @param other The KinematicChain object from which we synchronize the current object
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
     * @return bool
     */
    template <typename... SyncFlags>
    bool syncFrom(const KinematicChain& other, SyncFlags... flags);
    
    /**
     * @brief Prints a pretty table about chain state.
     * 
     * @return void
     */
    void print() const;
    
    /**
     * @brief Prints a pretty table about chain tracking.
     * 
     * @return void
     */
    void printTracking() const;

    friend std::ostream& operator<<(std::ostream& os, const XBot::KinematicChain& c);
    
    

protected:
    
    std::map<std::string, XBot::Joint::Ptr> _joint_name_map;
    std::map<int, XBot::Joint::Ptr> _joint_id_map;
    std::vector<XBot::Joint::Ptr> _joint_vector;
    
    std::vector<ForceTorqueSensor::Ptr> _ft_vector;
    std::map<std::string, ForceTorqueSensor::Ptr> _ft_map;
    
    std::vector<ImuSensor::Ptr> _imu_vector;
    std::map<std::string, ImuSensor::Ptr> _imu_map;
    
    /**
     * @brief Getter for the i-th Joint Ptr 
     * 
     * @param i index inside the chain (i=0 is the child joint of base link)
     * @return A shared pointer to the requested joint
     */
    Joint::Ptr getJointInternal(int i) const;
    
    std::map< std::string, ForceTorqueSensor::Ptr > getForceTorqueInternal() const;
    std::map< std::string, ImuSensor::Ptr > getImuInternal() const;
    
    
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

template <typename... SyncFlags>
bool KinematicChain::syncFrom(const KinematicChain &other, SyncFlags... flags)
{
    
    int pos = 0;
    for (const XBot::Joint::Ptr & j : other._joint_vector) {
        _joint_vector[pos++]->syncFrom(*j, flags...);
    }
}
    
}

#endif // __KINEMATIC_CHAIN_H__
