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
#include <XBotInterface/TypedefAndEnums.h>

#define LIB_MIDDLE_PATH "/build/install/lib/"

namespace XBot {

class XBotInterface {

public:


     typedef std::shared_ptr<XBotInterface> Ptr;
     typedef std::shared_ptr<const XBotInterface> ConstPtr;

     // Constructor, copy constructor, copy assignment, virtual destructor, init function

     XBotInterface();
     XBotInterface ( const XBotInterface& other );
     XBotInterface& operator= ( const XBotInterface& rhs );
     virtual ~XBotInterface();

     bool init ( const std::string& path_to_cfg );


     // URDF, SRDF getters

     /**
      * @brief Getter for the robot URDF model corresponding to the URDF xml file
      * specified in the YAML config file.
      *
      * @return A reference to const urdf::ModelInterface
      */
     const urdf::ModelInterface& getUrdf() const;

     /**
      * @brief getUrdfPath Getter for the path to the URDF model
      * @return a reference to std::string
      */
     const std::string& getUrdfPath() const;

     /**
      * @brief Getter fot the robot SRDF model corresponding to the SRDF xml file
      * specified in the YAML config file.
      *
      * @return A reference to const srdf_advd::Model
      */
     const srdf_advr::Model& getSrdf() const;

     /**
      * @brief getSrdfPath Getter for the path to the SRDF model
      * @return a reference to std::string
      */
     const std::string& getSrdfPath() const;

     /**
      * @brief Returns the robot URDF xml as a string. The URDF file is specified
      * inside the YAML config file.
      *
      * @return A const reference to the required string.
      */
     const std::string& getUrdfString() const;

     /**
      * @brief Returns the robot SRDF xml as a string. The SRDF file is specified
      * inside the YAML config file.
      *
      * @return A const reference to the required string.
      */
     const std::string& getSrdfString() const;


     // Group states

     /**
     * @brief Gets the robot joints group state configuration as specified in the robot SRDF.
     *
     *
     * @param state_name The name of the requested group state.
     * @param q The chain joint configuration as Eigen vector of joint positions,
     *          This is an output parameter; any contents present in the vector will be overwritten in this function.
     * @return True if the state_name exists in the SRDF, false otherwise.
     */
     bool getRobotState ( const std::string& state_name, Eigen::VectorXd& q ) const;

     /**
     * @brief Gets the robot joints group state configuration as specified in the robot SRDF.
     *
     * @param state_name The name of the requested group state.
     * @param q The robot joint configuration as a map with key representing the joint ID (i.e. numerical name of the joint) and value representing joint positions. This is an output parameter; it will not be cleared before being filled.
     * @return True if the state_name exists in the SRDF, false otherwise.
     */
     bool getRobotState ( const std::string& state_name, JointIdMap& q ) const;

     /**
     * @brief Gets the robot joints group state configuration as specified in the robot SRDF.
     *
     * @param state_name The name of the requested group state.
     * @param q The robot joint configuration as a map with key representing the joint name and value representing joint positions. This is an output parameter; it will not be cleared before being filled.
     * @return True if the state_name exists in the SRDF, false otherwise.
     */
     bool getRobotState ( const std::string& state_name, JointNameMap& q ) const;


     // Kinematic chains

     /**
      * @brief Return a vector of available chain names as strings.
      *
      * @return A vector of available chain names.
      */
     std::vector<std::string> getChainNames() const;

     /**
      * @brief A method for determining if a chain with name "chain_name" is defined
      * inside the interface.
      *
      * @param chain_name The name of the chain whose existence has to be checked.
      * @return True if the required chain exists.
      */
     bool hasChain ( const std::string& chain_name ) const;

     /**
      * @brief Returns the number of legs defined inside the interface. This equals
      * the number of elements of the "legs" group inside the SRDF which was provided in the
      * YAML config file.
      *
      * @return The number of defined legs.
      */
     int legs() const;

     /**
      * @brief Returns the number of arms defined inside the interface. This equals
      * the number of elements of the "arms" group inside the SRDF which was provided in the
      * YAML config file.
      *
      * @return The number of defined arms.
      */
     int arms() const;

     // Joints

     /**
      * @brief Returns a vector of enabled joint names.
      *
      * @return A const reference to the vector of enabled joint names.
      */
     const std::vector<std::string>& getEnabledJointNames() const;

     /**
      * @brief Returns a vector of enabled joint IDs.
      *
      * @return A const reference to the vector of enabled joint IDs.
      */
     const std::vector<int>& getEnabledJointId() const;

     /**
      * @brief Checks that a joint with name "joint_name" is defined as an enabled
      * joint inside the interface.
      *
      * @param joint_name The name of the joint we want to check existence for.
      * @return True if the required joint is defined and enabled.
      */
     bool hasJoint ( const std::string& joint_name ) const;

     /**
      * @brief Getter for the number of enabled joints.
      *
      * @return The number of enabled joints.
      */
     int getJointNum() const;

     /**
      * @brief Gets the joint with name "joint_name" if it is defined as enabled.
      * Otherwise, a null pointer is returned and an error is printed to screen.
      *
      * @param joint_name The name of the required joint.
      * @return A const shared pointer to the required joint. A null pointer is returned if
      * joint_name is not defined as enabled.
      */
     XBot::Joint::ConstPtr getJointByName ( const std::string& joint_name ) const;

     /**
      * @brief Gets the joint with ID "joint_id" if it is defined as enabled.
      * Otherwise, a null pointer is returned and an error is printed to screen.
      * Joint IDs are defined inside a YAML file which is specified in the config
      * file which is used to initialize the interface.
      *
      * @param joint_id The ID of the required joint.
      * @return A const shared pointer to the required joint. A null pointer is returned if
      * a joint with the requested ID is not defined as enabled.
      */
     XBot::Joint::ConstPtr getJointByID ( int joint_id ) const;

     /**
      * @brief Gets the joint with the required dof index. This means that the idx-th
      * entry of all Eigen vectors which are taken as arguments by XBotInterface methods refers
      * to the returned joint.
      *
      * @param idx The dof index of the required joint. Valid indices span from 0 to getJointNum()-1.
      * @return A const shared pointer to the required joint. A null pointer is returned if
      * idx is not a valid index.
      */
     XBot::Joint::ConstPtr getJointByDofIndex ( int idx ) const;

     /**
      * @brief Gets the Eigen ID of the joint with name "joint_name". This means that the i-th
      * entry of all Eigen vectors which are taken as arguments by XBotInterface methods refers
      * to the joint namd "joint_name".
      *
      * @param joint_name The name of the required joint.
      * @return The Eigen index of the required joint. A value of -1 is returned when joint_name is not valid.
      */
     int getDofIndex ( const std::string& joint_name ) const;

     /**
      * @brief Gets the Eigen ID of the joint with ID "joint_id". This means that the i-th
      * entry of all Eigen vectors which are taken as arguments by XBotInterface methods refers
      * to the joint with the given ID.
      *
      * @param joint_id The ID of the required joint.
      * @return The Eigen index of the required joint. A value of -1 is returned when joint_id is not valid.
      */
     int getDofIndex ( int joint_id ) const;

     /**
      * @brief Gets dof indices of all joints inside chain "chain_name".
      *
      * @param chain_name The name of the requested chain.
      * @param ids The output vector of dof indices.
      * @return True if chain_name is a valid chain name.
      */
     bool getDofIndex ( const std::string& chain_name, std::vector<int>& ids ) const;


     // Force-torque sensors

     /**
     * @brief Getter for the force-torque sensor map pertaining to the whole robot.
     *
     * @return A map whose key is the sensor name (i.e. the name of the sensor link inside the URDF) and
     * whose value is a shared pointer to the force torque sensor.
     */
     std::map< std::string, ForceTorqueSensor::ConstPtr > getForceTorque();

     /**
     * @brief Returns a force-torque sensor given its parent-link name.
     *
     * @param parent_link_name Name of the link to which the sensor is attached
     * @return A shared pointer to the required force-torque sensor. The returned pointer is null if
     * the robot either does not contain a link named "parent_link_name" or such a link does not contain
     * any FT.
     */
     ForceTorqueSensor::ConstPtr getForceTorque ( const std::string& parent_link_name ) const;

     // IMU sensors

     /**
     * @brief Getter for the IMU sensor map pertaining to the chain.
     *
     * @return A map whose key is the sensor name (i.e. the name of the sensor link inside the URDF) and
     * whose value is a shared pointer to the IMU sensor.
     */
     std::map< std::string, ImuSensor::ConstPtr > getImu();

     /**
     * @brief Returns an IMU sensor given its parent-link name.
     *
     * @param parent_link_name Name of the link to which the sensor is attached
     * @return A shared pointer to the required IMU sensor. The returned pointer is null if
     * the chain either does not contain a link named "parent_link_name" or such a link does not contain
     * any IMU.
     */
     ImuSensor::ConstPtr getImu ( const std::string& parent_link_name ) const;
		
	/**
     * @brief Converts a state vector for the whole robot to its JointNameMap representation.
     * Note that the output map is not cleared before it is filled. It is the responsibility of
     * the user to do so if required.
     * 
     * @param vector A state vector for the whole robot (its size must match getJointNum())
     * @param name_map The output map to be filled
     * @return True if the input vector is valid.
     */
    bool eigenToMap(const Eigen::VectorXd& vector, JointNameMap& name_map) const;
    
    /**
     * @brief Converts a state vector for the whole robot to its JointIdMap representation.
     * Note that the output map is not cleared before it is filled. It is the responsibility of
     * the user to do so if required.
     * 
     * @param vector A state vector for the whole robot (its size must match getJointNum())
     * @param id_map The output map to be filled
     * @return True if the input vector is valid.
     */
    bool eigenToMap(const Eigen::VectorXd& vector, JointIdMap& id_map) const;
    
    /**
     * @brief Converts a state vector for an arbitrary subset of the robot state (specified 
     * as a JointNameMap) to its Eigen representation.
     * Note that the output vector is resized and set to zero if its size does not match the
     * number of joints. 
     * 
     * @param map A JointNameMap contaning the state of the robot (or a part of it)
     * @param vector The output vector to be filled.
     * @return True if the input map contains valid joints.
     */
    bool mapToEigen(const JointNameMap& map, Eigen::VectorXd& vector) const;
    
    /**
     * @brief Converts a state vector for an arbitrary subset of the robot state (specified 
     * as a JointIdMap) to its Eigen representation.
     * Note that the output vector is resized and set to zero if its size does not match the
     * number of joints. 
     * 
     * @param map A JointIdMap contaning the state of the robot (or a part of it)
     * @param vector The output vector to be filled.
     * @return True if the input map contains valid joints.
     */
    bool mapToEigen(const JointIdMap& map, Eigen::VectorXd& vector) const;



     // Getters for RX

     /**
      * @brief Gets the robot joint positions as an Eigen vector. The joint order inside
      * the vector can be queried by calling the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param q A reference to an Eigen vector to be filled with the requested joint state.
      * If the provided vector has wrong size it is resized automatically.
      * @return bool
      */
     bool getJointPosition ( Eigen::VectorXd& q ) const;

     /**
      * @brief Gets the robot motor positions as an Eigen vector. The joint order inside
      * the vector can be queried by calling the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param q A reference to an Eigen vector to be filled with the requested joint state.
      * If the provided vector has wrong size it is resized automatically.
      * @return bool
      */
     bool getMotorPosition ( Eigen::VectorXd& q ) const;

     /**
      * @brief Gets the robot joint velocities as an Eigen vector. The joint order inside
      * the vector can be queried by calling the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param qdot A reference to an Eigen vector to be filled with the requested joint state.
      * If the provided vector has wrong size it is resized automatically.
      * @return bool
      */
     bool getJointVelocity ( Eigen::VectorXd& qdot ) const;

     /**
      * @brief Gets the robot motor velocities as an Eigen vector. The joint order inside
      * the vector can be queried by calling the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param qdot A reference to an Eigen vector to be filled with the requested joint state.
      * If the provided vector has wrong size it is resized automatically.
      * @return bool
      */
     bool getMotorVelocity ( Eigen::VectorXd& qdot ) const;

     /**
      * @brief Gets the robot joint accelerations as an Eigen vector. The joint order inside
      * the vector can be queried by calling the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param qddot A reference to an Eigen vector to be filled with the requested joint state.
      * If the provided vector has wrong size it is resized automatically.
      * @return bool
      */
     bool getJointAcceleration ( Eigen::VectorXd& qddot ) const;

     /**
      * @brief Gets the robot joint efforts as an Eigen vector. The joint order inside
      * the vector can be queried by calling the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param tau A reference to an Eigen vector to be filled with the requested joint state.
      * If the provided vector has wrong size it is resized automatically.
      * @return bool
      */
     bool getJointEffort ( Eigen::VectorXd& tau ) const;

     /**
      * @brief Gets the robot joint temperatures as an Eigen vector. The joint order inside
      * the vector can be queried by calling the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param temp A reference to an Eigen vector to be filled with the requested joint state.
      * If the provided vector has wrong size it is resized automatically.
      * @return bool
      */
     bool getTemperature ( Eigen::VectorXd& temp ) const;


     /**
      * @brief Gets the robot joint positions as a JointIdMap, i.e. a map whose key
      * represents the joint ID and whose value represents the required joint state.
      *
      * @param q A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getJointPosition ( JointIdMap& q ) const;

     /**
      * @brief Gets the robot motor positions as a JointIdMap, i.e. a map whose key
      * represents the joint ID and whose value represents the required joint state.
      *
      * @param q A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getMotorPosition ( JointIdMap& q ) const;

     /**
      * @brief Gets the robot joint velocities as a JointIdMap, i.e. a map whose key
      * represents the joint ID and whose value represents the required joint state.
      *
      * @param qdot A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getJointVelocity ( JointIdMap& qdot ) const;

     /**
      * @brief Gets the robot motor velocities as a JointIdMap, i.e. a map whose key
      * represents the joint ID and whose value represents the required joint state.
      *
      * @param qdot A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getMotorVelocity ( JointIdMap& qdot ) const;

     /**
      * @brief Gets the robot joint accelerations as a JointIdMap, i.e. a map whose key
      * represents the joint ID and whose value represents the required joint state.
      *
      * @param qddot A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getJointAcceleration ( JointIdMap& qddot ) const;

     /**
      * @brief Gets the robot joint efforts as a JointIdMap, i.e. a map whose key
      * represents the joint ID and whose value represents the required joint state.
      *
      * @param tau A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getJointEffort ( JointIdMap& tau ) const;

     /**
      * @brief Gets the robot joint temperatures as a JointIdMap, i.e. a map whose key
      * represents the joint ID and whose value represents the required joint state.
      *
      * @param temp A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getTemperature ( JointIdMap& temp ) const;


     /**
      * @brief Gets the robot joint postions as a JointNameMap, i.e. a map whose key
      * represents the joint human-readable name and whose value represents the required joint state.
      *
      * @param q A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getJointPosition ( JointNameMap& q ) const;

     /**
      * @brief Gets the robot motor postions as a JointNameMap, i.e. a map whose key
      * represents the joint human-readable name and whose value represents the required joint state.
      *
      * @param q A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getMotorPosition ( JointNameMap& q ) const;

     /**
      * @brief Gets the robot joint velocities as a JointNameMap, i.e. a map whose key
      * represents the joint human-readable name and whose value represents the required joint state.
      *
      * @param qdot A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getJointVelocity ( JointNameMap& qdot ) const;

     /**
      * @brief Gets the robot motor velocities as a JointNameMap, i.e. a map whose key
      * represents the joint human-readable name and whose value represents the required joint state.
      *
      * @param qdot A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getMotorVelocity ( JointNameMap& qdot ) const;

     /**
      * @brief Gets the robot joint accelerations as a JointNameMap, i.e. a map whose key
      * represents the joint human-readable name and whose value represents the required joint state.
      *
      * @param qddot A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getJointAcceleration ( JointNameMap& qddot ) const;

     /**
      * @brief Gets the robot joint efforts as a JointNameMap, i.e. a map whose key
      * represents the joint human-readable name and whose value represents the required joint state.
      *
      * @param tau A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getJointEffort ( JointNameMap& tau ) const;

     /**
      * @brief Gets the robot joint temperatures as a JointNameMap, i.e. a map whose key
      * represents the joint human-readable name and whose value represents the required joint state.
      *
      * @param temp A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getTemperature ( JointNameMap& temp ) const;

     // Setters for RX

     /**
      * @brief Sets the XBotInterface internal joint positions according to the
      * provided vector, which must be ordered as specified by the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param q The vector of input joint states.
      * @return True if the size of the provided vector matches getJointNum().
      */
     bool setJointPosition ( const Eigen::VectorXd& q );

     /**
      * @brief Sets the XBotInterface internal motor positions according to the
      * provided vector, which must be ordered as specified by the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param q The vector of input joint states.
      * @return True if the size of the provided vector matches getJointNum().
      */
     bool setMotorPosition ( const Eigen::VectorXd& q );

     /**
      * @brief Sets the XBotInterface internal joint velocities according to the
      * provided vector, which must be ordered as specified by the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param qdot The vector of input joint states.
      * @return True if the size of the provided vector matches getJointNum().
      */
     bool setJointVelocity ( const Eigen::VectorXd& qdot );

     /**
      * @brief Sets the XBotInterface internal motor velocities according to the
      * provided vector, which must be ordered as specified by the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param qdot The vector of input joint states.
      * @return True if the size of the provided vector matches getJointNum().
      */
     bool setMotorVelocity ( const Eigen::VectorXd& qdot );

     /**
      * @brief Sets the XBotInterface internal joint accelerations according to the
      * provided vector, which must be ordered as specified by the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param qddot The vector of input joint states.
      * @return True if the size of the provided vector matches getJointNum().
      */
     bool setJointAcceleration ( const Eigen::VectorXd& qddot );

     /**
      * @brief Sets the XBotInterface internal joint efforts according to the
      * provided vector, which must be ordered as specified by the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param tau The vector of input joint states.
      * @return True if the size of the provided vector matches getJointNum().
      */
     bool setJointEffort ( const Eigen::VectorXd& tau );

     /**
      * @brief Sets the XBotInterface internal joint temperatures according to the
      * provided vector, which must be ordered as specified by the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param temp The vector of input joint states.
      * @return True if the size of the provided vector matches getJointNum().
      */
     bool setTemperature ( const Eigen::VectorXd& temp );


     /**
      * @brief Sets the XBotInterface internal joint positions according to the input JointIdMap (i.e. a map whose key represents the joint ID and whose value represents the joint state), which
      * can include joint states for an arbitrary subset of the whole robot.
      *
      * @param q The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setJointPosition ( const JointIdMap& q );

     /**
      * @brief Sets the XBotInterface internal motor positions according to the input JointIdMap (i.e. a map whose key represents the joint ID and whose value represents the joint state), which
      * can include joint states for an arbitrary subset of the whole robot.
      *
      * @param q The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setMotorPosition ( const JointIdMap& q );

     /**
      * @brief Sets the XBotInterface internal joint velocities according to the input JointIdMap (i.e. a map whose key represents the joint ID and whose value represents the joint state), which
      * can include joint states for an arbitrary subset of the whole robot.
      *
      * @param qdot The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setJointVelocity ( const JointIdMap& qdot );

     /**
      * @brief Sets the XBotInterface internal motor velocities according to the input JointIdMap (i.e. a map whose key represents the joint ID and whose value represents the joint state), which
      * can include joint states for an arbitrary subset of the whole robot.
      *
      * @param qdot The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setMotorVelocity ( const JointIdMap& qdot );

     /**
      * @brief Sets the XBotInterface internal joint accelerations according to the input JointIdMap (i.e. a map whose key represents the joint ID and whose value represents the joint state), which
      * can include joint states for an arbitrary subset of the whole robot.
      *
      * @param qddot The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setJointAcceleration ( const JointIdMap& qddot );

     /**
      * @brief Sets the XBotInterface internal joint efforts according to the input JointIdMap (i.e. a map whose key represents the joint ID and whose value represents the joint state), which
      * can include joint states for an arbitrary subset of the whole robot.
      *
      * @param tau The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setJointEffort ( const JointIdMap& tau );

     /**
      * @brief Sets the XBotInterface internal motor temperatures according to the input JointIdMap (i.e. a map whose key represents the joint ID and whose value represents the joint state), which
      * can include joint states for an arbitrary subset of the whole robot.
      *
      * @param temp The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setTemperature ( const JointIdMap& temp );

     /**
      * @brief Sets the XBotInterface internal joint positions according to the input
      * JointNameMap (i.e. a map whose key represents the joint name and whose value
      * represents the joint state), which can include joint states for an arbitrary
      * subset of the whole robot.
      *
      * @param q The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setJointPosition ( const JointNameMap& q );

     /**
      * @brief Sets the XBotInterface internal motor positions according to the input
      * JointNameMap (i.e. a map whose key represents the joint name and whose value
      * represents the joint state), which can include joint states for an arbitrary
      * subset of the whole robot.
      *
      * @param q The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setMotorPosition ( const JointNameMap& q );

     /**
      * @brief Sets the XBotInterface internal joint velocities according to the input
      * JointNameMap (i.e. a map whose key represents the joint name and whose value
      * represents the joint state), which can include joint states for an arbitrary
      * subset of the whole robot.
      *
      * @param qdot The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setJointVelocity ( const JointNameMap& qdot );

     /**
      * @brief Sets the XBotInterface internal motor velocities according to the input
      * JointNameMap (i.e. a map whose key represents the joint name and whose value
      * represents the joint state), which can include joint states for an arbitrary
      * subset of the whole robot.
      *
      * @param qdot The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setMotorVelocity ( const JointNameMap& qdot );

     /**
      * @brief Sets the XBotInterface internal joint accelerations according to the input
      * JointNameMap (i.e. a map whose key represents the joint name and whose value
      * represents the joint state), which can include joint states for an arbitrary
      * subset of the whole robot.
      *
      * @param qddot The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setJointAcceleration ( const JointNameMap& qddot );

     /**
      * @brief Sets the XBotInterface internal joint efforts according to the input
      * JointNameMap (i.e. a map whose key represents the joint name and whose value
      * represents the joint state), which can include joint states for an arbitrary
      * subset of the whole robot.
      *
      * @param tau The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setJointEffort ( const JointNameMap& tau );

     /**
      * @brief Sets the XBotInterface internal joint temperatures according to the input
      * JointNameMap (i.e. a map whose key represents the joint name and whose value
      * represents the joint state), which can include joint states for an arbitrary
      * subset of the whole robot.
      *
      * @param temp The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setTemperature ( const JointNameMap& temp );

     // Getters for TX

     /**
      * @brief Gets the robot position references as an Eigen vector. The joint order inside
      * the vector can be queried by calling the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param q A reference to an Eigen vector to be filled with the requested joint state.
      * If the provided vector has wrong size it is resized automatically.
      * @return bool
      */
     bool getPositionReference ( Eigen::VectorXd& q ) const;
     
     /**
      * @brief Gets the robot velocity references as an Eigen vector. The joint order inside
      * the vector can be queried by calling the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param qdot A reference to an Eigen vector to be filled with the requested joint state.
      * If the provided vector has wrong size it is resized automatically.
      * @return bool
      */
     bool getVelocityReference ( Eigen::VectorXd& qdot ) const;
     
     /**
      * @brief Gets the robot effort references as an Eigen vector. The joint order inside
      * the vector can be queried by calling the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param tau A reference to an Eigen vector to be filled with the requested joint state.
      * If the provided vector has wrong size it is resized automatically.
      * @return bool
      */
     bool getEffortReference ( Eigen::VectorXd& tau ) const;
     
     /**
      * @brief Gets the robot stiffness as an Eigen vector. The joint order inside
      * the vector can be queried by calling the getEnabledJointNames/getEnabledJointId
      * methods.
      * 
      * @param K A reference to an Eigen vector to be filled with the requested joint state.
      * If the provided vector has wrong size it is resized automatically.
      * @return bool
      */
     bool getStiffness ( Eigen::VectorXd& K ) const;
     
     /**
      * @brief Gets the robot damping as an Eigen vector. The joint order inside
      * the vector can be queried by calling the getEnabledJointNames/getEnabledJointId
      * methods.
      * 
      * @param D A reference to an Eigen vector to be filled with the requested joint state.
      * If the provided vector has wrong size it is resized automatically.
      * @return bool
      */
     bool getDamping ( Eigen::VectorXd& D ) const;
     

     /**
      * @brief Gets the robot position references as a JointIdMap, i.e. a map whose key
      * represents the joint ID and whose value represents the required joint state.
      *
      * @param q A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getPositionReference ( JointIdMap& q ) const;
     
     /**
      * @brief Gets the robot velocity references as a JointIdMap, i.e. a map whose key
      * represents the joint ID and whose value represents the required joint state.
      *
      * @param qdot A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getVelocityReference ( JointIdMap& qdot ) const;
     
     /**
      * @brief Gets the robot effort references as a JointIdMap, i.e. a map whose key
      * represents the joint ID and whose value represents the required joint state.
      *
      * @param tau A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getEffortReference ( JointIdMap& tau ) const;
     
     /**
      * @brief Gets the robot stiffness as a JointIdMap, i.e. a map whose key
      * represents the joint ID and whose value represents the required joint state.
      *
      * @param K A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getStiffness ( JointIdMap& K ) const;
     
     /**
      * @brief Gets the robot damping as a JointIdMap, i.e. a map whose key
      * represents the joint ID and whose value represents the required joint state.
      *
      * @param D A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getDamping ( JointIdMap& D ) const;

     /**
      * @brief Gets the robot postion references as a JointNameMap, i.e. a map whose key
      * represents the joint human-readable name and whose value represents the required joint state.
      * 
      * @param q A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getPositionReference ( JointNameMap& q ) const;
     
     /**
      * @brief Gets the robot velocity references as a JointNameMap, i.e. a map whose key
      * represents the joint human-readable name and whose value represents the required joint state.
      *
      * @param qdot A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getVelocityReference ( JointNameMap& qdot ) const;
     
     /**
      * @brief Gets the robot effort references as a JointNameMap, i.e. a map whose key
      * represents the joint human-readable name and whose value represents the required joint state.
      *
      * @param tau A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getEffortReference ( JointNameMap& tau ) const;
     
     /**
      * @brief Gets the robot stiffness as a JointNameMap, i.e. a map whose key
      * represents the joint human-readable name and whose value represents the required joint state.
      *
      * @param K A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getStiffness ( JointNameMap& K ) const;
     
     /**
      * @brief Gets the robot damping as a JointNameMap, i.e. a map whose key
      * represents the joint human-readable name and whose value represents the required joint state.
      *
      * @param D A reference to a JointIdMap to be filled with the requested joint state.
      * It is the user responsibility to clear the map before it is filled (if required).
      * @return bool
      */
     bool getDamping ( JointNameMap& D ) const;

     // Setters for TX

     /**
      * @brief Sets the XBotInterface internal joint position references according to the
      * provided vector, which must be ordered as specified by the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param q The vector of input joint states.
      * @return True if the size of the provided vector matches getJointNum().
      */
     bool setPositionReference ( const Eigen::VectorXd& q );
     
     /**
      * @brief Sets the XBotInterface internal joint velocity references according to the
      * provided vector, which must be ordered as specified by the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param qdot The vector of input joint states.
      * @return True if the size of the provided vector matches getJointNum().
      */
     bool setVelocityReference ( const Eigen::VectorXd& qdot );
     
     /**
      * @brief Sets the XBotInterface internal joint effort references according to the
      * provided vector, which must be ordered as specified by the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param tau The vector of input joint states.
      * @return True if the size of the provided vector matches getJointNum().
      */
     bool setEffortReference ( const Eigen::VectorXd& tau );
     
     /**
      * @brief Sets the XBotInterface internal joint stiffness according to the
      * provided vector, which must be ordered as specified by the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param K The vector of input joint states.
      * @return True if the size of the provided vector matches getJointNum().
      */
     bool setStiffness ( const Eigen::VectorXd& K );
     
     /**
      * @brief Sets the XBotInterface internal joint damping according to the
      * provided vector, which must be ordered as specified by the getEnabledJointNames/getEnabledJointId
      * methods.
      *
      * @param D The vector of input joint states.
      * @return True if the size of the provided vector matches getJointNum().
      */
     bool setDamping ( const Eigen::VectorXd& D );

     /**
      * @brief Sets the XBotInterface internal joint position references according to the input JointIdMap (i.e. a map whose key represents the joint ID and whose value represents the joint state), which
      * can include joint states for an arbitrary subset of the whole robot.
      *
      * @param q The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setPositionReference ( const JointIdMap& q );
     
     /**
      * @brief Sets the XBotInterface internal joint velocity references according to the input JointIdMap (i.e. a map whose key represents the joint ID and whose value represents the joint state), which
      * can include joint states for an arbitrary subset of the whole robot.
      *
      * @param qdot The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setVelocityReference ( const JointIdMap& qdot );
     
     /**
      * @brief Sets the XBotInterface internal joint effort references according to the input JointIdMap (i.e. a map whose key represents the joint ID and whose value represents the joint state), which
      * can include joint states for an arbitrary subset of the whole robot.
      *
      * @param tau The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setEffortReference ( const JointIdMap& tau );
     
     /**
      * @brief Sets the XBotInterface internal joint stiffness according to the input JointIdMap (i.e. a map whose key represents the joint ID and whose value represents the joint state), which
      * can include joint states for an arbitrary subset of the whole robot.
      *
      * @param K The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setStiffness ( const JointIdMap& K );
     
     /**
      * @brief  Sets the XBotInterface internal joint damping according to the input JointIdMap (i.e. a map whose key represents the joint ID and whose value represents the joint state), which
      * can include joint states for an arbitrary subset of the whole robot.
      *
      * @param K The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setDamping ( const JointIdMap& D );

     /**
      * @brief Sets the XBotInterface internal joint position references according to the input
      * JointNameMap (i.e. a map whose key represents the joint name and whose value
      * represents the joint state), which can include joint states for an arbitrary
      * subset of the whole robot.
      *
      * @param q The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setPositionReference ( const JointNameMap& q );
     
     /**
      * @brief Sets the XBotInterface internal joint velocity references according to the input
      * JointNameMap (i.e. a map whose key represents the joint name and whose value
      * represents the joint state), which can include joint states for an arbitrary
      * subset of the whole robot.
      *
      * @param qdot The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setVelocityReference ( const JointNameMap& qdot );
     
     /**
      * @brief Sets the XBotInterface internal joint effort references according to the input
      * JointNameMap (i.e. a map whose key represents the joint name and whose value
      * represents the joint state), which can include joint states for an arbitrary
      * subset of the whole robot.
      *
      * @param tau The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setEffortReference ( const JointNameMap& tau );
     
     /**
      * @brief Sets the XBotInterface internal joint stiffness according to the input
      * JointNameMap (i.e. a map whose key represents the joint name and whose value
      * represents the joint state), which can include joint states for an arbitrary
      * subset of the whole robot.
      *
      * @param K The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setStiffness ( const JointNameMap& K );
     
     /**
      * @brief Sets the XBotInterface internal joint damping according to the input
      * JointNameMap (i.e. a map whose key represents the joint name and whose value
      * represents the joint state), which can include joint states for an arbitrary
      * subset of the whole robot.
      *
      * @param D The input JointIdMap
      * @return True if all joint IDs inside the provided map are valid.
      */
     bool setDamping ( const JointNameMap& D );



     // Joint limits

     /**
     * @brief Gets a vector of the robot joint limits, as specified in the URDF file.
     *
     * @param q_min The output vector of the chain joints lower limits.
     * @param q_max The output vector of the chain joints upper limits.
     * @return void
     */
     void getJointLimits ( Eigen::VectorXd& q_min, Eigen::VectorXd& q_max ) const;

     /**
     * @brief Gets a vector of the robot joint velocity limits, as specified in the URDF file.
     *
     * @param qdot_max The output vector of the chain joints velocity limits
     * @return void
     */
     void getVelocityLimits ( Eigen::VectorXd& qdot_max ) const;

     /**
     * @brief Gets a vector of the robot joint effort limits, as specified in the URDF file.
     *
     * @param qdot_max The output vector of the chain joints effort limits
     * @return void
     */
     void getEffortLimits ( Eigen::VectorXd& tau_max ) const;

     /**
     * @brief Check the input joint position vector against joint limits. The names of joints violating the
     * limits are pushed into the violating_joints vector (which is not cleared before being filled)
     *
     * @param q A joint position vector to be checked against joint limits.
     * @param violating_joints The vector of the names of joints violating the limits. Note that this is not cleared before use.
     * @return True if all joints are within their limits.
     */
     bool checkJointLimits ( const Eigen::VectorXd& q,
                             std::vector<std::string>& violating_joints ) const;

     /**
     * @brief Check the input joint velocity vector against joint limits. The names of joints violating the
     * limits are pushed into the violating_joints vector (which is not cleared before being filled)
     *
     * @param qdot A joint position vector to be checked against joint limits.
     * @param violating_joints The vector of the names of joints violating the limits. Note that this is not cleared before use.
     * @return True if all joints are within their limits.
     */
     bool checkVelocityLimits ( const Eigen::VectorXd& qdot,
                                std::vector<std::string>& violating_joints ) const;

     /**
     * @brief Check the input joint effort vector against joint limits. The names of joints violating the
     * limits are pushed into the violating_joints vector (which is not cleared before being filled)
     *
     * @param tau A joint position vector to be checked against joint limits.
     * @param violating_joints The vector of the names of joints violating the limits. Note that this is not cleared before use.
     * @return True if all joints are within their limits.
     */
     bool checkEffortLimits ( const Eigen::VectorXd& tau,
                              std::vector<std::string>& violating_joints ) const;


     /**
     * @brief Check the input joint position vector against joint limits.
     *
     * @param q A joint position vector to be checked against joint limits.
     * @return True if all joints are within their limits.
     */
     bool checkJointLimits ( const Eigen::VectorXd& q ) const;

     /**
     * @brief Check the input joint velocity vector against joint limits.
     *
     * @param qdot A joint velocity vector to be checked against joint limits.
     * @return True if all joints are within their limits.
     */
     bool checkVelocityLimits ( const Eigen::VectorXd& qdot ) const;

     /**
     * @brief Check the input joint effort vector against joint limits.
     *
     * @param tau A joint effort vector to be checked against joint limits.
     * @return True if all joints are within their limits.
     */
     bool checkEffortLimits ( const Eigen::VectorXd& tau ) const;
     
     bool enforceJointLimits( Eigen::VectorXd& q ) const;
     
     bool enforceEffortLimit( Eigen::VectorXd& tau ) const;
     
     bool enforceVelocityLimit( Eigen::VectorXd& qdot ) const;

     /**
      * @brief Synchronize the current XBotInterface with another XBotInterface object
      * 
      * @param flags ...
      * @param other The XBotInterface object from which we synchronize the current object
      * @return True if the synchronization is feasible ( i.e. the two XBotInterface object have the same chains). False * otherwise
      */
     template <typename... SyncFlags>
     bool sync ( const XBotInterface& other, SyncFlags... flags );

     /**
      * @brief Getter for the chain map inside the XBotInterface
      * 
      * @return the chain map inside the XBotInterface
      */
     const std::map<std::string, XBot::KinematicChain::Ptr>&  getChainMap() const;

     friend std::ostream& operator<< ( std::ostream& os, const XBot::XBotInterface& robot );

    /**
     * @brief Print a pretty table about the robot state.
     * 
     * @return void
     */
    void print() const;
    
    /**
     * @brief Print a pretty table about the robot tracking.
     * 
     * @return void
     */
    void printTracking() const;

protected:



     // Chain getter for developers

     const std::map<std::string, ForceTorqueSensor::Ptr>& getForceTorqueInternal() const; // TBD change the Internal with something more meaningful
     const std::map<std::string, ImuSensor::Ptr>& getImuInternal() const; // TBD change the Internal with something more meaningful



     virtual bool init_internal ( const std::string& path_to_cfg ) {
          return true;
     }

     const std::string& getPathToConfig() const;

     static bool computeAbsolutePath ( const std::string& input_path,
                                       const std::string& midlle_path,
                                       std::string& absolute_path );

     const std::vector<std::string>& getModelOrderedChainName() const;


     // internal XBotCoreModel object: it does the trick using URDF, SRDF and joint map configuration
     XBotCoreModel _XBotModel;

     std::map<std::string, XBot::KinematicChain::Ptr> _chain_map;
     std::vector<Joint::Ptr> _ordered_joint_vector;
     std::map<std::string, ForceTorqueSensor::Ptr> _ft_map;
     std::map<std::string, ImuSensor::Ptr> _imu_map;
     std::vector<std::string> _ordered_chain_names;

     std::map<int, int> _joint_id_to_eigen_id;
     std::map<std::string, int> _joint_name_to_eigen_id;

private:

     int _joint_num;

     std::string _urdf_string, _srdf_string;
     std::string _urdf_path;
     std::string _srdf_path;
     std::string _joint_map_config;

     std::vector<std::string> _ordered_joint_name;
     std::vector<int> _ordered_joint_id;

     std::string _path_to_cfg;

     bool parseYAML ( const std::string &path_to_cfg );


};

std::ostream& operator<< ( std::ostream& os, const XBot::XBotInterface& robot );

template <typename... SyncFlags>
bool XBot::XBotInterface::sync ( const XBot::XBotInterface &other, SyncFlags... flags )
{
     bool success = true;
     for ( const auto & c : other._chain_map ) {
          const std::string &chain_name = c.first;
          const KinematicChain &chain = *c.second;
          if ( _chain_map.count ( chain_name ) ) {
               _chain_map.at ( chain_name )->syncFrom ( chain, flags... );

          } else {
               if ( !chain.isVirtual() ) {
                    std::cerr << "ERROR " << __func__ << " : you are trying to synchronize XBotInterfaces with different chains!!" << std::endl;
                    success = false;
               }
          }
     }
     return success;
}

}

#endif // __I_XBOT_INTERFACE_H__
