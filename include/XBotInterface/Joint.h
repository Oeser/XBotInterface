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

#ifndef __JOINT_H__
#define __JOINT_H__

#include <string>
#include <memory>
#include <iostream>
#include <srdfdom_advr/model.h>
#include <XBotInterface/SyncFlags.h>
#include <bprinter/table_printer.h>


#include <XBotInterface/ControlMode.h>



namespace XBot
{
    
    class KinematicChain;
    class RobotChain;
    class RobotInterface;
    class XBotInterface;

/**
* @brief Container for the Joint state and information.
*
*/
class Joint
{
    

public:
    
    typedef std::shared_ptr<Joint> Ptr;
    typedef std::shared_ptr<const Joint> ConstPtr;
    
    friend XBot::KinematicChain;
    friend XBot::RobotChain;
    friend XBot::XBotInterface;
    friend XBot::RobotInterface;

    /**
     * @brief Default constructor
     *
     */
    Joint();

    /**
     * @brief Construct using the joint name, joint id, urdf joint and chain name
     *
     * @param joint_name the joint name
     * @param joint_id the joint id
     * @param urdf_joint shared pointer to urdf::Joint struct
     * @param joint_name the parent chain name
     */
    Joint(const std::string &joint_name,
          int joint_id,
          urdf::JointConstSharedPtr urdf_joint,
          const std::string &chain_name);

    /**
     * @brief Getter for the joint name
     *
     * @return const std::string& joint name
     */
    const std::string &getJointName() const;

    /**
     * @brief Getter for the joint id
     *
     * @return int joint id
     */
    int getJointId() const;
    
    /**
     * @brief Getter for the parent chain name
     *
     * @return const std::string& parent chain name
     */
    const std::string &getChainName() const;
    
    /**
     * @brief Getter for the joint position limits in the urdf
     * 
     * @param qmin lower position limit of the joint
     * @param qmax upper position limit of the joint
     * @return void
     */
    void getJointLimits(double& qmin, double& qmax) const;
    
    /**
     * @brief Getter for the joint velocity limit in the urdf
     * 
     * @param qdot_max joint velocity limit
     * @return void
     */
    void getVelocityLimit(double& qdot_max) const;
    
    /**
     * @brief Getter for the joint effort limit in the urdf
     * 
     * @param tau_max joint effort limit
     * @return void
     */
    void getEffortLimit(double& tau_max) const;
    
    /**
     * @brief check that the param q is inside the position limits
     * 
     * @param q the position to check
     * @return true if the param q is inside the joint position limits
     */
    bool checkJointLimits(double q) const;
    
    /**
     * @brief check that the param qdot is below the joint velocity limit
     * 
     * @param qdot the velocity to check
     * @return true if the param qdot is below the joint velocity limit
     */
    bool checkVelocityLimit(double qdot) const;
    
    /**
     * @brief check that the param tau is below the joint effort limit
     * 
     * @param tau the effort to check
     * @return true if the param tau is below the joint effort limit
     */
    bool checkEffortLimit(double tau) const;
    

    void enforceJointLimits(double& q) const;
    
    void enforceVelocityLimit(double& qdot) const;
    
    void enforceEffortLimit(double& tau) const;
    
    
    
    
    /**
     * @brief Getter for the urdf Joint object
     * 
     * @return a (boost) shared pointer to a const urdf Joint object
     */
    urdf::JointConstSharedPtr getUrdfJoint() const;

    friend std::ostream& operator<< ( std::ostream& os, const XBot::Joint& j );
    
    /**
     * @brief Prints a pretty table about joint state.
     * 
     * @return void
     */
    void print() const;
    
    /**
     * @brief Prints a pretty table about joint tracking error.
     * 
     * @return void
     */
    void printTracking() const;
    
     /**
     * @brief return true if the joint is specified fixed in the model, but inside the controlled_joints group of the SRDF
     * e.g. Hands
     * 
     * @return true if the joint is fixed and controlled
     */
    bool isFixedControlledJoint() const;

protected:
    
    
    ///////////////
    // RX VALUES //
    ///////////////
    
    
    /**
     * @brief Setter for the joint name
     * 
     * @param joint_name the joint name to set
     * @return void
     */
    void setJointName(const std::string &joint_name);
    
    /**
     * @brief Setter for the joint id
     * 
     * @param joint_id the joint id to set
     * @return void
     */
    void setJointId(int joint_id);
    
    /**
     * @brief Setter for the link side encoder reading
     * 
     * @param link_pos the link side encoder reading to set
     * @return void
     */
    void setJointPosition(double link_pos);
    
    /**
     * @brief setter for the motor side encoder reading
     * 
     * @param motor_pos the motor side encoder reading to set
     * @return void
     */
    void setMotorPosition(double motor_pos);
    
    /**
     * @brief setter for the link side velocity
     * 
     * @param link_vel the link side velocity to set
     * @return void
     */
    void setJointVelocity(double link_vel);
    
    /**
     * @brief setter for the motor side velocity
     * 
     * @param motor_vel the motor side velocity to set
     * @return void
     */
    void setMotorVelocity(double motor_vel);
    
    /**
     * @brief setter for the link side acceleration
     * 
     * @param link_acc the link side velocity to set
     * @return void
     */
    void setJointAcceleration(double link_acc);
    
    /**
     * @brief setter for the joint effort (generalized force)
     * 
     * @param effort the joint effort (generalized force) to set
     * @return void
     */
    void setJointEffort(double effort);
    
    /**
     * @brief setter for the joint temperature
     * 
     * @param temperature the joint temperature
     * @return void
     */
    void setTemperature(double temperature);
    
    
    /**
     * @brief getter for the link side encoder reading
     * 
     * @return the link side encoder reading
     */
    double getJointPosition() const;
    
    /**
     * @brief getter for the motor side encoder reading
     * 
     * @return the motor side encoder reading
     */
    double getMotorPosition() const;
    
    /**
     * @brief getter for the link side velocity
     * 
     * @return the link side velocity
     */
    double getJointVelocity() const;
    
    /**
     * @brief getter for the motor side velocity
     * 
     * @return the motor side velocity
     */
    double getMotorVelocity() const;
    
    /**
     * @brief getter for the link side acceleration
     * 
     * @return the link side acceleration
     */
    double getJointAcceleration() const;
    
    /**
     * @brief getter for the joint effort (generalized force)
     * 
     * @return double the joint effort (generalized force)
     */
    double getJointEffort() const;
    
    /**
     * @brief getter for the joint temperature
     * 
     * @return the joint temperature
     */
    double getTemperature() const;
    
    
    ///////////////
    // TX VALUES //
    ///////////////
    
    
    /**
     * @brief setter for the motor position reference 
     * 
     * @param pos_ref the motor position reference to set
     * @return void
     */
    void setPositionReference(double pos_ref);
    
    /**
     * @brief setter for the velocity reference
     * 
     * @param vel_ref the velocity reference to set
     * @return void
     */
    void setVelocityReference(double vel_ref);
    
    /**
     * @brief setter for the effort (generalized force) reference
     * 
     * @param effort_ref the effort (generalized force) reference to set
     * @return void
     */
    void setEffortReference(double effort_ref);
    
    /**
     * @brief set the joint stiffness
     * 
     * @param stiffness the joint stiffness to set
     * @return void
     */
    void setStiffness(double stiffness);
    
    /**
     * @brief set the joint damping
     * 
     * @param damping the joint damping to set
     * @return void
     */
    void setDamping(double damping);
    
    
    /**
     * @brief getter for the commanded motor position reference 
     * 
     * @return the commanded motor position reference
     */
    double getPositionReference() const;
    
    /**
     * @brief getter for the commanded velocity reference
     * 
     * @return the commanded velocity reference
     */
    double getVelocityReference() const;
    
    /**
     * @brief getter for the commanded effort (generalized force) reference
     * 
     * @return the commanded effort (generalized force) reference
     */
    double getEffortReference() const;
    
    /**
     * @brief get the commanded joint stiffness
     * 
     * @return the commanded joint stiffness
     */
    double getStiffness() const;
    
    /**
     * @brief get the commanded joint damping
     * 
     * @return the commanded joint damping
     */
    double getDamping() const;
    
    //////////////////
    // CONTROL MODE //
    //////////////////
    
    void setControlMode(const ControlMode& control_mode);
    
    void getControlMode(ControlMode& control_mode) const;
    
    
    /**
     * @brief Synchronize the current Joint with another Joint object
     * 
     * @param other The Joint object from which we synchronize the current object
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
     * @return True if the synchronization is feasible ( i.e. the two Joint object have exactly the same names/ids). False * otherwise
     */
    template <typename... SyncFlags>
    bool syncFrom(const Joint &other, SyncFlags... flags);
    
    /**
     * @brief Set the joint references (TX) from other joint state (RX)
     * 
     * @param other The Joint object which references are read from
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
     * @return True if the synchronization is feasible ( i.e. the two Joint object have exactly the same names/ids). False * otherwise.
     */
    template <typename... SyncFlags>
    bool setReferenceFrom(const Joint& other, SyncFlags... flags);
    
    /**
     * @brief operator << to synchronize the current Joint with another Joint object
     * 
     * @param from the Joint object from which we synchronize the current object
     * @return const XBot::Joint& the current synchornized object
     */
    const XBot::Joint& operator<< ( const XBot::Joint& from);
    
    /**
     * @brief return true if the joint is virtual
     * 
     * @return true if the joint is virtual. False otherwise
     */
    bool isVirtualJoint();
    
   
    

private:
    ////////////////
    // Joint info //
    ////////////////

    /**
     * @brief the name of the joint
     *
     */
    std::string _joint_name;

    /**
     * @brief the id of the joint
     *
     */
    int _joint_id;

    /**
     * @brief the name of the joint
     *
     */
    std::string _chain_name;
    
    // TBD implement getter and setter
    std::string _joint_control_type; 
    
    
    urdf::JointConstSharedPtr _urdf_joint;
    

    ///////////////////
    // RX from board //
    ///////////////////

    /**
     * @brief link side encoder reading
     * 
     */
    double _link_pos;
    
    /**
     * @brief motor side encoder reading
     * 
     */
    double _motor_pos;
    
    /**
     * @brief link side velocity
     * 
     */
    double _link_vel;
    
    /**
     * @brief motor side velocity
     * 
     */
    double _motor_vel;
    
    /**
     * @brief link side acceleration
     * 
     */
    double _link_acc;
    
    /**
     * @brief joint effort (generalized force)
     * 
     */
    double _effort;
    
    /**
     * @brief joint temperature
     * 
     */
    double _temperature;

    /////////////////
    // TX to board //
    /////////////////

    /**
     * @brief motor side position reference
     * 
     */
    double _pos_ref;
    
    /**
     * @brief velocity reference 
     * 
     */
    double _vel_ref;
    
    /**
     * @brief effort (generalized force) reference
     * 
     */
    double _effort_ref;
    
    /**
     * @brief joint stiffness
     * 
     */
    double _stiffness;
    
    /**
     * @brief joint damping
     * 
     */
    double _damping;
    
    ControlMode _control_mode;
    
    /**
     * @brief initialize joint internal varibles
     * 
     * @return void
     */
    void init();



};

/**
 * @brief ostream operator << for a Joint object
 * 
 * @param os ostream
 * @param j the Joint object to print using ostream
 * @return std::ostream& result ostream
 */
std::ostream& operator<< ( std::ostream& os, const XBot::Joint& j );




template <typename... SyncFlags>
bool XBot::Joint::syncFrom(const XBot::Joint &other, SyncFlags... flags)
{


    if(_joint_name != other._joint_name || _joint_id != other._joint_id){
        std::cerr << "ERROR in " << __func__ << "! Attempt to synchronize joints with different names/ids!" << std::endl;
        return false;
    }

    bool sync_position = false, 
         sync_velocity = false, 
         sync_acceleration = false, 
         sync_effort = false,
         sync_stiffness = false,
         sync_damping;

    parseFlags(sync_position, 
               sync_velocity, 
               sync_acceleration, 
               sync_effort, 
               sync_stiffness,
               sync_damping,
               flags...);




    if(sync_position){
        
        // RX
        _link_pos = other._link_pos;
        _motor_pos = other._motor_pos;
        
        // TX
        _pos_ref = other._pos_ref;
    }
    
    if(sync_velocity){
        
        // RX
        _link_vel = other._link_vel;
        _motor_vel = other._motor_vel;
        
        // TX
        _vel_ref = other._vel_ref;
    }
    
    if(sync_acceleration){
        
        _link_acc = other._link_acc;
    }
    
    if(sync_effort){
        
        // RX
        _effort = other._effort;
        
        // TX
        _effort_ref = other._effort_ref;
    }
    
    if(sync_stiffness){
        _stiffness = other._stiffness;
    }
    
    if(sync_damping){
        _damping = other._damping;
    }
    
    
    ///////////////////
    // RX from board //
    ///////////////////
    
    _temperature = other._temperature;
    
    return true;

}

template <typename... SyncFlags>
bool XBot::Joint::setReferenceFrom(const XBot::Joint& other, SyncFlags... flags)
{
    if(_joint_name != other._joint_name || _joint_id != other._joint_id){
        std::cerr << "ERROR in " << __func__ << "! Attempt to set reference from joint with different name/id!" << std::endl;
        return false;
    }

    bool sync_position = false, 
         sync_velocity = false, 
         sync_acceleration = false, 
         sync_effort = false,
         sync_stiffness = false,
         sync_damping;

    parseFlags(sync_position, 
               sync_velocity, 
               sync_acceleration, 
               sync_effort, 
               sync_stiffness,
               sync_damping,
               flags...);

    /////////////////
    // TX to board //
    /////////////////
    
    if(sync_position){
        _pos_ref = other._link_pos;
    }
    if(sync_velocity){
        _vel_ref = other._link_vel;
    }
    if(sync_effort){
        _effort_ref = other._effort;
    }
    if(sync_stiffness){
        _stiffness = other._stiffness;
    }
    if(sync_damping){
        _damping = other._damping;
    }
    
    return true;
}



}

#endif // __JOINT_H__