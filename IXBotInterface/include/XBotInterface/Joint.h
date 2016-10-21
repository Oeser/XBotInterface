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

namespace XBot
{
    
    class KinematicChain;

/**
* @brief Container for the Joint information.
*
*/
class Joint
{
    

public:
    
    friend XBot::KinematicChain;

    /**
     * @brief Default constructor
     *
     */
    Joint();

    /**
     * @brief Construct using the joint name and id
     *
     * @param joint_name the joint name
     * @param joint_id the joint id
     * @param joint_name the parent chain name
     */
    Joint(const std::string &joint_name,
          int joint_id,
          const std::string &chain_name);

    /**
     * @brief Shared pointer to Joint
     * 
     */
    typedef std::shared_ptr<Joint> Ptr;
    
    /**
     * @brief Const shared pointer to Joint
     * 
     */
    typedef std::shared_ptr<const Joint> ConstPtr;

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
     * @brief getter for the joint effort (generalized force)
     * 
     * @return double the joint effort (generalized force)
     */
    double getJointEffort() const;
    
    /**
     * @brief getter for the commanded joint temperature
     * 
     * @return the joint temperature
     */
    double getTemperature() const;

    
    
    
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
    
    
    /**
     * @brief return true if the joint is virtual
     * 
     * @return true if the joint is virtual. False otherwise
     */
    bool isVirtualJoint();
    
    

    void setPositionReference(double pos_ref);
    void setVelocityReference(double vel_ref);
    void setEffortReference(double effort_ref);
    void setStiffness(double stiffness);
    void setDamping(double damping);
    
    
    
    // TBD implement checkPositionLimit(), checkVelocityLimit(), checkEffortLimit()

    /**
     * @brief Synchronize the current Joint with another Joint object
     * 
     * @param other The Joint object from which we synchronize the current object
     * @return True if the synchronization is feasible ( i.e. the two Joint object have exactly the same names/ids). False otherwise
     */
    bool syncFrom(const Joint &other);
    
    /**
     * @brief Set the joint references (TX) from other joint state (RX)
     * 
     * @param other The Joint object which references are read from
     * @return True if the synchronization is feasible ( i.e. the two Joint object have exactly the same names/ids). False otherwise.
     */
    bool setReferenceFrom(const Joint& other);
    
    /**
     * @brief operator << to synchronize the current Joint with another Joint object
     * 
     * @param from the Joint object from which we synchronize the current object
     * @return const XBot::Joint& the current synchornized object
     */
    const XBot::Joint& operator<< ( const XBot::Joint& from);
    
    friend std::ostream& operator<< ( std::ostream& os, const XBot::Joint& j );
    

protected:

    void setJointName(const std::string &joint_name);
    void setJointId(int joint_id);
    
    // TBD

    /**
     * @brief ...
     * 
     * @param link_pos ...
     * @return void
     */
    void setJointPosition(double link_pos);
    
    /**
     * @brief ...
     * 
     * @param motor_pos ...
     * @return void
     */
    void setMotorPosition(double motor_pos);
    
    /**
     * @brief ...
     * 
     * @param link_vel ...
     * @return void
     */
    void setJointVelocity(double link_vel);
    
    /**
     * @brief ...
     * 
     * @param motor_vel ...
     * @return void
     */
    void setMotorVelocity(double motor_vel);
    
    /**
     * @brief ...
     * 
     * @param effort ...
     * @return void
     */
    void setJointEffort(double effort);
    
    /**
     * @brief ...
     * 
     * @param temperature ...
     * @return void
     */
    void setTemperature(double temperature);

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

}

#endif // __JOINT_H__