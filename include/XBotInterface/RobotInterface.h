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

#include <XBotInterface/XBotInterface.h>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotChain.h>

namespace XBot
{

class RobotInterface : public XBotInterface
{

public:

    typedef std::shared_ptr<RobotInterface> Ptr;
    typedef std::shared_ptr<const RobotInterface> ConstPtr;

    /**
     * @brief Default constructor
     *
     */
    RobotInterface();
    virtual ~RobotInterface(){
      std::cout<<"~RobotInterface()"<<std::endl;
    };


    
    /**
     * @brief Getter for the robot singleton
     *
     * @param path_to_cfg path to the config file where the robot parameters(e.g. control framework, internal model ...) are specified
     * @param any_map a map with objects needed by RobotInterface actual implementations
     * @return a shared pointer to the RobotInterface instance
     */
    static RobotInterface::Ptr getRobot(const std::string& path_to_cfg,
                                        const std::string& robot_name = "",
                                        AnyMapConstPtr any_map = AnyMapConstPtr(),
                                        const std::string& framework = "");

    /**
     * @brief Getter for the internal model instance
     *
     * @return the internal model instance
     */
    ModelInterface& model();

    RobotInterface& operator=(const RobotInterface& other) = delete;
    RobotInterface(const RobotInterface& other) = delete;

    /**
     * @brief Getter for the time in the robot framework
     *
     * @return the time in the robot framework
     */

    virtual double getTime() const = 0;

    /**
     * @brief ...
     *
     * @return bool
     */
    virtual bool isRunning() const = 0;

    /**
     * @brief Reads the current state of the robot calling sense_internal() and read_sensors() implemented by the derived class.
     * It synchronizes the internal model if the sync_model param is true.
     *
     * @param sync_model sync flag: true if the internal model needs to be synchronized with the robot
     * @return true if the sense is successful, false otherwise
     */
    bool sense(bool sync_model = true);

    /**
     * @brief Moves the robot calling the move_internal() implemented by the derived class
     *
     * @return true if the move is successful, false otherwis
     */
    bool move();

    /**
     * @brief Getter for the robot chain with a certain chain_name
     *
     * @param chain_name the requested chain name
     * @return the robot chain with the requested chain_name
     */
    RobotChain& operator()(const std::string& chain_name);
    const RobotChain& operator()(const std::string& chain_name) const;

    /**
     * @brief Getter for the robot chain with a certain chain_name
     *
     * @param chain_name the requested chain name
     * @return the robot chain with the requested chain_name
     */
    RobotChain& chain(const std::string& chain_name);
    const RobotChain& chain(const std::string& chain_name) const;

    /**
     * @brief Getter for the standard kinematic group arm: you can quickly access the i-th arm in the order specified in the SRDF
     *
     * @param arm_id the id of the requested arm (i.e. the index as specifed in the SRDF group
     * @return the arm chain with the requested arm_id
     */
    RobotChain& arm(int arm_id);
    const RobotChain& arm(int arm_id) const;

    /**
     * @brief Getter for the standard kinematic group leg: you can quickly access the i-th leg in the order specified in the SRDF
     *
     * @param arm_id the id of the requested leg (i.e. the index as specifed in the SRDF group
     * @return the leg chain with the requested leg_id
     */
    RobotChain& leg(int leg_id);
    const RobotChain& leg(int leg_id) const;

   /**
    * @brief Sets the robot references according to a ModelInterface.
    * Flags can be specified to select a part of the state to be synchronized.
    *
    * @usage robot.setReferenceFrom(model, XBot::Sync::Position, XBot::Sync::Effort)
    * @usage robot.setReferenceFrom(other_model, XBot::Sync::Position)
    *
    * @param model The ModelInterface whose state is used as a reference for the robot.
    * model.
    * @param flags Flags to specify what part of the model state must be used a reference. By default (i.e. if
    * this argument is omitted) the whole state is used. Otherwise, an arbitrary number of flags
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
    *
    * @return True if the synchronization is allowed, false otherwise.
    */
    template <typename... SyncFlags>
    bool setReferenceFrom(const ModelInterface& model, SyncFlags... flags);

    // Control mode
    bool setControlMode(const ControlMode& control_mode);
    bool setControlMode(const std::string& chain_name, const ControlMode& control_mode);
    bool setControlMode(const std::map<std::string, ControlMode>& control_mode);
    bool setControlMode(const std::map<int, ControlMode>& control_mode);


    void getControlMode(std::map<std::string, ControlMode>& control_mode) const;
    void getControlMode(std::map<int, ControlMode>& control_mode) const;


    // Getters for RX

    using XBotInterface::getJointPosition;
    using XBotInterface::getMotorPosition;
    using XBotInterface::getJointVelocity;
    using XBotInterface::getMotorVelocity;
    using XBotInterface::getJointEffort;
    using XBotInterface::getTemperature;

    /**
     * @brief Get the RX timestamp: it corresponds to the last call to sense()
     *
     * @return the RX timestamp in seconds
     */
    double getTimestampRx() const;


    // Getters for TX

    using XBotInterface::getPositionReference;
    using XBotInterface::getVelocityReference;
    using XBotInterface::getEffortReference;
    using XBotInterface::getStiffness;
    using XBotInterface::getDamping;

    /**
     * @brief Get the TX timestamp: it corresponds to the last call to move()
     *
     * @return the TX timestamp in seconds
     */
    double getTimestampTx() const;

    // Setters for TX

    using XBotInterface::setPositionReference;
    using XBotInterface::setVelocityReference;
    using XBotInterface::setEffortReference;
    using XBotInterface::setStiffness;
    using XBotInterface::setDamping;



protected:

    virtual bool sense_internal() = 0;
    virtual bool read_sensors() = 0;
    virtual bool sense_hands() = 0;
    
    virtual bool move_internal() = 0;
    virtual bool move_hands() = 0;
    
    virtual bool init_robot(const std::string& path_to_cfg, AnyMapConstPtr any_map) = 0;
    virtual bool set_control_mode_internal(int joint_id, const ControlMode& control_mode);

    // Setters for RX

    using XBotInterface::setJointPosition;
    using XBotInterface::setMotorPosition;
    using XBotInterface::setJointVelocity;
    using XBotInterface::setMotorVelocity;
    using XBotInterface::setJointEffort;
    using XBotInterface::setTemperature;
    using XBotInterface::setJointAcceleration;
    using XBotInterface::getJointAcceleration;
    using XBotInterface::sync;

    using XBotInterface::getChainMap;


private:

    virtual bool init_internal(const std::string &path_to_cfg, AnyMapConstPtr any_map);

    using XBotInterface::_chain_map;
    using XBotInterface::_ordered_joint_vector;
    using XBotInterface::_ordered_chain_names;
    std::map<std::string, XBot::RobotChain::Ptr> _robot_chain_map;
    XBot::RobotChain _dummy_chain;

    

    ModelInterface::Ptr _model;

    static std::map<std::string, RobotInterface::Ptr> _instance_ptr_map;

    std::vector<std::string> _model_ordered_chain_name;

    static bool get_path_to_shared_lib(const std::string &path_to_cfg, 
                                       const std::string& framework, 
                                       std::string& path_to_so);

    double _ts_rx;
    double _ts_tx;



};

template <typename... SyncFlags>
bool XBot::RobotInterface::setReferenceFrom ( const XBot::ModelInterface& model, SyncFlags... flags )
{
    bool success = true;
    for (const auto & c : model._model_chain_map) {

        const std::string &chain_name = c.first;
        const ModelChain &chain = *c.second;

        if (_robot_chain_map.count(chain_name)) {
            _robot_chain_map.at(chain_name)->setReferenceFrom(chain, flags...);
        } else {
            if(!chain.isVirtual()){
                std::cerr << "ERROR " << __func__ << " : you are trying to synchronize XBotInterfaces with different chains!!" << std::endl;
                success = false;
            }
        }
    }
    return success;
}

}

#endif // __ROBOT_INTERFACE_H__