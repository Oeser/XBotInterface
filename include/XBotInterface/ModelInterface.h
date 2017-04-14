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

#ifndef __MODEL_INTERFACE_H__
#define __MODEL_INTERFACE_H__


#include <vector>
#include <map>

#include <SharedLibraryClass.h>
#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames_io.hpp>
#include <eigen_conversions/eigen_kdl.h>

#include <XBotInterface/XBotInterface.h>
#include <XBotInterface/ModelChain.h>

namespace XBot {

class RobotInterface;

/**
 * @brief ModelInterface is an abstraction layer for a kinematic/dynamic
 * model of a robot. It inherits from the XBotInterface class the
 * organization of a robot as a collection of kinematic chains.
 */
class ModelInterface : public XBotInterface {

public:


    friend XBot::RobotInterface;

    /**
     * @brief Shared pointer to a ModelInterface
     */
    typedef std::shared_ptr<ModelInterface> Ptr;

    /**
     * @brief Shared pointer to const ModelInterface
     */
    typedef std::shared_ptr<ModelInterface> ConstPtr;

    // ModelInterface must be default constructible and non-copyable
    ModelInterface() = default;
    ModelInterface& operator=(const ModelInterface& other) = delete;
    ModelInterface(const ModelInterface& other) = delete;


    /**
     * @brief Factory method for generating instances of the ModelInterface class
     * according to the provided configuration file, which specifies the URDF/SRDF
     * to be used, as well as the specific implementation of ModelInterface to be
     * loaded.
     *
     * @param path_to_cfg Path to the YAML configuration file.
     * @param any_map a map with objects needed by RobotInterface actual implementations
     * @return A shared pointer to a new instance of ModelInterface.
     */
    static ModelInterface::Ptr getModel(const std::string &path_to_cfg, AnyMapConstPtr any_map = AnyMapConstPtr());


    /**
     * @brief Returns a handle to the kinematic chain named chain_name.
     * If such a chain is not defined, a dummy chain is returned and an
     * error is displayed.
     *
     * @param chain_name The name of the requested chain
     * @return A reference to the requested kinematic chain
     */
    ModelChain& chain(const std::string& chain_name);

    /**
     * @brief Returns a handle to the kinematic chain named chain_name.
     * If such a chain is not defined, a dummy chain is returned and an
     * error is displayed.
     *
     * @param chain_name The name of the requested chain
     * @return A reference to the requested kinematic chain
     */
    ModelChain& operator()(const std::string& chain_name);

    /**
     * @brief Returns a handle to the kinematic chain corresponding to the
     * arm n° arm_id. The order of arms is defined in the SRDF file which
     * was provided to ModelInterface::getModel(). If the requested chain
     * is not defined, a dummy chain is returned and an error is displayed.
     *
     * @param arm_id The requested arm id, from 0 to arms()-1
     * @return A reference to the requested kinematic chain
     */
    ModelChain& arm(int arm_id);

    /**
     * @brief Returns a handle to the kinematic chain corresponding to the
     * leg n° leg_id. The order of legs is defined in the SRDF file which
     * was provided to ModelInterface::getModel(). If the requested chain
     * is not defined, a dummy chain is returned and an error is displayed.
     *
     * @param arm_id The requested leg id, from 0 to legs()-1
     * @return A reference to the requested kinematic chain
     */
    ModelChain& leg(int leg_id);

    /**
     * @brief Synchronizes the internal model state to the one of the XBotInterface
     * given as an argument. This can be either another ModelInterface or a RobotInterface.
     * Flags can be specified to select a part of the state to be synchronized.
     * ModelInterface::update() is automatcally called, so that the kinematics and
     * dynamics of the ModelInterface is updated as well.
     *
     * @usage model.syncFrom(other_model, XBot::Sync::Position, XBot::Sync::Effort)
     * @usage model.syncFrom(other_model, XBot::Sync::Position)
     *
     * @param other The RobotInterface or ModelInterface whose state is copied to this
     * model.
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

     * @return True if the synchronization is allowed, false otherwise.
     */
    template <typename... SyncFlags>
    bool syncFrom(const XBotInterface& other, SyncFlags... flags);


    /**
     * @brief Synchronizes the internal model state to the one of the XBotInterface
     * given as an argument. This can be either another ModelInterface or a RobotInterface.
     * Flags can be specified to select a part of the state to be synchronized.
     * As the only difference with ModelInterface::syncFrom(), ModelInterface::update() is NOT automatcally called.
     *
     * @usage model.syncFrom(other_model, XBot::Sync::Position, XBot::Sync::Effort)
     * @usage model.syncFrom(other_model, XBot::Sync::Position)
     *
     * @param other The RobotInterface or ModelInterface whose state is copied to this
     * model.
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

     * @return True if the synchronization is allowed, false otherwise.
     */
    template <typename... SyncFlags>
    bool syncFrom_no_update(const XBotInterface& other, SyncFlags... flags);

    /**
     * @brief Updates the kinematic variables of the model according to the current state of the model.
     * When called without arguments, it only updates joint positions (no velocities/accelerations).
     *
     * @param update_position True if you want to update the positions. False otherwise.
     * @param update_velocity True if you want to update the velocities. False otherwise.
     * @param update_desired_acceleration True if you want to update the desired accelerations. False otherwise.
     * @return True if the update is successful. False otherwise.
     */
    virtual bool update( bool update_position = true,
                         bool update_velocity = true,
                         bool update_desired_acceleration = true ) = 0;

    /**
    * @brief Gets a vector with the joint names ordered according to the model. If the model is floating-base,
    * the first six names are the virtual joint names.
    *
    * @param joint_name The joint names ordered according to the model.
    * @return void
    */
    virtual void getModelOrderedJoints( std::vector<std::string>& joint_name ) const = 0;

    /**
     * @brief True if the robot is floating base, false if it has a fixed base.
     *
     * @return True if the robot is floating base, false if it has a fixed base.
     */
    bool isFloatingBase() const;

    /**
    * @brief Sets the floating base pose w.r.t. the world frame
    *
    * @param floating_base_pose A homogeneous transformation which transforms a point from floating base frame to world frame
    * @return True if the floating_base_pose frame is valid. False otherwise.
    */
    virtual bool setFloatingBasePose( const KDL::Frame& floating_base_pose ) = 0;

    /**
    * @brief Computes the pose of the source_frame w.r.t. the world frame
    *
    * @param source_frame The source link name.
    * @param pose A homogeneous transformation which transforms a point from source frame to world frame
    * @return True if source_frame is valid. False otherwise.
    */
    virtual bool getPose( const std::string& source_frame,
                          KDL::Frame& pose ) const = 0;

    /**
    * @brief Computes the pose of the target_frame w.r.t. the source_frame
    *
    * @param source_frame The source link name. If you want it w.r.t. the world frame, pass "world"
    * @param target_frame The target link name. If you want it w.r.t. the world frame, pass "world"
    * @param pose A homogeneous transformation which transforms a point from source frame to target frame
    * @return True if both source_frame and target_frame are valid. False otherwise.
    */
    bool getPose( const std::string& source_frame,
                  const std::string& target_frame,
                  KDL::Frame& pose ) const;

    /**
    * @brief Computes the orientation of the source_frame w.r.t. the world frame
    *
    * @param source_frame The source link name.
    * @param orientation A rotation matrix which rotates a vector from source frame to world frame
    * @return True if source_frame is valid. False otherwise.
    */
    bool getOrientation(const std::string& source_frame,
                        KDL::Rotation& orientation) const;

    /**
    * @brief Computes the orientation of the target_frame w.r.t. the source_frame
    *
    * @param source_frame The source link name. If you want it w.r.t. the world frame, pass "world"
    * @param target_frame The target link name. If you want it w.r.t. the world frame, pass "world"
    * @param orientation A rotation matrix which rotates a vector from source frame to target frame
    * @return True if both source_frame and target_frame are valid. False otherwise.
    */
    bool getOrientation(const std::string& source_frame,
                        const std::string& target_frame,
                        KDL::Rotation& orientation) const;


    /**
    * @brief Gets the position in world coordinates for a source_point expressed w.r.t. a source_frame.
    *
    * @param source_frame The frame according to which source_point is expressed.
    * @param source_point The source_point in source-frame coordinates.
    * @param world_point The requested point in world coordinates.
    * @return True if source_frame is a valid link name.
    */
    bool getPointPosition(const std::string& source_frame,
                          const KDL::Vector& source_point,
                          KDL::Vector& world_point) const;

    /**
    * @brief Gets the position in target_frame coordinates for a source_point expressed w.r.t. source_frame.
    *
    * @param source_frame The source link name.
    * @param target_frame The target link name.
    * @param source_point The source_point in world coordinates.
    * @param target_point The requested point in target_frame coordinates.
    * @return True if target_frame and source_frame are valid link names.
    */
    bool getPointPosition(const std::string& source_frame,
                          const std::string& target_frame,
                          const KDL::Vector& source_point,
                          KDL::Vector& target_point) const;
    /**
     * @brief Gets the Jacobian of link_name expressed in the world frame, i.e a matrix such that its product with
     * the derivative of the configuration vector gives the velocity twist of link_name (i.e. first linear then angular velocity). The reference point of the Jacobian is the reference_point arg.
     *
     * @param link_name The link name
     * @param reference_point The reference point w.r.t. which the jacobian is computed
     * @param J The Jacobian expressed in the world frame
     * @return True if the link_name is a valid link name. False otherwise.
     */
    virtual bool getJacobian( const std::string& link_name,
                              const KDL::Vector& reference_point,
                              KDL::Jacobian& J) const = 0;

    /**
     * @brief Gets the Jacobian of link_name expressed in the world frame, i.e a matrix such that its product with
     * the derivative of the configuration vector gives the velocity twist of link_name (i.e. first linear then angular velocity). The reference point is the origin of the link with link_name name.
     *
     * @param link_name The link name
     * @param J  The Jacobian expressed in the world frame
     * @return True if the link_name is a valid link name. False otherwise.
     */
    bool getJacobian( const std::string& link_name,
                      KDL::Jacobian& J) const;

    /**
     * @brief Gets the relative Jacobian of target_link_name w.r.t to base_link_name.
     * The result is represented in the reference frame of base_link_name.
     *
     * @param target_link_name The name of the target link
     * @param base_link_name The name of the base link
     * @param J The requested jacobian.
     * @return bool
     */
    virtual bool getRelativeJacobian( const std::string& target_link_name,
                              const std::string& base_link_name,
                              KDL::Jacobian& J) const;

    /**
    * @brief Gets the velocity twist of link_name (first linear, then angular velocity).
    * The reference point of the velocity twist is the origin of the link itself.
    *
    * @param link_name The link name
    * @param velocity The twist of the link "link_name"
    * @return True if the link_name is a valid link name. False otherwise.
    */
    virtual bool getVelocityTwist( const std::string& link_name,
                                     KDL::Twist& velocity) const = 0;

    /**
    * @brief Gets the acceleration twist of link_name (first linear, then angular acceleration).
    * The reference point of the acceleration twist is the origin of the link itself.
    *
    * @param link_name The link name
    * @param velocity The acceleration twist of the link "link_name"
    * @return True if the link_name is a valid link name. False otherwise.
    */
    virtual bool getAccelerationTwist( const std::string& link_name,
                                         KDL::Twist& acceleration) const = 0;


    /**
    * @brief Gets the acceleration of a given point which is solidal with the given link.
    *
    * @param link_name The link which point is attached to.
    * @param point A point in link frame coordinates.
    * @param acceleration The acceleration of point.
    * @return bool True if link_name is a valid link name.
    */
    virtual bool getPointAcceleration(const std::string& link_name,
                                      const KDL::Vector& point,
                                      KDL::Vector& acceleration) const = 0;

    /**
    * @brief Computed the product of the link_name jacobian time derivative by the joint velocity
    * vector. The reference point of the jacobian is provided as the argument "point".
    *
    * @param link_name The link name
    * @param point A point in link_name coordinates-
    * @param jdotqdot The resulting acceleration twist.
    * @return True if link_name is a valid link name.
    */
    virtual bool computeJdotQdot(const std::string& link_name,
                         const KDL::Vector& point,
                         KDL::Twist& jdotqdot) const = 0;

    /**
     * @brief Gets the COM position vector w.r.t. the world frame
     *
     * @param com_position The center of mass position vector w.r.t. the world frame
     * @return void
     */
    virtual void getCOM( KDL::Vector& com_position ) const = 0;

    /**
     * @brief Gets the COM position vector w.r.t. the reference_frame
     *
     * @param reference_frame The link name w.r.t. which the COM position will be expressed
     * @param com_position The center of mass position vector w.r.t. the reference_frame
     * @return True if the reference_frame is a valid link name. False otherwise.
     */
    bool getCOM( const std::string& reference_frame,
                         KDL::Vector& com_position ) const;

    /**
     * @brief Gets the COM Jacobian expressed in the world frame
     *
     * @param J The COM Jacobian expressed in the world frame. Note that, since the COM is not fixed
     * to any link, the Jacobian orientation part (i.e. the lower three rows) are undefined and filled with
     * zeros.
     * @return void
     */
    virtual void getCOMJacobian( KDL::Jacobian& J) const = 0;

    /**
     * @brief Gets the COM velocity expressed in the world frame
     *
     * @param velocity The COM velocity expressed in the world frame
     * @return void
     */
    virtual void getCOMVelocity( KDL::Vector& velocity) const = 0;

    /**
     * @brief Gets the COM acceleration expressed in the world frame
     *
     * @param velocity The COM acceleration expressed in the world frame
     * @return void
     */
    virtual void getCOMAcceleration( KDL::Vector& acceleration) const = 0;

    /**
     * @brief Gets the gravity vector expressed in the world frame
     *
     * @param gravity The gravity vector expressed in the world frame.
     * @return void
     */
    virtual void getGravity( KDL::Vector& gravity ) const = 0;

    /**
     * @brief  Gets the gravity vector expressed in the reference_frame
     *
     * @param reference_frame The link name w.r.t. which the gravity vector will be expressed
     * @param gravity The gravity vector expressed in the world frame
     * @return True if the reference_frame is a valid link name. False otherwise.
     */
    bool getGravity( const std::string& reference_frame,
                     KDL::Vector& gravity ) const;
    /**
     * @brief Sets the gravity vector expressed in the world frame
     *
     * @param gravity The gravity vector expressed in the world frame.
     * @return void
     */
    virtual void setGravity( const KDL::Vector& gravity ) = 0;

    /**
     * @brief  Sets the gravity vector expressed in the reference_frame
     *
     * @param reference_frame The link name w.r.t. which the gravity vector will be expressed
     * @param gravity The gravity vector expressed in the reference_frame
     * @return True if the reference_frame is a valid link name. False otherwise.
     */
    bool setGravity( const std::string& reference_frame,
                     const KDL::Vector& gravity );

    /**
     * @brief Gets the joint space inertia matrix of the robot.
     *
     * @param M The requested inertia matrix.
     * @return void
     */
    virtual void getInertiaMatrix(Eigen::MatrixXd& M) const = 0;


    /**
     * @brief Computes gravity compensation torques. Make sure that you correctly specified
     * the gravity vector in order to obtain correct results.
     *
     * @param g The gravity compensation torque vector (if model is floating base, includes virtual joints effort).
     * @return void
     */
    virtual void computeGravityCompensation( Eigen::VectorXd& g ) const = 0;

    /**
     * @brief Computes the non-linear torque term, i.e. the sum of centrifugal/coriolis/gravity contributions. Make sure that you correctly specified the gravity vector in order to obtain correct results.
     *
     * @param n The non-linear torques vector (if model is floating base, includes virtual joints effort).
     * @return void
     */
    virtual void computeNonlinearTerm( Eigen::VectorXd& n ) const = 0; //TBD termss

    /**
     * @brief Computes inverse dynamics.
     *
     * @param tau The inverse dynamics torque vector (if model is floating base, includes virtual joints effort).
     * @return void
     */
    virtual void computeInverseDynamics( Eigen::VectorXd& tau) const = 0;

    /**
     * @brief Computes inverse dynamics for a constrained system.
     *
     * @param J The constraint jacobian
     * @param weights Torque weights for weighted least-squares computation (only matters if system is floating-base)
     * @param tau The resultind ID torque vector
     * @return True if input has correct dimensions and computation goes alright.
     */
    bool computeConstrainedInverseDynamics( const Eigen::MatrixXd& J, const Eigen::VectorXd& weights, Eigen::VectorXd& tau) const;

    /**
     * @brief Gets a selection matrix for the requested chain, i.e. a matrix S such that the product
     * of S by the configuration vector returns the sub-vector pertaining to the specified chain.
     *
     * @param chain_name The chain name.
     * @param S The requested selection matrix.
     * @return True if chain_name is a valid chain name.
     */
    bool getChainSelectionMatrix( const std::string& chain_name, Eigen::MatrixXd& S ) const;

    /**
     * @brief Gets a selection matrix for the requested joint, i.e. a matrix S such that the product
     * of S by the configuration vector returns the value pertaining to the specified joint.
     *
     * @param joint_name The chain name.
     * @param S The requested selection matrix.
     * @return True if joint_name is a valid joint name.
     */
    bool getJointSelectionMatrix( const std::string& joint_name, Eigen::RowVectorXd& S ) const;

    /**
     * @brief Gets a selection matrix for the requested joint, i.e. a matrix S such that the product
     * of S by the configuration vector returns the value pertaining to the specified joint.
     *
     * @param joint_id The chain ID.
     * @param S The requested selection matrix.
     * @return True if joint_id is a valid joint ID.
     */
    bool getJointSelectionMatrix( int joint_id, Eigen::RowVectorXd& S ) const;

    /**
     * @brief Applies a mask corresponding to the specified chain to the jacobian provided as an argument.
     * This amounts to setting the columns of the jacobian corresponding the the chain to zero.
     *
     * @param chain_name The name of the chain.
     * @param J The jacobian which the mask has to be applied to.
     * @return True if chain_name is a valid chain name and J has the correct row size.
     */
    bool maskJacobian( const std::string& chain_name, KDL::Jacobian& J) const;

    // EIGEN OVERLOADS
    /**
    * @brief Sets the floating base pose w.r.t. the world frame
    *
    * @param floating_base_pose A homogeneous transformation which transforms a point from floating base frame to world frame
    * @return True if the floating_base_pose frame is valid. False otherwise.
    */
    bool setFloatingBasePose( const Eigen::Affine3d& floating_base_pose );

    /**
    * @brief Computes the pose of the target_frame w.r.t. the source_frame
    *
    * @param source_frame The source link name. If you want it w.r.t. the world frame, pass "world"
    * @param target_frame The target link name. If you want it w.r.t. the world frame, pass "world"
    * @param pose A homogeneous transformation which transforms a point from source frame to target frame
    * @return True if both source_frame and target_frame are valid. False otherwise.
    */
    bool getPose(   const std::string& source_frame,
                    const std::string& target_frame,
                    Eigen::Affine3d& pose ) const;
    /**
    * @brief Computes the pose of the source_frame w.r.t. the world frame
    *
    * @param source_frame The source link name.
    * @param pose A homogeneous transformation which transforms a point from source frame to world frame
    * @return True if source_frame is valid. False otherwise.
    */
    bool getPose( const std::string& source_frame,
                  Eigen::Affine3d& pose ) const;

    /**
    * @brief Computes the orientation of the source_frame w.r.t. the world frame
    *
    * @param source_frame The source link name.
    * @param orientation A rotation matrix which rotates a vector from source frame to world frame
    * @return True if source_frame is valid. False otherwise.
    */
    bool getOrientation(    const std::string& target_frame,
                            Eigen::Matrix3d& orientation) const;
    /**
    * @brief Computes the orientation of the target_frame w.r.t. the source_frame
    *
    * @param source_frame The source link name. If you want it w.r.t. the world frame, pass "world"
    * @param target_frame The target link name. If you want it w.r.t. the world frame, pass "world"
    * @param orientation A rotation matrix which rotates a vector from source frame to target frame
    * @return True if both source_frame and target_frame are valid. False otherwise.
    */
    bool getOrientation(    const std::string& source_frame,
                            const std::string& target_frame,
                            Eigen::Matrix3d& orientation) const;
    /**
    * @brief Gets the position in world coordinates for a source_point expressed w.r.t. a source_frame.
    *
    * @param source_frame The frame according to which source_point is expressed.
    * @param source_point The source_point in source-frame coordinates.
    * @param world_point The requested point in world coordinates.
    * @return True if source_frame is a valid link name.
    */
    bool getPointPosition(  const std::string& source_frame,
                            const Eigen::Vector3d& source_point,
                            Eigen::Vector3d& world_point) const;

    /**
    * @brief Gets the position in target_frame coordinates for a source_point expressed w.r.t. source_frame.
    *
    * @param source_frame The source link name.
    * @param target_frame The target link name.
    * @param source_point The source_point in world coordinates.
    * @param target_point The requested point in target_frame coordinates.
    * @return True if target_frame and source_frame are valid link names.
    */
    bool getPointPosition(  const std::string& source_frame,
                            const std::string& target_frame,
                            const Eigen::Vector3d& source_point,
                            Eigen::Vector3d& target_point) const;

    /**
     * @brief Gets the Jacobian of link_name expressed in the world frame, i.e a matrix such that its product with
     * the derivative of the configuration vector gives the velocity twist of link_name (i.e. first linear then angular velocity). The reference point is the origin of the link with link_name name.
     *
     * @param link_name The link name
     * @param J  The Jacobian expressed in the world frame
     * @return True if the link_name is a valid link name. False otherwise.
     */
    bool getJacobian( const std::string& link_name,
                      Eigen::MatrixXd& J);

    /**
     * @brief Gets the Jacobian of link_name expressed in the world frame, i.e a matrix such that its product with
     * the derivative of the configuration vector gives the velocity twist of link_name (i.e. first linear then angular velocity). The reference point of the Jacobian is the reference_point arg.
     *
     * @param link_name The link name
     * @param reference_point The reference point w.r.t. which the jacobian is computed
     * @param J The Jacobian expressed in the world frame
     * @return True if the link_name is a valid link name. False otherwise.
     */
    bool getJacobian( const std::string& link_name,
                      const Eigen::Vector3d& reference_point,
                      Eigen::MatrixXd& J);

    /**
     * @brief Gets the relative Jacobian of target_link_name w.r.t to base_link_name.
     * The result is represented in the reference frame of base_link_name.
     *
     * @param target_link_name The name of the target link
     * @param base_link_name The name of the base link
     * @param J The requested jacobian.
     * @return bool
     */
    bool getRelativeJacobian( const std::string& target_link_name,
                              const std::string& base_link_name,
                              Eigen::MatrixXd& J) const;

    /**
     * @brief Gets the COM position vector w.r.t. the world frame
     *
     * @param com_position The center of mass position vector w.r.t. the world frame
     * @return void
     */
    void getCOM( Eigen::Vector3d& com_position ) const;

    /**
     * @brief Gets the COM position vector w.r.t. the reference_frame
     *
     * @param reference_frame The link name w.r.t. which the COM position will be expressed
     * @param com_position The center of mass position vector w.r.t. the reference_frame
     * @return True if the reference_frame is a valid link name. False otherwise.
     */
    bool getCOM( const std::string& reference_frame,
                 Eigen::Vector3d& com_position ) const;

    /**
     * @brief Gets the COM Jacobian expressed in the world frame
     *
     * @param J The COM Jacobian expressed in the world frame. Note that, since the COM is not fixed
     * to any link, the Jacobian orientation part (i.e. the lower three rows) are undefined and filled with
     * zeros.
     * @return void
     */
    void getCOMJacobian( Eigen::MatrixXd& J) const;

    /**
     * @brief Gets the COM velocity expressed in the world frame
     *
     * @param velocity The COM velocity expressed in the world frame
     * @return void
     */
    void getCOMVelocity( Eigen::Vector3d& velocity) const;

    /**
     * @brief Gets the COM acceleration expressed in the world frame
     *
     * @param velocity The COM acceleration expressed in the world frame
     * @return void
     */
    void getCOMAcceleration( Eigen::Vector3d& acceleration) const;

    /**
    * @brief Gets the velocity twist of link_name (first linear, then angular velocity).
    * The reference point of the velocity twist is the origin of the link itself.
    *
    * @param link_name The link name
    * @param velocity The twist of the link "link_name"
    * @return True if the link_name is a valid link name. False otherwise.
    */
    bool getVelocityTwist( const std::string& link_name,
                             Eigen::Matrix<double,6,1>& velocity) const;

    /**
    * @brief Gets the acceleration twist of link_name (first linear, then angular acceleration).
    * The reference point of the acceleration twist is the origin of the link itself.
    *
    * @param link_name The link name
    * @param velocity The acceleration twist of the link "link_name"
    * @return True if the link_name is a valid link name. False otherwise.
    */
    bool getAccelerationTwist( const std::string& link_name,
                          Eigen::Matrix<double,6,1>& acceleration) const;

    /**
    * @brief Gets the acceleration of a given point which is solidal with the given link.
    *
    * @param link_name The link which point is attached to.
    * @param point A point in link frame coordinates.
    * @param acceleration The acceleration of point.
    * @return bool True if link_name is a valid link name.
    */
    bool getPointAcceleration(const std::string& link_name,
                              const Eigen::Vector3d& point,
                              Eigen::Vector3d& acceleration) const;

    /*
    * @brief Computed the product of the link_name jacobian time derivative by the joint velocity
    * vector. The reference point of the jacobian is provided as the argument "point".
    *
    * @param link_name The link name
    * @param point A point in link_name coordinates-
    * @param jdotqdot The resulting acceleration twist.
    * @return True if link_name is a valid link name.
    */
    bool computeJdotQdot(const std::string& link_name,
                         const Eigen::Vector3d& point,
                         Eigen::Matrix<double,6,1>& jdotqdot) const;

    /**
     * @brief Gets the gravity vector expressed in the world frame
     *
     * @param gravity The gravity vector expressed in the world frame.
     * @return void
     */
    void getGravity( Eigen::Vector3d& gravity ) const;

    /**
     * @brief  Gets the gravity vector expressed in the reference_frame
     *
     * @param reference_frame The link name w.r.t. which the gravity vector will be expressed
     * @param gravity The gravity vector expressed in the world frame
     * @return True if the reference_frame is a valid link name. False otherwise.
     */
    bool getGravity( const std::string& reference_frame,
                     Eigen::Vector3d& gravity ) const;

    /**
     * @brief Sets the gravity vector expressed in the world frame
     *
     * @param gravity The gravity vector expressed in the world frame.
     * @return void
     */
    void setGravity( const Eigen::Vector3d& gravity );

    /**
     * @brief  Sets the gravity vector expressed in the reference_frame
     *
     * @param reference_frame The link name w.r.t. which the gravity vector will be expressed
     * @param gravity The gravity vector expressed in the reference_frame
     * @return True if the reference_frame is a valid link name. False otherwise.
     */
    bool setGravity( const std::string& reference_frame,
                     const Eigen::Vector3d& gravity );

    /**
     * @brief Applies a mask corresponding to the specified chain to the jacobian provided as an argument.
     * This amounts to setting the columns of the jacobian corresponding the the chain to zero.
     *
     * @param chain_name The name of the chain.
     * @param J The jacobian which the mask has to be applied to.
     * @return True if chain_name is a valid chain name and J has the correct row size.
     */
    bool maskJacobian( const std::string& chain_name, Eigen::MatrixXd& J) const;

    /**
     * @brief getLinkID return the link ID
     * @param link_name the name fo the link
     * @return ID of the link, -1 if the link does not exist
     */
    virtual int getLinkID(const std::string& link_name) const = 0;


    // Getters for RX

    using XBotInterface::getJointPosition;
    using XBotInterface::getMotorPosition;
    using XBotInterface::getJointVelocity;
    using XBotInterface::getMotorVelocity;
    using XBotInterface::getJointAcceleration;
    using XBotInterface::getJointEffort;
    using XBotInterface::getStiffness;
    using XBotInterface::getDamping;


    // Setters for RX

    using XBotInterface::setJointPosition;
    using XBotInterface::setMotorPosition;
    using XBotInterface::setJointVelocity;
    using XBotInterface::setMotorVelocity;
    using XBotInterface::setJointAcceleration;
    using XBotInterface::setJointEffort;


protected:

    virtual bool init_internal(const std::string &path_to_cfg, AnyMapConstPtr any_map);
    virtual bool init_model(const std::string &path_to_cfg) = 0;

    inline void rotationEigenToKDL(const Eigen::Matrix3d& eigen_rotation, KDL::Rotation& kdl_rotation) const;
    inline void rotationKDLToEigen(const KDL::Rotation& kdl_rotation, Eigen::Matrix3d& eigen_rotation) const;


private:


    using XBotInterface::_chain_map;
    using XBotInterface::_ordered_joint_vector;
    std::map<std::string, XBot::ModelChain::Ptr> _model_chain_map;
    XBot::ModelChain _dummy_chain;

    static shlibpp::SharedLibraryClassFactory<ModelInterface> _model_interface_factory;
    static std::vector<std::shared_ptr<shlibpp::SharedLibraryClass<ModelInterface> > > _model_interface_instance;

    using XBotInterface::_ordered_chain_names;
    std::map<int, int> _joint_id_to_model_id;

    static bool parseYAML(const std::string &path_to_cfg, std::map<std::string, std::string>& vars);

    bool fillModelOrderedChain();

    mutable KDL::Twist _tmp_kdl_twist;
    mutable KDL::Frame _tmp_kdl_frame, _tmp_kdl_frame_1;
    mutable KDL::Jacobian _tmp_kdl_jacobian, _tmp_kdl_jacobian_1;
    mutable KDL::Vector _tmp_kdl_vector, _tmp_kdl_vector_1;
    mutable KDL::Rotation _tmp_kdl_rotation, _tmp_kdl_rotation_1;
    mutable std::vector<int> _tmp_int_vector;

    using XBotInterface::getTemperature;
    using XBotInterface::setTemperature;

    // Getters for TX

    using XBotInterface::getPositionReference;
    using XBotInterface::getVelocityReference;
    using XBotInterface::getEffortReference;


    // Setters for TX

    using XBotInterface::setPositionReference;
    using XBotInterface::setVelocityReference;
    using XBotInterface::setEffortReference;
    using XBotInterface::setStiffness;
    using XBotInterface::setDamping;

    using XBotInterface::getChainMap;

    using XBotInterface::sync;

    void seekAndDestroyFixedControlledJoints();

};
};


// NOTE we should put all the utils function in an hpp file with a proper namespace.


inline void XBot::ModelInterface::rotationEigenToKDL(const Eigen::Matrix3d& eigen_rotation, KDL::Rotation& kdl_rotation) const
{
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            kdl_rotation.data[3*i+j] = eigen_rotation(i,j);
        }
    }
}

inline void XBot::ModelInterface::rotationKDLToEigen(const KDL::Rotation& kdl_rotation, Eigen::Matrix3d& eigen_rotation) const
{
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            eigen_rotation(i,j) = kdl_rotation.data[3*i+j];
        }
    }
}

template <typename... SyncFlags>
bool XBot::ModelInterface::syncFrom(const XBot::XBotInterface& other, SyncFlags... flags)
{
    if( XBot::XBotInterface::sync(other, flags...) ){
        update(true, true, true);
        return true;
    }
    else{
        return false;
    }
}

template <typename... SyncFlags>
bool XBot::ModelInterface::syncFrom_no_update(const XBot::XBotInterface& other, SyncFlags... flags)
{
    if( XBot::XBotInterface::sync(other, flags...) ){
        update(true, true, true);
        return true;
    }
    else{
        return false;
    }
}


#endif // __MODEL_INTERFACE_H__
