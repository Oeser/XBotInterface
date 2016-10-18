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

#ifndef __MODEL_INTERFACE_H__
#define __MODEL_INTERFACE_H__

#include <vector>
#include <map>

#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <eigen_conversions/eigen_kdl.h>

#include <XBotInterface/IXBotInterface.h>

namespace XBot {
class ModelInterface : public IXBotInterface {

public:

    /**
     * @brief Updates the kinematic variables of the model according to the current state of the model.
     * 
     * @param update_position True if you want to update the positions. False otherwise.
     * @param update_velocity True if you want to update the velocities. False otherwise.
     * @param update_desired_acceleration True if you want to update the desired accelerations. False otherwise.
     * @return True if the update is successful. False otherwise.
     */
    virtual bool update( bool update_position = true, 
                         bool update_velocity = false,
                         bool update_desired_acceleration = false ) = 0;
     
    /**
    * @brief Computes the pose of the target_frame w.r.t. the source_frame
    * 
    * @param source_frame The source link name. If you want it w.r.t. the world frame, pass "world"
    * @param target_frame The target link name. If you want it w.r.t. the world frame, pass "world"
    * @param pose A homogeneous transformation which transforms a point from source frame to target frame
    * @return True if both source_frame and target_frame are valid. False otherwise.
    */
    virtual bool getPose( const std::string& source_frame,
                          const std::string& target_frame,
                          KDL::Frame& pose ) const = 0;
    /**
    * @brief Computes the pose of the source_frame w.r.t. the world frame
    * 
    * @param source_frame The source link name.
    * @param pose A homogeneous transformation which transforms a point from source frame to world frame
    * @return True if source_frame is valid. False otherwise.
    */
    virtual bool getPose( const std::string& source_frame,
                          KDL::Frame& pose ) = 0;
                          
    /**
    * @brief Sets the floating base pose w.r.t. the world frame
    * 
    * @param floating_base_pose A homogeneous transformation which transforms a point from floating base frame to world frame
    * @return True if the floating_base_pose frame is valid. False otherwise.
    */
    virtual bool setFloatingBasePose( const KDL::Frame& floating_base_pose ) = 0;
    
    
    /**
     * @brief Gets the COM position vector w.r.t. the world frame
     * 
     * @param com_position the Center Of Mass position vector w.r.t. the world frame
     * @return void
     */
    virtual void getCOM( KDL::Vector& com_position ) const = 0;
    
    
    /**
     * @brief Gets the COM Jacobian expressed in the world frame
     * 
     * @param J the COM Jacobian expressed in the world frame
     * @return void
     */
    virtual void getCOMJacobian( KDL::Jacobian& J) const = 0;   
    
    
    /**
     * @brief Gets the COM velocity expressed in the world frame
     * 
     * @param velocity the COM velocity expressed in the world frame
     * @return void
     */
    virtual void getCOMVelocity( KDL::Vector& velocity) const = 0;
    
    
    /**
     * @brief Gets the COM acceleration expressed in the world frame
     * 
     * @param acceleration the COM acceleration expressed in the world frame
     * @return void
     */
    virtual void getCOMAcceleration( KDL::Vector& acceleration) const = 0;
    
    
    /**
     * @brief Gets the gravity vector expressed in the world frame
     * 
     * @param gravity the gravity vector expressed in the world frame
     * @return void
     */
    virtual void getGravity( KDL::Vector& gravity ) const = 0;
    

                             
    /**
     * @brief Sets the gravity vector expressed in the world frame
     * 
     * @param gravity the gravity vector expressed in the world frame
     * @return void
     */
    virtual void setGravity( const KDL::Vector& gravity ) = 0;
       
                              
    /**
     * @brief Gets the Jacobian of link_name expressed in the world frame. The reference point of the Jacobian is the reference_point arg.
     * 
     * @param link_name the link name
     * @param reference_point the reference point w.r.t. which the jacobian is computed
     * @param J  the Jacobian expressed in the world frame
     * @return True if the link_name is a valid link name. False otherwise.
     */
    virtual bool getPointJacobian( const std::string& link_name, 
                                   const KDL::Vector& reference_point, 
                                   KDL::Jacobian& J) const = 0;
                                   
    
    /**
    * @brief Gets the spatial velocity of the link with link_name name. The reference point of the velocity twist is the origin of the link with link_name name.
    * 
    * @param link_name the link name
    * @param velocity the spatial velocity of the link with link_name name
    * @return True if the link_name is a valid link name. False otherwise.
    */
    virtual bool getSpatialVelocity( const std::string& link_name, 
                                     KDL::Twist& velocity) const = 0;
                                     
    /**
    * @brief Gets the spatial acceleration of the link with link_name name. The reference point of the acceleration twist is the origin of the link with link_name name.
    * 
    * @param link_name the link name
    * @param velocity the spatial velocity of the link with link_name name
    * @return True if the link_name is a valid link name. False otherwise.
    */                                 
    virtual bool getSpatialAcceleration( const std::string& link_name, 
                                         KDL::Twist& acceleration) const = 0;

    
    
    /**
    * @brief Gets the model ID of the joint with joint_name name
    * 
    * @param joint_name the joint name
    * @return int the model ID of the joint with joint_name name
    */
    virtual int getModelID( const std::string& joint_name) const = 0;
    
    /**
     * @brief Gets the vector of model IDs of the joints inside chain with name chain_name
     * 
     * @param chain_name the chain name
     * @param model_id_vector the vector of model IDs of the joints inside chain with name chain_name
     * @return True is a chain with chain_name name exists.
     */
    virtual bool getModelID( const std::string& chain_name,
                             std::vector<int>& model_id_vector) const = 0;
                             
                             
                             
                             
   
    bool getSpatialVelocity( const std::string& link_name, 
                             Eigen::Matrix<double,6,1>& velocity) const;
    bool getAcceleration( const std::string& link_name, 
                          Eigen::Matrix<double,6,1>& acceleration) const;
    
    /**
     * @brief Gets the Jacobian of link_name expressed in the world frame. The reference point is the origin of the link with link_name name.
     * 
     * @param link_name the link name
     * @param J  the Jacobian expressed in the world frame
     * @return True if the link_name is a valid link name. False otherwise.
     */
    bool getJacobian( const std::string& link_name, 
                      KDL::Jacobian& J) const;
    bool getJacobian( const std::string& link_name, 
                      Eigen::MatrixXd& J);
    
    bool getPointJacobian( const std::string& link_name, 
                           const Eigen::Vector3d& point, 
                           Eigen::MatrixXd& J);
    
    /**
     * @brief  Gets the gravity vector expressed in the reference_frame
     * 
     * @param reference_frame The link name w.r.t. which the gravity vector will be expressed
     * @param gravity the gravity vector expressed in the world frame
     * @return True if the reference_frame is a valid link name. False otherwise.
     */
    bool getGravity( const std::string& reference_frame, 
                     KDL::Vector& gravity ) const;
    void getGravity( Eigen::Vector3d& gravity ) const;
    bool getGravity( const std::string& reference_frame, 
                     Eigen::Vector3d& gravity ) const;
                     /**
                      * @brief ...
                      * 
                      * @param reference_frame ...
                      * @param gravity ...
                      * @return bool
                      */
    
    /**
     * @brief  Sets the gravity vector expressed in the reference_frame
     * 
     * @param reference_frame The link name w.r.t. which the gravity vector will be expressed
     * @param gravity the gravity vector expressed in the reference_frame
     * @return True if the reference_frame is a valid link name. False otherwise.
     */                     
    bool setGravity( const std::string& reference_frame, 
                     const KDL::Vector& gravity ); 
    void setGravity( const Eigen::Vector3d& gravity );
    bool setGravity( const std::string& reference_frame, 
                     const Eigen::Vector3d& gravity );
                 
    /**
     * @brief Gets the COM position vector w.r.t. the reference_frame
     * 
     * @param reference_frame The link name w.r.t. which the COM position will be expressed
     * @param com_position the Center Of Mass position vector w.r.t. the reference_frame
     * @return True if the reference_frame is a valid link name. False otherwise.
     */
    bool getCOM( const std::string& reference_frame, 
                         KDL::Vector& com_position ) const;
                         
    void getCOM( Eigen::Vector3d& com_position ) const;
    bool getCOM( const std::string& reference_frame, 
                 Eigen::Vector3d& com_position ) const;
                 
    
    bool setFloatingBasePose( const Eigen::Affine3d& floating_base_pose );
                                                    
    bool getPose(   const std::string& source_frame,
                    const std::string& target_frame,
                    Eigen::Affine3d& pose ) const;
                          
    bool getPose( const std::string& source_frame,
                  Eigen::Affine3d& pose ) const;
                  
    
                                         
    void getCOMJacobian( Eigen::MatrixXd& J) const;                         
    void getCOMVelocity( Eigen::Vector3d& velocity) const;
    void getCOMAcceleration( Eigen::Vector3d& acceleration) const;                  
                          
                          
                          
    bool getOrientation(    const std::string& target_frame,
                            KDL::Rotation& target_point) const;                           
    bool getOrientation(    const std::string& target_frame,
                            Eigen::Matrix3d& target_point) const; 
                            
    bool getOrientation(    const std::string& source_frame,
                            const std::string& target_frame,
                            KDL::Rotation& target_point) const;                           
    bool getOrientation(    const std::string& source_frame,
                            const std::string& target_frame,
                            Eigen::Matrix3d& target_point) const;                       
                          
    bool getPointPosition(  const std::string& target_frame,
                            const KDL::Vector& source_point,
                            KDL::Vector& target_point) const;                           
    bool getPointPosition(  const std::string& target_frame,
                            const Eigen::Vector3d& source_point,
                            Eigen::Vector3d& target_point) const; 
    bool getPointPosition(  const std::string& source_frame,
                            const std::string& target_frame,
                            const KDL::Vector& source_point,
                            KDL::Vector& target_point) const;                           
    bool getPointPosition(  const std::string& source_frame,
                            const std::string& target_frame,
                            const Eigen::Vector3d& source_point,
                            Eigen::Vector3d& target_point) const; 



protected:

private:

};
};

#endif // __MODEL_INTERFACE_H__
