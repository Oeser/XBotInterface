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
#include <iostream>

#include <eigen3/Eigen/Dense>

#include<XBotInterface/Joint.h>
#include<XBotCoreModel.h>

namespace XBot
{
// TBD update liburdf-headers-dev to version >0.4
typedef boost::shared_ptr<urdf::Joint const> JointConstSharedPtr;
typedef boost::shared_ptr<urdf::Link const> LinkConstSharedPtr;

class KinematicChain
{

public:

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

    typedef std::shared_ptr<KinematicChain> Ptr;

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
     * @brief Method returning the ID of the i-th joint of the chain
     *
     * @param i The position of the joint whose ID is queried (i = 0 refers to the joint attached to the base link)
     *
     * @return The ID of the joint n° i
     */
    int jointId(int i) const;

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
    const std::vector< XBot::JointConstSharedPtr > &getJoints() const;

    /**
     * @brief Method returning the vector of XBot::Links corresponding to the chain.
     *
     * @return const std::vector< XBot::LinkConstSharedPtr>&
     */
    const std::vector< XBot::LinkConstSharedPtr > &getLinks() const;


    bool getLinkPos(Eigen::VectorXd &q) const;
    bool getMotorPos(Eigen::VectorXd &q) const;
    bool getLinkVel(Eigen::VectorXd &qdot) const;
    bool getMotorVel(Eigen::VectorXd &qdot) const;
    bool getEffort(Eigen::VectorXd &tau) const;
    bool getTemperature(Eigen::VectorXd &temp) const;

    bool getLinkPos(std::map<int, double> &q) const;
    bool getMotorPos(std::map<int, double> &q) const;
    bool getLinkVel(std::map<int, double> &qdot) const;
    bool getMotorVel(std::map<int, double> &qdot) const;
    bool getEffort(std::map<int, double> &tau) const;
    bool getTemperature(std::map<int, double> &temp) const;

    bool getLinkPos(std::map<std::string, double> &q) const;
    bool getMotorPos(std::map<std::string, double> &q) const;
    bool getLinkVel(std::map<std::string, double> &qdot) const;
    bool getMotorVel(std::map<std::string, double> &qdot) const;
    bool getEffort(std::map<std::string, double> &tau) const;
    bool getTemperature(std::map<std::string, double> &temp) const;

    double getLinkPos(int index) const;
    double getMotorPos(int index) const;
    double getLinkVel(int index) const;
    double getMotorVel(int index) const;
    double getEffort(int index) const;
    double getTemperature(int index) const;



    bool getPosRef(Eigen::VectorXd &q) const;
    bool getVelRef(Eigen::VectorXd &qdot) const;
    bool getEffortRef(Eigen::VectorXd &tau) const;
    bool getStiffness(Eigen::VectorXd &K) const;
    bool getDamping(Eigen::VectorXd &D) const;

    bool getPosRef(std::map<int, double> &q) const;
    bool getVelRef(std::map<int, double> &qdot) const;
    bool getEffortRef(std::map<int, double> &tau) const;
    bool getStiffness(std::map<int, double> &K) const;
    bool getDamping(std::map<int, double> &D) const;

    bool getPosRef(std::map<std::string, double> &q) const;
    bool getVelRef(std::map<std::string, double> &qdot) const;
    bool getEffortRef(std::map<std::string, double> &tau) const;
    bool getStiffness(std::map<std::string, double> &K) const;
    bool getDamping(std::map<std::string, double> &D) const;

    double getPosRef(int index) const;
    double getVelRef(int index) const;
    double getEffortRef(int index) const;
    double getStiffness(int index) const;
    double getDamping(int index) const;



    bool setLinkPos(const Eigen::VectorXd &q);
    bool setMotorPos(const Eigen::VectorXd &q);
    bool setLinkVel(const Eigen::VectorXd &qdot);
    bool setMotorVel(const Eigen::VectorXd &qdot);
    bool setEffort(const Eigen::VectorXd &tau);
    bool setTemperature(const Eigen::VectorXd &temp);

    bool setLinkPos(const std::map<int, double> &q);
    bool setMotorPos(const std::map<int, double> &q);
    bool setLinkVel(const std::map<int, double> &qdot);
    bool setMotorVel(const std::map<int, double> &qdot);
    bool setEffort(const std::map<int, double> &tau);
    bool setTemperature(const std::map<int, double> &temp);

    bool setLinkPos(const std::map<std::string, double> &q);
    bool setMotorPos(const std::map<std::string, double> &q);
    bool setLinkVel(const std::map<std::string, double> &qdot);
    bool setMotorVel(const std::map<std::string, double> &qdot);
    bool setEffort(const std::map<std::string, double> &tau);
    bool setTemperature(const std::map<std::string, double> &temp);

    bool setLinkPos(int i, double q);
    bool setMotorPos(int i, double q);
    bool setLinkVel(int i, double qdot);
    bool setMotorVel(int i, double qdot);
    bool setEffort(int i, double tau);
    bool setTemperature(int i, double temp);



    bool setPosRef(const Eigen::VectorXd &q);
    bool setVelRef(const Eigen::VectorXd &qdot);
    bool setEffortRef(const Eigen::VectorXd &tau);
    bool setStiffness(const Eigen::VectorXd &K);
    bool setDamping(const Eigen::VectorXd &D);

    bool setPosRef(const std::map<int, double> &q);
    bool setVelRef(const std::map<int, double> &qdot);
    bool setEffortRef(const std::map<int, double> &tau);
    bool setStiffness(const std::map<int, double> &K);
    bool setDamping(const std::map<int, double> &D);

    bool setPosRef(const std::map<std::string, double> &q);
    bool setVelRef(const std::map<std::string, double> &qdot);
    bool setEffortRef(const std::map<std::string, double> &tau);
    bool setStiffness(const std::map<std::string, double> &K);
    bool setDamping(const std::map<std::string, double> &D);

    bool setPosRef(int i, double q);
    bool setVelRef(int i, double qdot);
    bool setEffortRef(int i, double tau);
    bool setStiffness(int i, double K);
    bool setDamping(int i, double D);

    bool sync(const KinematicChain &other);
    friend std::ostream& operator<<(std::ostream& os, const XBot::KinematicChain& c);

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

    std::ostream& operator<<(std::ostream& os, const XBot::KinematicChain& c);
}

#endif // __KINEMATIC_CHAIN_H__
