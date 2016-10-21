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

#include <XBotInterface/ModelChain.h>

bool XBot::ModelChain::setJointEffort ( int i, double tau )
{
     return XBot::KinematicChain::setJointEffort ( i, tau );
}

bool XBot::ModelChain::setJointEffort ( const std::map< std::string, double >& tau )
{
     return XBot::KinematicChain::setJointEffort ( tau );
}

bool XBot::ModelChain::setJointEffort ( const std::map< int, double >& tau )
{
     return XBot::KinematicChain::setJointEffort ( tau );
}

bool XBot::ModelChain::setJointEffort ( const Eigen::VectorXd& tau )
{
     return XBot::KinematicChain::setJointEffort ( tau );
}

bool XBot::ModelChain::setJointPosition ( int i, double q )
{
     return XBot::KinematicChain::setJointPosition ( i, q );
}

bool XBot::ModelChain::setJointPosition ( const std::map< std::string, double >& q )
{
     return XBot::KinematicChain::setJointPosition ( q );
}

bool XBot::ModelChain::setJointPosition ( const std::map< int, double >& q )
{
     return XBot::KinematicChain::setJointPosition ( q );
}

bool XBot::ModelChain::setJointPosition ( const Eigen::VectorXd& q )
{
     return XBot::KinematicChain::setJointPosition ( q );
}

bool XBot::ModelChain::setJointVelocity ( int i, double qdot )
{
     return XBot::KinematicChain::setJointVelocity ( i, qdot );
}

bool XBot::ModelChain::setJointVelocity ( const std::map< std::string, double >& qdot )
{
     return XBot::KinematicChain::setJointVelocity ( qdot );
}

bool XBot::ModelChain::setJointVelocity ( const std::map< int, double >& qdot )
{
     return XBot::KinematicChain::setJointVelocity ( qdot );
}

bool XBot::ModelChain::setJointVelocity ( const Eigen::VectorXd& qdot )
{
     return XBot::KinematicChain::setJointVelocity ( qdot );
}

bool XBot::ModelChain::setMotorPosition ( int i, double q )
{
     return XBot::KinematicChain::setMotorPosition ( i, q );
}

bool XBot::ModelChain::setMotorPosition ( const std::map< std::string, double >& q )
{
     return XBot::KinematicChain::setMotorPosition ( q );
}

bool XBot::ModelChain::setMotorPosition ( const std::map< int, double >& q )
{
     return XBot::KinematicChain::setMotorPosition ( q );
}

bool XBot::ModelChain::setMotorPosition ( const Eigen::VectorXd& q )
{
     return XBot::KinematicChain::setMotorPosition ( q );
}

bool XBot::ModelChain::setMotorVelocity ( int i, double qdot )
{
     return XBot::KinematicChain::setMotorVelocity ( i, qdot );
}

bool XBot::ModelChain::setMotorVelocity ( const std::map< std::string, double >& qdot )
{
     return XBot::KinematicChain::setMotorVelocity ( qdot );
}

bool XBot::ModelChain::setMotorVelocity ( const std::map< int, double >& qdot )
{
     return XBot::KinematicChain::setMotorVelocity ( qdot );
}

bool XBot::ModelChain::setMotorVelocity ( const Eigen::VectorXd& qdot )
{
     return XBot::KinematicChain::setMotorVelocity ( qdot );
}

bool XBot::ModelChain::setTemperature ( int i, double temp )
{
     return XBot::KinematicChain::setTemperature ( i, temp );
}

bool XBot::ModelChain::setTemperature ( const std::map< std::string, double >& temp )
{
     return XBot::KinematicChain::setTemperature ( temp );
}

bool XBot::ModelChain::setTemperature ( const std::map< int, double >& temp )
{
     return XBot::KinematicChain::setTemperature ( temp );
}

bool XBot::ModelChain::setTemperature ( const Eigen::VectorXd& temp )
{
     return XBot::KinematicChain::setTemperature ( temp );
}

bool XBot::ModelChain::syncFrom ( const XBot::KinematicChain& other )
{
     return XBot::KinematicChain::syncFrom ( other );
}
