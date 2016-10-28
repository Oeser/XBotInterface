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

#include "XBotInterface/RobotChain.h"

namespace XBot {
    
RobotChain::RobotChain()
{

}

bool RobotChain::setReferenceFrom(const ModelChain& model_chain)
{
    // TBD: check that chains are indeed omologues??
    int pos = 0;
    for (const XBot::Joint::Ptr & j : model_chain._joint_vector) {
        _joint_vector[pos++]->setReferenceFrom(*j);
    }    
}

double RobotChain::getDamping ( int index ) const
{
     return XBot::KinematicChain::getDamping ( index );
}

bool RobotChain::getDamping ( std::map< std::string, double >& D ) const
{
     return XBot::KinematicChain::getDamping ( D );
}

bool RobotChain::getDamping ( std::map< int, double >& D ) const
{
     return XBot::KinematicChain::getDamping ( D );
}

bool RobotChain::getDamping ( Eigen::VectorXd& D ) const
{
     return XBot::KinematicChain::getDamping ( D );
}

double RobotChain::getEffortReference ( int index ) const
{
     return XBot::KinematicChain::getEffortReference ( index );
}

bool RobotChain::getEffortReference ( std::map< std::string, double >& tau ) const
{
     return XBot::KinematicChain::getEffortReference ( tau );
}

bool RobotChain::getEffortReference ( std::map< int, double >& tau ) const
{
     return XBot::KinematicChain::getEffortReference ( tau );
}

bool RobotChain::getEffortReference ( Eigen::VectorXd& tau ) const
{
     return XBot::KinematicChain::getEffortReference ( tau );
}


double RobotChain::getPositionReference ( int index ) const
{
     return XBot::KinematicChain::getPositionReference ( index );
}

bool RobotChain::getPositionReference ( std::map< std::string, double >& q ) const
{
     return XBot::KinematicChain::getPositionReference ( q );
}


bool RobotChain::getPositionReference ( std::map< int, double >& q ) const
{
     return XBot::KinematicChain::getPositionReference ( q );
}


bool RobotChain::getPositionReference ( Eigen::VectorXd& q ) const
{
     return XBot::KinematicChain::getPositionReference ( q );
}


double RobotChain::getStiffness ( int index ) const
{
     return XBot::KinematicChain::getStiffness ( index );
}


bool RobotChain::getStiffness ( std::map< std::string, double >& K ) const
{
     return XBot::KinematicChain::getStiffness ( K );
}

bool RobotChain::getStiffness ( std::map< int, double >& K ) const
{
     return XBot::KinematicChain::getStiffness ( K );
}

bool RobotChain::getStiffness ( Eigen::VectorXd& K ) const
{
     return XBot::KinematicChain::getStiffness ( K );
}

double RobotChain::getVelocityReference ( int index ) const
{
     return XBot::KinematicChain::getVelocityReference ( index );
}

bool RobotChain::getVelocityReference ( std::map< std::string, double >& qdot ) const
{
     return XBot::KinematicChain::getVelocityReference ( qdot );
}

bool RobotChain::getVelocityReference ( std::map< int, double >& qdot ) const
{
     return XBot::KinematicChain::getVelocityReference ( qdot );
}

bool RobotChain::getVelocityReference ( Eigen::VectorXd& qdot ) const
{
     return XBot::KinematicChain::getVelocityReference ( qdot );
}

bool RobotChain::setDamping ( int i, double D )
{
     return XBot::KinematicChain::setDamping ( i, D );
}

bool RobotChain::setDamping ( const std::map< std::string, double >& D )
{
     return XBot::KinematicChain::setDamping ( D );
}

bool RobotChain::setDamping ( const std::map< int, double >& D )
{
     return XBot::KinematicChain::setDamping ( D );
}

bool RobotChain::setDamping ( const Eigen::VectorXd& D )
{
     return XBot::KinematicChain::setDamping ( D );
}

bool RobotChain::setEffortReference ( int i, double tau )
{
     return XBot::KinematicChain::setEffortReference ( i, tau );
}

bool RobotChain::setEffortReference ( const std::map< std::string, double >& tau )
{
     return XBot::KinematicChain::setEffortReference ( tau );
}

bool RobotChain::setEffortReference ( const std::map< int, double >& tau )
{
     return XBot::KinematicChain::setEffortReference ( tau );
}

bool RobotChain::setEffortReference ( const Eigen::VectorXd& tau )
{
     return XBot::KinematicChain::setEffortReference ( tau );
}


bool RobotChain::setPositionReference ( int i, double q )
{
     return XBot::KinematicChain::setPositionReference ( i, q );
}

bool RobotChain::setPositionReference ( const std::map< std::string, double >& q )
{
     return XBot::KinematicChain::setPositionReference ( q );
}


bool RobotChain::setPositionReference ( const std::map< int, double >& q )
{
     return XBot::KinematicChain::setPositionReference ( q );
}

bool RobotChain::setPositionReference ( const Eigen::VectorXd& q )
{
     return XBot::KinematicChain::setPositionReference ( q );
}

bool RobotChain::setStiffness ( int i, double K )
{
     return XBot::KinematicChain::setStiffness ( i, K );
}

bool RobotChain::setStiffness ( const std::map< std::string, double >& K )
{
     return XBot::KinematicChain::setStiffness ( K );
}

bool RobotChain::setStiffness ( const std::map< int, double >& K )
{
     return XBot::KinematicChain::setStiffness ( K );
}

bool RobotChain::setStiffness ( const Eigen::VectorXd& K )
{
     return XBot::KinematicChain::setStiffness ( K );
}

bool RobotChain::setVelocityReference ( int i, double qdot )
{
     return XBot::KinematicChain::setVelocityReference ( i, qdot );
}

bool RobotChain::setVelocityReference ( const std::map< std::string, double >& qdot )
{
     return XBot::KinematicChain::setVelocityReference ( qdot );
}

bool RobotChain::setVelocityReference ( const std::map< int, double >& qdot )
{
     return XBot::KinematicChain::setVelocityReference ( qdot );
}

bool RobotChain::setVelocityReference ( const Eigen::VectorXd& qdot )
{
     return XBot::KinematicChain::setVelocityReference ( qdot );
}


    
}