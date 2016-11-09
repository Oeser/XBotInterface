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
        _joint_vector[pos++]->setReferenceFrom(*j);  // TBD check the return
    }    
    return true; // TBD check the above TBD
}



    
}