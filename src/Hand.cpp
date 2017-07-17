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

#include <XBotInterface/Hand.h>

namespace XBot {

    
Hand::Hand ( int hand_id, std::string hand_name ) : _hand_id(hand_id), _hand_name(hand_name), _grasp_percentage(0)
{

}

int Hand::getHandId()
{
    return _hand_id;
}


const std::string& Hand::getHandName()
{
    return _hand_name;
}


void Hand::grasp ( double grasp_percentage )
{
    _grasp_percentage = grasp_percentage;
}

double XBot::Hand::getGraspReference()
{
    return _grasp_percentage;
}


double Hand::getGraspState() const
{
    return _grasp_state;
}

void Hand::setGraspState ( double grasp_state )
{
    _grasp_state = grasp_state;
}



}