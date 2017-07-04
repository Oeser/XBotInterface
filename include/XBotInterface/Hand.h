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

#ifndef __XBOT_HAND_H__
#define __XBOT_HAND_H__

#include <string>
#include <memory>
#include <iostream>


namespace XBot {
    
    /**
     * @brief Hand
     * 
     */
    class Hand {

//     class RobotInterface;
    
public:
    
    typedef std::shared_ptr<Hand> Ptr;
    typedef std::shared_ptr<const Hand> ConstPtr;

//     friend XBot::RobotInterface;
    
    /**
     * @brief Constructor with the hand id
     * 
     * @param hand_id the hand id
     * @param hand_name the hand name
     */
    Hand(int hand_id, std::string hand_name);
    
    /**
     * @brief get the hand id
     * 
     * @return int the hand id
     */
    int getHandId();
    
    /**
     * @brief get the hand name
     * 
     * @return const std::string& the hand name
     */
    const std::string& getHandName();
    
    /**
     * @brief grasp till grasp_percentage
     * 
     * @param grasp_percentage grasp percentage from 0.0 to 1.0 (0% to 100% grasp) 
     */
    void grasp(double grasp_percentage);
    
    /**
     * @brief get the grasp reference set with grasp_percentage
     * 
     * @return double the grasp reference set in previous grasp() function
     */
    double getGraspReference();
    
    /**
     * @brief get actual grasp percentage
     * 
     * @return double the actual grasp percentage
     */
    double getGraspState() const;

    /**
     * @brief set the grasp state read from the hand
     * 
     * @param grasp_state the grasp state to set
     * @return void
     */
    void setGraspState(double grasp_state);

private:
    
    int _hand_id;
    std::string _hand_name;
    
    double _grasp_percentage;
    
    double _grasp_state;
    
    };
}
#endif