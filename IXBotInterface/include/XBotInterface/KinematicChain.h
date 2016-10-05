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

#include<XBotInterface/Joint.h>
#include<XBotCoreModel.h>

namespace XBot
{

    class KinematicChain {
    private:
        
        std::map<std::string, XBot::Joint::Ptr> _joint_name_map;
        std::map<int, XBot::Joint::Ptr> _joint_id_map;
        std::vector<XBot::Joint::Ptr> _joint_vector;
        
        std::vector<std::string> _ordered_joint_name;
        std::vector<int> _ordered_joint_id;
        
        XBot::XBotCoreModel _XBotModel;
        
        std::string _chain_name;
        int _joint_num;
        
    protected:
        
    public:
        
        typedef std::shared_ptr<KinematicChain> Ptr;
        
        KinematicChain();
        KinematicChain( const std::string& chain_name, 
                        const XBot::XBotCoreModel& XBotModel);
        // TBD copy constructor and =
        
        int getJointNum();
        
    };
}

#endif // __KIN_CHAIN_H__