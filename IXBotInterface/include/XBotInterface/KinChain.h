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

#ifndef __KIN_CHAIN_H__
#define __KIN_CHAIN_H__

#include <string>
#include <vector>
#include <map>
#include <memory>

#include<XBotInterface/Joint.h>

namespace XBot
{

    class KinChain {
    private:
        
        std::map<std::string, XBot::Joint::Ptr> _joint_name_map;
        std::map<int, XBot::Joint::Ptr> _joint_id_map;
        std::vector<XBot::Joint::Ptr> _joint_vector;
        
        std::string _chain_name;
        int _joint_num;
        
        void initialize_joint_vector();
        
    protected:
        
    public:
        
        typedef std::shared_ptr<KinChain> Ptr;
        
        KinChain();
        KinChain(std::string chain_name, 
                std::map<std::string, int> joint_name_id_map);
        
        int getJointNum();
        
    };
}

#endif // __KIN_CHAIN_H__