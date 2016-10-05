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

#ifndef __JOINT_H__
#define __JOINT_H__

#include <string>
#include <memory>

namespace XBot
{

    /**
    * @brief Container for the Joint information.
    * 
    */
    class Joint {
    private:
        
        std::string _joint_name;
        int _joint_id;
        
        double _link_pos;
        double _motor_pos;
        double _link_vel;
        double _motor_vel;
        double _effort;
        double _temperature;
        
    protected:
        
        void setJointName(std::string joint_name);
        void setJointId(int joint_id);
        
        void setLinkPos(double link_pos);
        void setMotorPos(double motor_pos);
        void setLinkVel(double link_vel);
        void setMotorVel(double motor_vel);
        void setEffort(double effort);
        void setTemperature(double temperature);
        
    public:
        
        typedef std::shared_ptr<Joint> Ptr;

        std::string getJointName();
        int getJointId();
        
        double getLinkPos();
        double getMotorPos();
        double getLinkVel();
        double getMotorVel();
        double getEffort();
        double getTemperature();

        
        Joint();
        Joint(std::string joint_name, int joint_id); 
        
        
    };
}

#endif // __JOINT_H__