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

#ifndef __ROBOT_INTERFACE_H__
#define __ROBOT_INTERFACE_H__

#include <vector>
#include <map>
#include <memory>
#include <SharedLibraryClass.h>

#include <XBotInterface/IXBotInterface.h>

namespace XBot
{

    class RobotInterface : public IXBotInterface {

    public:
		
        RobotInterface();

        typedef std::shared_ptr<RobotInterface> Ptr;

        static RobotInterface::Ptr getRobot(const std::string& path_to_cfg, int argc, char **argv);

        bool sense( bool sync_model = true );
        bool move( bool sync_model = true );
        
        virtual bool setControlMode(const std::map<std::string, std::string>& joint_control_mode_map) = 0;
        virtual bool setControlMode(const std::string& robot_control_mode) = 0;
        virtual bool getControlMode(std::map<std::string, std::string>& joint_control_mode_map) = 0;

        
    protected:
      
        
        virtual bool sense_internal() = 0;
        virtual bool move_internal() = 0;
		virtual bool init_internal(const std::string& path_to_cfg) = 0;
		

    private:

        IXBotInterface::Ptr model; // TBD it is going to be the ModelInterface inside the RobotInterface
        
        static RobotInterface::Ptr _instance_ptr;
		static shlibpp::SharedLibraryClass<RobotInterface> _robot_interface_instance;
		
    };
}

#endif // __ROBOT_INTERFACE_H__