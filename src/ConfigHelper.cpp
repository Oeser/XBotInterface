/*
 * Copyright (C) 2017 IIT-ADVR
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

#include <XBotInterface/ConfigHelper.h>

namespace XBot {

ConfigHelper::ConfigHelper(std::string path_to_config_file):
        console(*XBot::ConsoleLogger::getLogger())
    {
        if(!(path_to_config_file.at(0) == '/')) {

            // Input path does not begin with a /, so is a relative path
            const char * env_p = std::getenv("ROBOTOLOGY_ROOT");

            // check the env, otherwise error
            if(env_p) {
                std::string current_path(env_p);
                // default relative path when working with the superbuild
                path_to_config_file = current_path + "/" + path_to_config_file;
            }
            else {
                 console.error() << "in " << __PRETTY_FUNCTION__ << " : the input path  " << path_to_config_file <<
                 " is neither an absolute path nor related with the ROBOTOLOGY SUPERBUILD. Download it!" << console.endl();
                return;
            }
        }

        std::ifstream fin(path_to_config_file);
        if (fin.fail()) {
            console.error() << "in " << __PRETTY_FUNCTION__ << "! Can NOT open " << path_to_config_file << "!" << console.endl();
        }

        _current_node = _root_node = YAML::LoadFile(path_to_config_file);

        if(!_root_helper){
            _root_helper.reset(new ConfigHelper(_root_node));
        }

    }

ConfigHelper& ConfigHelper::operator[](const std::string& subnode_name)
    {
        auto it = _instance_map.find(subnode_name);

        if( it == _instance_map.end() ){
            YAML::Node subnode = _current_node[subnode_name];
            if( subnode.IsMap() ){
                _instance_map[subnode_name] = std::shared_ptr<ConfigHelper>(new ConfigHelper(subnode));
                return *_instance_map[subnode_name];
            }
            else{
                console.error() << "in " << __PRETTY_FUNCTION__ << "! Subnode " << subnode_name << " is not a defined inside \
                the node " << _current_node.Tag() << console.endl();
                return *_root_helper;
            }

        }
        else{
            return *(it->second);
        }
    }

std::shared_ptr<ConfigHelper> ConfigHelper::_root_helper;
std::map<std::string, std::shared_ptr<ConfigHelper>> ConfigHelper::_instance_map;

}