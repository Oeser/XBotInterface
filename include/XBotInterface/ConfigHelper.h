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

#ifndef __XBOT_CONFIG_HELPER_H__
#define __XBOT_CONFIG_HELPER_H__

#include <yaml-cpp/yaml.h>
#include <memory>
#include <XBotInterface/Logger.hpp>
#include <fstream>


namespace XBot {


class ConfigHelper {

public:

    ConfigHelper(std::string path_to_config_file);

    ConfigHelper& operator[](const std::string& subnode_name);

    template <typename ParameterType>
    bool get(const std::string& parameter_name, ParameterType& parameter_value) const
    {
        YAML::Node param_node = _current_node[parameter_name];
        if(!param_node){
            console.error() << "in " << __PRETTY_FUNCTION__ << "! Parameter " << parameter_name << " is not a defined inside \
            the node " << _current_node.Tag() << console.endl();
            return false;
        }

        parameter_value = param_node.as<ParameterType>();
    }



private:

    ConfigHelper(const YAML::Node& node):
        _current_node(node), console(*XBot::ConsoleLogger::getLogger())
    {

    }

    YAML::Node _current_node, _root_node;
    ConsoleLogger& console;

    static std::shared_ptr<ConfigHelper> _root_helper;
    static std::map<std::string, std::shared_ptr<ConfigHelper>> _instance_map;


};



}

#endif