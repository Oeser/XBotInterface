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

#include <XBotInterface/RobotInterface.h>
#include <ros/ros.h> //TBD: remove include

// NOTE Static members need to be defined in the cpp
std::string XBot::RobotInterface::_framework;
std::string XBot::RobotInterface::_subclass_name;
std::string XBot::RobotInterface::_path_to_shared_lib;
std::string XBot::RobotInterface::_subclass_factory_name;
XBot::RobotInterface::Ptr XBot::RobotInterface::_instance_ptr;
XBot::ModelInterface::Ptr XBot::RobotInterface::_model;
shlibpp::SharedLibraryClass<XBot::RobotInterface> XBot::RobotInterface::_robot_interface_instance;
shlibpp::SharedLibraryClassFactory<XBot::RobotInterface> XBot::RobotInterface::_robot_interface_factory;

XBot::RobotInterface::RobotInterface()
{
}

bool XBot::RobotInterface::parseYAML(const std::string &path_to_cfg, const std::string& framework)
{
    std::ifstream fin(path_to_cfg);
    if (fin.fail()) {
        std::cerr << "ERROR in " << __func__ << "! Can NOT open " << path_to_cfg << "!" << std::endl;
        return false;
    }

    // loading YAML
    YAML::Node root_cfg = YAML::LoadFile(path_to_cfg);
    YAML::Node x_bot_interface;
    // XBotInterface info
    if(root_cfg["RobotInterface"]) {
        x_bot_interface = root_cfg["RobotInterface"];
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : YAML file  " << path_to_cfg << "  does not contain RobotInterface mandatory node!!" << std::endl;
        return false;
    }
    // check framework
    if(x_bot_interface["framework_name"]) {
        _framework = x_bot_interface["framework_name"].as<std::string>();
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : RobotInterface node of  " << path_to_cfg << "  does not contain framework_name mandatory node!!" << std::endl;
        return false;
    }

    if( framework != "" ){
        _framework = framework;
    }

    // subclass forced
    _subclass_name = std::string("RobotInterface") + _framework;
    // check the path to shared lib
    if(root_cfg[_subclass_name]["path_to_shared_lib"]) {
        computeAbsolutePath(root_cfg[_subclass_name]["path_to_shared_lib"].as<std::string>(),
                            LIB_MIDDLE_PATH,
                            _path_to_shared_lib);
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : YAML file  " << path_to_cfg << "  does not contain " << _subclass_name << " mandatory node!!" << std::endl;
        return false;
    }

    if(root_cfg[_subclass_name]["subclass_factory_name"]) {
        _subclass_factory_name = root_cfg[_subclass_name]["subclass_factory_name"].as<std::string>();
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : " << _subclass_name << " node of  " << path_to_cfg << "  does not contain subclass_factory_name mandatory node!!" << std::endl;
        return false;
    }
    return true;

}


XBot::RobotInterface::Ptr XBot::RobotInterface::getRobot(const std::string &path_to_cfg,
                                        AnyMapConstPtr any_map,
                                        const std::string& framework)
{
    // NOTE singleton
    if (_instance_ptr) {
        return _instance_ptr;
    }
    // parsing YAML
    if (!parseYAML(path_to_cfg, framework)) {
        std::cerr << "ERROR in " << __func__ << " : could not parse the YAML " << path_to_cfg << " . See error above!!" << std::endl;
        return _instance_ptr;
    }


    // loading the requested model interface internal to the robot
    _model = XBot::ModelInterface::getModel(path_to_cfg);


    // loading the requested robot interface
    _robot_interface_factory.open( _path_to_shared_lib.c_str(),
                                                                                _subclass_factory_name.c_str());
    if (!_robot_interface_factory.isValid()) {
        // NOTE print to celebrate the wizard
        printf("error (%s) : %s\n", shlibpp::Vocab::decode(_robot_interface_factory.getStatus()).c_str(),
               _robot_interface_factory.getLastNativeError().c_str());
    }
    // open and init robot interface
    _robot_interface_instance.open(_robot_interface_factory);
    _robot_interface_instance->init(path_to_cfg, any_map);
    // static instance of the robot interface
    _instance_ptr = std::shared_ptr<RobotInterface>(&_robot_interface_instance.getContent(), [](RobotInterface* ptr){return;});



    return _instance_ptr;

}

XBot::ModelInterface& XBot::RobotInterface::model()
{
    return *_model;
}




bool XBot::RobotInterface::sense(bool sync_model)
{
    bool sense_ok = sense_internal();
    bool sense_hands_ok = sense_hands();
    bool sensors_ok = read_sensors();
    _ts_rx = getTime();
    if (sync_model) {
        return  (_model->syncFrom(*this)) && sense_ok && sense_hands_ok && sensors_ok;
    }
    return sense_ok && sense_hands_ok && sensors_ok;
}


bool XBot::RobotInterface::move()
{
    _ts_tx = getTime();
    return move_internal();
}

bool XBot::RobotInterface::init_internal(const std::string& path_to_cfg, AnyMapConstPtr any_map)
{
    // Fill _robot_chain_map with shallow copies of chains in _chain_map

    _robot_chain_map.clear();

    for( const auto& c : _chain_map){

        RobotChain::Ptr robot_chain(new RobotChain());
        robot_chain->shallowCopy(*c.second);

        _robot_chain_map[c.first] = robot_chain;

    }

    // Call virtual init_robot
    bool success = init_robot(path_to_cfg, any_map);

    _ordered_chain_names.clear();
    for( const std::string& s : model().getModelOrderedChainName() ){
            if( s == "virtual_chain" ){}
            else{
                _ordered_chain_names.push_back(s);
            }
    }

    // Since fixed controlled joints do not appear in the model, some chains may
    // be missing. So we add them!
    for( const auto& pair : _chain_map ){
        auto it = std::find(_ordered_chain_names.begin(), _ordered_chain_names.end(), pair.first);
        if( it == _ordered_chain_names.end() ){
            _ordered_chain_names.push_back(pair.first);
        }
    }

    return success;
}

XBot::RobotChain& XBot::RobotInterface::arm(int arm_id)
{
    if (_XBotModel.get_arms_chain().size() > arm_id) {
        const std::string &requested_arm_name = _XBotModel.get_arms_chain().at(arm_id);
        return *_robot_chain_map.at(requested_arm_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a arms with id " << arm_id << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

XBot::RobotChain& XBot::RobotInterface::leg(int leg_id)
{
    if (_XBotModel.get_legs_chain().size() > leg_id) {
        const std::string &requested_leg_name = _XBotModel.get_legs_chain().at(leg_id);
        return *_robot_chain_map.at(requested_leg_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a legs with id " << leg_id << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

XBot::RobotChain& XBot::RobotInterface::operator()(const std::string& chain_name)
{
    if (_robot_chain_map.count(chain_name)) {
        return *_robot_chain_map.at(chain_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a chain with name " << chain_name << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

XBot::RobotChain& XBot::RobotInterface::chain(const std::string& chain_name)
{
    return operator()(chain_name);
}

double XBot::RobotInterface::getTimestampRx() const
{
    return _ts_rx;
}

double XBot::RobotInterface::getTimestampTx() const
{
    return _ts_tx;
}

bool XBot::RobotInterface::set_control_mode_internal(int joint_id, const XBot::ControlMode& control_mode)
{
    return false;
}


void XBot::RobotInterface::getControlMode(std::map< int, XBot::ControlMode >& control_mode) const
{
    ControlMode ctrl;
    for(int i = 0; i < getJointNum(); i++){
        _joint_vector[i]->getControlMode(ctrl);
        control_mode[_joint_vector[i]->getJointId()] = ctrl;
    }
}

void XBot::RobotInterface::getControlMode(std::map< std::string, XBot::ControlMode >& control_mode) const
{
    ControlMode ctrl;
    for(int i = 0; i < getJointNum(); i++){
        _joint_vector[i]->getControlMode(ctrl);
        control_mode[_joint_vector[i]->getJointName()] = ctrl;
    }
}



bool XBot::RobotInterface::setControlMode(const std::map< int, XBot::ControlMode >& control_mode)
{
    bool success = true;

    for( const auto& pair: control_mode ){


        if( hasJoint(pair.first) ){

            bool set_internal_success = set_control_mode_internal(pair.first, pair.second);

            if(set_internal_success){
                getJointByIdInternal(pair.first)->setControlMode(pair.second);
                std::cout << "INFO: Joint " << getJointByIdInternal(pair.first)->getJointName() <<
                             " - with id : " << pair.first << " - control mode changed to " << pair.second.getName() << std::endl;
            }
            else {
                std::cout << "WARNING: Joint " << getJointByIdInternal(pair.first)->getJointName() <<
                             " - with id : " << pair.first << " - CANNOT change control mode to " << pair.second.getName() << std::endl;
            }

            success = success && set_internal_success;
        }

    }

    return success;
}

bool XBot::RobotInterface::setControlMode(const std::map< std::string, XBot::ControlMode >& control_mode)
{
    bool success = true;

    for( const auto& pair: control_mode ){

        if( hasJoint(pair.first) ){

            bool set_internal_success = set_control_mode_internal(getJointByNameInternal(pair.first)->getJointId(), pair.second);

            if(set_internal_success){
                getJointByNameInternal(pair.first)->setControlMode(pair.second);
                std::cout << "INFO: Joint " << pair.first << " - with id : "
                          << getJointByNameInternal(pair.first)->getJointId()
                          << " - control mode changed to " << pair.second.getName() << std::endl;
            }
            else {
                std::cout << "WARNING: Joint " << pair.first  << " - with id : "
                          << getJointByNameInternal(pair.first)->getJointId() << " - CANNOT change control mode to "
                          << pair.second.getName() << std::endl;
            }

            success = set_internal_success && success;
        }

    }

    return success;
}

bool XBot::RobotInterface::setControlMode(const XBot::ControlMode& control_mode)
{
    bool success = true;

    for( int i = 0; i < getJointNum(); i++ ){

        bool set_internal_success = success = set_control_mode_internal(_joint_vector[i]->getJointId(), control_mode);

        if(set_internal_success){
            _joint_vector[i]->setControlMode(control_mode);
            std::cout << "INFO: Joint " << _joint_vector[i]->getJointName() << " - with id : "
                      << _joint_vector[i]->getJointId()
                      << " - control mode changed to " << control_mode.getName() << std::endl;
        }
        else {
            std::cout << "WARNING: Joint " << _joint_vector[i]->getJointName()  << " - with id : "
                      << _joint_vector[i]->getJointId() << " - CANNOT change control mode to "
                      << control_mode.getName() << std::endl;
        }

        success = set_internal_success && success;
    }

    return success;
}



bool XBot::RobotInterface::setControlMode(const std::string& chain_name, const XBot::ControlMode& control_mode)
{
    auto it = _chain_map.find(chain_name);

    if( it == _chain_map.end() ){
        std::cerr << "ERROR in " << __func__ << "! Chain " << chain_name << " is NOT defined!" << std::endl;
        return false;
    }

    bool success = true;

    for( int i = 0; i < it->second->getJointNum(); i++){

        bool set_internal_success = set_control_mode_internal(it->second->getJoint(i)->getJointId(), control_mode);

        if( set_internal_success ){
            it->second->getJointInternal(i)->setControlMode(control_mode);
            std::cout << "INFO: Joint " << (it->second->getJoint(i)->getJointName()) << " - with id : "
                      << (it->second->getJoint(i)->getJointId())
                      << " - control mode changed to " << control_mode.getName() << std::endl;
        }
        else {
            std::cout << "WARNING: Joint " << (it->second->getJoint(i)->getJointName())  << " - with id : "
                      << (it->second->getJoint(i)->getJointId()) << " - CANNOT change control mode to "
                      << control_mode.getName() << std::endl;
        }

        success = success && set_internal_success;
    }

    return success;
}










