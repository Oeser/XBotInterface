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

#ifndef __I_XBOT_INTERFACE_H__
#define __I_XBOT_INTERFACE_H__

#include <vector>
#include <map>
#include <memory>

#include <XBotInterface/KinematicChain.h>

namespace XBot {
    
class IXBotInterface {
  
  public:
    
    typedef std::shared_ptr<IXBotInterface> Ptr;
    
    explicit IXBotInterface(const XBotCoreModel& XBotModel);
    
    IXBotInterface(const IXBotInterface& other);

    static IXBotInterface::Ptr getRobot(const std::string& cfg);

    virtual void test() = 0;
    
    virtual ~IXBotInterface();
    
        // TODO: bool hasVelocity(), hasImpedance(), ....
    
    const urdf::ModelInterface& getUrdf() const;
    const srdf::Model& getSrdf() const;
    const std::string& getUrdfString() const;
    const std::string& getSrdfString() const;
    std::vector<std::string> getChainNames() const;
    
    KinematicChain& operator()(const std::string& group_name, int id);
    KinematicChain& operator()(const std::string& chain_name);
    KinematicChain& leg(int id);
    KinematicChain& arm(int id);

    
    int legs() const;
    int arms() const;
    bool findGroup(const std::string& group_name) const;
    
    bool setJointPosition    (const IXBotInterface& other);
    bool setJointVelocity    (const IXBotInterface& other);
    bool setJointEffort      (const IXBotInterface& other);
    bool setJointImpedance   (const IXBotInterface& other);
    bool setJointAll         (const IXBotInterface& other);
    
    

  protected:
    
    IXBotInterface& operator= (const IXBotInterface& rhs);
    

  private:

    int _joint_num;
    XBotCoreModel _XBotModel;
    std::string _urdf_string, _srdf_string;
    
    std::vector<std::string> _ordered_joint_name;
    std::vector<int> _ordered_joint_id;
    
    std::map<std::string, XBot::KinematicChain::Ptr> _chain_map;

};


class YARPInterface : public IXBotInterface {
  friend IXBotInterface::Ptr XBot::IXBotInterface::getRobot(const std::string& cfg); // NOTE careful
public:
  virtual void test();
  
  virtual ~YARPInterface();

protected:
  YARPInterface(const XBotCoreModel& XBotModel);
};
}

#endif // __I_XBOT_INTERFACE_H__