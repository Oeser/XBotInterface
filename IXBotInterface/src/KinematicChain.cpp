#include <XBotInterface/KinematicChain.h>

namespace XBot {

KinematicChain::KinematicChain() : 
    _chain_name("dummy_chain"),
    _joint_num(-1)
{

}

KinematicChain::KinematicChain(const std::string& chain_name,
                               const XBot::XBotCoreModel& XBotModel) : 
    _chain_name(chain_name),
    _XBotModel(XBotModel)
{
    _joint_num = _XBotModel.get_joint_num(chain_name);
    _XBotModel.get_enabled_joints_in_chain(chain_name, _ordered_joint_name);
    _XBotModel.get_enabled_joint_ids_in_chain(chain_name, _ordered_joint_id);
    // resize joint vector
    _joint_vector.resize(_joint_num);
    // initialize the joint maps and vector
    for(int i = 0; i < _joint_num; i++) {
        std::string actual_joint_name = _ordered_joint_name[i];
        int actual_joint_id = _ordered_joint_id[i];
        XBot::Joint::Ptr actual_joint = std::make_shared<Joint>(actual_joint_name, 
                                                                actual_joint_id);
        _joint_name_map[actual_joint_name] = actual_joint;
        _joint_id_map[actual_joint_id] = actual_joint;
        _joint_vector[i] = actual_joint;
    }
  
  // Get urdf model
  const urdf::ModelInterface& robot_urdf = *XBotModel.get_urdf_model();
  
  // Fill _urdf_joints and _urdf_links
  // NOTE: is it the correct way to handle links? What happens if some joint
  // along the chain is disabled/fixed/...?
  for( const std::string& joint_name : _ordered_joint_name ){
    
    _urdf_joints.push_back(robot_urdf.getJoint(joint_name));
    
    const std::string& parent_link_name = robot_urdf.getJoint(joint_name)->parent_link_name;
    _urdf_links.push_back(robot_urdf.getLink(parent_link_name));
    
  }
  
  // Add last link to _urdf_links
  _urdf_links.push_back(robot_urdf.getLink(childLinkName(getJointNum()-1)));

}

const std::vector< XBot::JointConstSharedPtr >& KinematicChain::getJoints() const
{
  return _urdf_joints;
}

const std::vector< XBot::LinkConstSharedPtr >& KinematicChain::getLinks() const
{
  return _urdf_links;
}

int KinematicChain::getJointNum() const
{
  return _joint_vector.size();
}

const std::string& KinematicChain::baseLinkName() const
{
  return _urdf_links[0]->name;
}

const std::string& KinematicChain::tipLinkName() const
{
  return _urdf_links[_urdf_links.size()-1]->name;
}

const std::string& KinematicChain::chainName() const
{
  return _chain_name;
}

const std::string& KinematicChain::childLinkName(int id) const
{
  return _urdf_joints[id]->child_link_name;
}

const std::string& KinematicChain::parentLinkName(int id) const
{
  return _urdf_joints[id]->parent_link_name;
}

const std::string& KinematicChain::jointName(int id) const
{
  return _urdf_joints[id]->name;
}



} // end namespace XBot