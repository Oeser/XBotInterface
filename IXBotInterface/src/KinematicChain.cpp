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
//      _joint_num(_XBotModel.),
//     _ordered_joint_name(ordered_joint_name),
//     _ordered_joint_id(ordered_joint_id)
//     // resize joint vector
//     _joint_vector.resize(_joint_num);
//     // initialize the joint maps and vector
//     for(int i = 0; i < _joint_num; i++) {
//         std::string actual_joint_name = ordered_joint_name[i];
//         int actual_joint_id = ordered_joint_id[i];
//         XBot::Joint::Ptr actual_joint = std::make_shared<Joint>(actual_joint_name, 
//                                                                 actual_joint_id);
//         _joint_name_map[actual_joint_name] = actual_joint;
//         _joint_id_map[actual_joint_id] = actual_joint;
//         _joint_vector[i] = actual_joint;
//     }
  
  // Get urdf model
  urdf::ModelInterface& robot_urdf = *XBotModel.get_urdf_model();
  
  // Fill urdf_joints_ and urdf_links_
  // TODO: is it the correct way to handle links? What happens if some joint
  // along the chain is disabled/fixed/...?
  for( const std::string& joint_name : _ordered_joint_name ){
    
    urdf_joints_.push_back(robot_urdf.getJoint(joint_name));
    
    const std::string& parent_link_name = robot_urdf.getJoint(joint_name)->parent_link_name;
    urdf_links_.push_back(robot_urdf.getLink(parent_link_name));
    
  }
  
  // Add last link to urdf_links_
  urdf_links_.push_back(robot_urdf.getLink(childLinkName(getJointNum()-1)));
  
  
  
    
}

const std::vector< urdf::JointConstSharedPtr >& KinematicChain::getJoints() const
{
  return urdf_joints_;
}

const std::vector< urdf::LinkConstSharedPtr >& KinematicChain::getLinks() const
{
  return urdf_links_;
}

int KinematicChain::getJointNum() const
{
  return _joint_vector.size();
}

const std::string& KinematicChain::baseLinkName() const
{
  return urdf_links_[0]->name;
}

const std::string& KinematicChain::tipLinkName() const
{
  return urdf_links_[urdf_links_.size()-1]->name;
}

const std::string& KinematicChain::chainName() const
{
  return _chain_name;
}

const std::string& KinematicChain::childLinkName(int id) const
{
  return urdf_joints_[id]->child_link_name;
}

const std::string& KinematicChain::parentLinkName(int id) const
{
  return urdf_joints_[id]->parent_link_name;
}

const std::string& KinematicChain::jointName(int id) const
{
  return urdf_joints_[id]->name;
}



} // end namespace XBot