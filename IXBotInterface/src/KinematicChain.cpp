#include <XBotInterface/KinematicChain.h>
#include <eigen3/Eigen/Dense>

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

KinematicChain::KinematicChain(const KinematicChain& other):
// Default copy of all non-ptr members
_urdf_joints(other._urdf_joints),
_urdf_links(other._urdf_links),
_ordered_joint_name(other._ordered_joint_name),
_ordered_joint_id(other._ordered_joint_id),
_XBotModel(other._XBotModel),
_chain_name(other._chain_name),
_joint_num(other._joint_num)
{
  
  
  for( const Joint::Ptr& other_jptr : other._joint_vector ){
    
    Joint::Ptr jptr = std::make_shared<Joint>(); 
    *jptr = *other_jptr;
    
    _joint_vector.push_back(jptr);
    _joint_name_map[jptr->getJointName()] = jptr;
    _joint_id_map[jptr->getJointId()] = jptr;
    
  }
  

}


KinematicChain& KinematicChain::operator=(const KinematicChain& rhs)
{
  
    // First, make a copy of rhs exploiting the custom (deep) copy constructor
    KinematicChain tmp(rhs);
    
    // Swap all elements
    std::swap(_urdf_joints, tmp._urdf_joints);
    std::swap(_urdf_links, tmp._urdf_links);
    std::swap(_ordered_joint_name, tmp._ordered_joint_name);
    std::swap(_ordered_joint_id, tmp._ordered_joint_id);
    std::swap(_XBotModel, tmp._XBotModel);
    std::swap(_chain_name, tmp._chain_name);
    std::swap(_joint_num, tmp._joint_num);
    std::swap(_joint_vector, tmp._joint_vector);
    std::swap(_joint_name_map, tmp._joint_name_map);
    std::swap(_joint_id_map, tmp._joint_id_map);
    
    // Return this
    return *this;
  
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

double KinematicChain::getLinkPos(int index) const
{
    return _joint_vector[index]->getLinkPos();
}


bool XBot::KinematicChain::getLinkPos(Eigen::VectorXd& q) const
{
    if(q.rows() != _joint_num) {
        q.resize(_joint_num);
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        q[pos++] = j->getLinkPos();
    }
    return true;
}

bool KinematicChain::setLinkPos(const Eigen::VectorXd& q)
{
    if(q.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : q has wrong size " << q.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        j->setLinkPos(q[pos++]);
    }
    return true;
}

bool XBot::KinematicChain::sync(const KinematicChain& other)
{
    int pos = 0;
    for( const XBot::Joint::Ptr& j : other._joint_vector) {
        _joint_vector[pos++]->sync(*j);
    }
}






} // end namespace XBot