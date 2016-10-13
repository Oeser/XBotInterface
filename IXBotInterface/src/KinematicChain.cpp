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

int KinematicChain::jointId(int i) const
{
	return _joint_vector[i]->getJointId();
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


bool KinematicChain::getLinkPos(Eigen::VectorXd& q) const
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

bool KinematicChain::getLinkPos(std::map< std::string, double >& q) const
{
	q.clear();
	
	for( const auto& name_joint_pair : _joint_name_map ){
		
		const std::string& joint_name = name_joint_pair.first;
		const Joint& joint = *name_joint_pair.second;
		
		q[joint_name] = joint.getLinkPos();
		
	}
		
}

bool KinematicChain::getDamping(std::map< std::string, double >& D) const
{
	D.clear();
	
	for( const auto& name_joint_pair : _joint_name_map ){
		
		const std::string& joint_name = name_joint_pair.first;
		const Joint& joint = *name_joint_pair.second;
		
		D[joint_name] = joint.getDamping();
		
	}
}

bool KinematicChain::getEffort(std::map< std::string, double >& tau) const
{
	tau.clear();
	
	for( const auto& name_joint_pair : _joint_name_map ){
		
		const std::string& joint_name = name_joint_pair.first;
		const Joint& joint = *name_joint_pair.second;
		
		tau[joint_name] = joint.getEffort();
		
	}
}

bool KinematicChain::getEffortRef(std::map< std::string, double >& tau) const
{
	tau.clear();
	
	for( const auto& name_joint_pair : _joint_name_map ){
		
		const std::string& joint_name = name_joint_pair.first;
		const Joint& joint = *name_joint_pair.second;
		
		tau[joint_name] = joint.getEffortRef();
		
	}
}

bool KinematicChain::getLinkVel(std::map< std::string, double >& qdot) const
{
	qdot.clear();
	
	for( const auto& name_joint_pair : _joint_name_map ){
		
		const std::string& joint_name = name_joint_pair.first;
		const Joint& joint = *name_joint_pair.second;
		
		qdot[joint_name] = joint.getLinkVel();
		
	}
}

bool KinematicChain::getMotorPos(std::map< std::string, double >& q) const
{
	q.clear();
	
	for( const auto& name_joint_pair : _joint_name_map ){
		
		const std::string& joint_name = name_joint_pair.first;
		const Joint& joint = *name_joint_pair.second;
		
		q[joint_name] = joint.getMotorPos();
		
	}
}

bool KinematicChain::getMotorVel(std::map< std::string, double >& qdot) const
{
	qdot.clear();
	
	for( const auto& name_joint_pair : _joint_name_map ){
		
		const std::string& joint_name = name_joint_pair.first;
		const Joint& joint = *name_joint_pair.second;
		
		qdot[joint_name] = joint.getMotorVel();
		
	}
}

bool KinematicChain::getStiffness(std::map< std::string, double >& K) const
{
	K.clear();
	
	for( const auto& name_joint_pair : _joint_name_map ){
		
		const std::string& joint_name = name_joint_pair.first;
		const Joint& joint = *name_joint_pair.second;
		
		K[joint_name] = joint.getStiffness();
		
	}
}

bool KinematicChain::getTemperature(std::map< std::string, double >& temp) const
{
	temp.clear();
	
	for( const auto& name_joint_pair : _joint_name_map ){
		
		const std::string& joint_name = name_joint_pair.first;
		const Joint& joint = *name_joint_pair.second;
		
		temp[joint_name] = joint.getTemperature();
		
	}
}

bool KinematicChain::getVelRef(std::map< std::string, double >& qdot) const
{
	qdot.clear();
	
	for( const auto& name_joint_pair : _joint_name_map ){
		
		const std::string& joint_name = name_joint_pair.first;
		const Joint& joint = *name_joint_pair.second;
		
		qdot[joint_name] = joint.getVelRef();
		
	}
}




bool KinematicChain::getLinkPos(std::map< int, double >& q) const
{
	q.clear();
	
	for( const auto& id_joint_pair : _joint_id_map ){
		
		int joint_id = id_joint_pair.first;
		const Joint& joint = *id_joint_pair.second;
		
		q[joint_id] = joint.getLinkPos();
		
	}
		
}

bool KinematicChain::getPosRef(Eigen::VectorXd& q) const
{
    if(q.rows() != _joint_num) {
        q.resize(_joint_num);
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        q[pos++] = j->getPosRef();
    }
    return true;	
}

bool KinematicChain::getPosRef(std::map< std::string, double >& q) const
{
	q.clear();
	
	for( const auto& name_joint_pair : _joint_name_map ){
		
		const std::string& joint_name = name_joint_pair.first;
		const Joint& joint = *name_joint_pair.second;
		
		q[joint_name] = joint.getPosRef();
		
	}
}

bool KinematicChain::getPosRef(std::map< int, double >& q) const
{
	q.clear();
	
	for( const auto& id_joint_pair : _joint_id_map ){
		
		int joint_id = id_joint_pair.first;
		const Joint& joint = *id_joint_pair.second;
		
		q[joint_id] = joint.getPosRef();
		
	}
}

bool KinematicChain::getDamping(std::map< int, double >& D) const
{
	D.clear();
	
	for( const auto& id_joint_pair : _joint_id_map ){
		
		int joint_id = id_joint_pair.first;
		const Joint& joint = *id_joint_pair.second;
		
		D[joint_id] = joint.getDamping();
		
	}
}

bool KinematicChain::getEffort(std::map< int, double >& tau) const
{
	tau.clear();
	
	for( const auto& id_joint_pair : _joint_id_map ){
		
		int joint_id = id_joint_pair.first;
		const Joint& joint = *id_joint_pair.second;
		
		tau[joint_id] = joint.getEffort();
		
	}
}

bool KinematicChain::getEffortRef(std::map< int, double >& tau) const
{
	tau.clear();
	
	for( const auto& id_joint_pair : _joint_id_map ){
		
		int joint_id = id_joint_pair.first;
		const Joint& joint = *id_joint_pair.second;
		
		tau[joint_id] = joint.getEffortRef();
		
	}
}

bool KinematicChain::getLinkVel(std::map< int, double >& qdot) const
{
	qdot.clear();
	
	for( const auto& id_joint_pair : _joint_id_map ){
		
		int joint_id = id_joint_pair.first;
		const Joint& joint = *id_joint_pair.second;
		
		qdot[joint_id] = joint.getLinkVel();
		
	}
}

bool KinematicChain::getMotorPos(std::map< int, double >& q) const
{
	q.clear();
	
	for( const auto& id_joint_pair : _joint_id_map ){
		
		int joint_id = id_joint_pair.first;
		const Joint& joint = *id_joint_pair.second;
		
		q[joint_id] = joint.getMotorPos();
		
	}
}

bool KinematicChain::getMotorVel(std::map< int, double >& qdot) const
{
	qdot.clear();
	
	for( const auto& id_joint_pair : _joint_id_map ){
		
		int joint_id = id_joint_pair.first;
		const Joint& joint = *id_joint_pair.second;
		
		qdot[joint_id] = joint.getMotorVel();
		
	}
}

bool KinematicChain::getStiffness(std::map< int, double >& K) const
{
	K.clear();
	
	for( const auto& id_joint_pair : _joint_id_map ){
		
		int joint_id = id_joint_pair.first;
		const Joint& joint = *id_joint_pair.second;
		
		K[joint_id] = joint.getStiffness();
		
	}
}

bool KinematicChain::getTemperature(std::map< int, double >& temp) const
{
	temp.clear();
	
	for( const auto& id_joint_pair : _joint_id_map ){
		
		int joint_id = id_joint_pair.first;
		const Joint& joint = *id_joint_pair.second;
		
		temp[joint_id] = joint.getTemperature();
		
	}
}

bool KinematicChain::getVelRef(std::map< int, double >& qdot) const
{
	qdot.clear();
	
	for( const auto& id_joint_pair : _joint_id_map ){
		
		int joint_id = id_joint_pair.first;
		const Joint& joint = *id_joint_pair.second;
		
		qdot[joint_id] = joint.getVelRef();
		
	}	
}

bool KinematicChain::getDamping(Eigen::VectorXd& D) const
{
    if(D.rows() != _joint_num) {
        D.resize(_joint_num);
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        D[pos++] = j->getDamping();
    }
    return true;
}

bool KinematicChain::getEffort(Eigen::VectorXd& tau) const
{
    if(tau.rows() != _joint_num) {
        tau.resize(_joint_num);
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        tau[pos++] = j->getEffort();
    }
    return true;
}

bool KinematicChain::getEffortRef(Eigen::VectorXd& tau) const
{
    if(tau.rows() != _joint_num) {
        tau.resize(_joint_num);
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        tau[pos++] = j->getEffortRef();
    }
    return true;
}

bool KinematicChain::getLinkVel(Eigen::VectorXd& qdot) const
{
    if(qdot.rows() != _joint_num) {
        qdot.resize(_joint_num);
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        qdot[pos++] = j->getLinkVel();
    }
    return true;
}

bool KinematicChain::getMotorPos(Eigen::VectorXd& q) const
{
    if(q.rows() != _joint_num) {
        q.resize(_joint_num);
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        q[pos++] = j->getMotorPos();
    }
    return true;
}

bool KinematicChain::getMotorVel(Eigen::VectorXd& qdot) const
{
    if(qdot.rows() != _joint_num) {
        qdot.resize(_joint_num);
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        qdot[pos++] = j->getMotorVel();
    }
    return true;
}

bool KinematicChain::getStiffness(Eigen::VectorXd& K) const
{
    if(K.rows() != _joint_num) {
        K.resize(_joint_num);
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        K[pos++] = j->getStiffness();
    }
    return true;
}

bool KinematicChain::getTemperature(Eigen::VectorXd& temp) const
{
    if(temp.rows() != _joint_num) {
        temp.resize(_joint_num);
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        temp[pos++] = j->getTemperature();
    }
    return true;
}

bool KinematicChain::getVelRef(Eigen::VectorXd& qdot) const
{
    if(qdot.rows() != _joint_num) {
        qdot.resize(_joint_num);
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        qdot[pos++] = j->getVelRef();
    }
    return true;
}




double KinematicChain::getPosRef(int index) const
{
	if( index >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index+1 << " joints!" << std::endl;
	}
	return _joint_vector[index]->getPosRef();
}

double KinematicChain::getEffort(int index) const
{
	if( index >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index+1 << " joints!" << std::endl;
	}	
	return _joint_vector[index]->getEffort();
}

double KinematicChain::getEffortRef(int index) const
{
	if( index >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index+1 << " joints!" << std::endl;
	}	
	return _joint_vector[index]->getEffortRef();
}

double KinematicChain::getLinkVel(int index) const
{
	if( index >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index+1 << " joints!" << std::endl;
	}	
	return _joint_vector[index]->getLinkVel();
}

double KinematicChain::getMotorPos(int index) const
{
	if( index >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index+1 << " joints!" << std::endl;
	}	
	return _joint_vector[index]->getMotorPos();
}

double KinematicChain::getMotorVel(int index) const
{
	if( index >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index+1 << " joints!" << std::endl;
	}	
	return _joint_vector[index]->getMotorVel();
}

double KinematicChain::getStiffness(int index) const
{
	if( index >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index+1 << " joints!" << std::endl;
	}	
	return _joint_vector[index]->getStiffness();
}

double KinematicChain::getTemperature(int index) const
{
	if( index >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index+1 << " joints!" << std::endl;
	}	
	return _joint_vector[index]->getTemperature();
}

double KinematicChain::getVelRef(int index) const
{
	if( index >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index+1 << " joints!" << std::endl;
	}	
	return _joint_vector[index]->getVelRef();
}

double KinematicChain::getDamping(int index) const
{
	if( index >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << index+1 << " joints!" << std::endl;
	}	
	return _joint_vector[index]->getDamping();
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


bool KinematicChain::setEffort(const Eigen::VectorXd& tau)
{
    if(tau.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : tau has wrong size " << tau.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        j->setEffort(tau[pos++]);
    }
    return true;
}

bool KinematicChain::setEffortRef(const Eigen::VectorXd& tau)
{
    if(tau.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : tau has wrong size " << tau.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        j->setEffortRef(tau[pos++]);
    }
    return true;
}

bool KinematicChain::setDamping(const Eigen::VectorXd& D)
{
    if(D.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : D has wrong size " << D.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        j->setDamping(D[pos++]);
    }
    return true;
}

bool KinematicChain::setLinkVel(const Eigen::VectorXd& qdot)
{
    if(qdot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qdot has wrong size " << qdot.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        j->setLinkVel(qdot[pos++]);
    }
    return true;
}

bool KinematicChain::setMotorPos(const Eigen::VectorXd& q)
{
    if(q.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : q has wrong size " << q.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        j->setMotorPos(q[pos++]);
    }
    return true;
}

bool KinematicChain::setMotorVel(const Eigen::VectorXd& qdot)
{
    if(qdot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qdot has wrong size " << qdot.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        j->setMotorVel(qdot[pos++]);
    }
    return true;
}

bool KinematicChain::setPosRef(const Eigen::VectorXd& q)
{
    if(q.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : q has wrong size " << q.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        j->setPosRef(q[pos++]);
    }
    return true;
}

bool KinematicChain::setStiffness(const Eigen::VectorXd& K)
{
    if(K.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : K has wrong size " << K.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        j->setStiffness(K[pos++]);
    }
    return true;
}

bool KinematicChain::setTemperature(const Eigen::VectorXd& temp)
{
    if(temp.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : temp has wrong size " << temp.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        j->setTemperature(temp[pos++]);
    }
    return true;
}

bool KinematicChain::setVelRef(const Eigen::VectorXd& qdot)
{
    if(qdot.rows() != _joint_num) {
        std::cerr << "ERROR in " << __func__ << " : qdot has wrong size " << qdot.rows() << " != chain joint number " << _joint_num << std::endl;
        return false;
    }
    int pos = 0;
    for( const XBot::Joint::Ptr& j : _joint_vector) {
        j->setVelRef(qdot[pos++]);
    }
    return true;
}






bool KinematicChain::setLinkPos(int i, double q){
	if( i >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i+1 << " joints!" << std::endl;
		return false;
	}
	_joint_vector[i]->setLinkPos(q);
}

bool KinematicChain::setEffort(int i, double tau)
{
	if( i >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i+1 << " joints!" << std::endl;
		return false;
	}
	_joint_vector[i]->setEffort(tau);
}

bool KinematicChain::setLinkVel(int i, double qdot)
{
	if( i >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i+1 << " joints!" << std::endl;
		return false;
	}
	_joint_vector[i]->setLinkVel(qdot);
}

bool KinematicChain::setMotorPos(int i, double q)
{
	if( i >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i+1 << " joints!" << std::endl;
		return false;
	}
	_joint_vector[i]->setMotorPos(q);
}

bool KinematicChain::setMotorVel(int i, double qdot)
{
	if( i >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i+1 << " joints!" << std::endl;
		return false;
	}
	_joint_vector[i]->setMotorVel(qdot);
}

bool KinematicChain::setTemperature(int i, double temp)
{
	if( i >= this->getJointNum() ){ 
		std::cerr << "ERROR in " << __func__ << " : chain " << chainName() << " has less than " << i+1 << " joints!" << std::endl;
		return false;
	}
	_joint_vector[i]->setTemperature(temp);
}




bool KinematicChain::sync(const KinematicChain& other)
{
	// TBD: check that chains are indeed omologues??
    int pos = 0;
    for( const XBot::Joint::Ptr& j : other._joint_vector) {
        _joint_vector[pos++]->sync(*j);
    }
}






} // end namespace XBot