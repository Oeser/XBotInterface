#include <XBotInterface/IXBotInterface.h>

#include <yaml-cpp/yaml.h>
#include <XBotCoreModel.h>

XBot::IXBotInterface::IXBotInterface(const XBot::XBotCoreModel& XBotModel) : 
    _XBotModel(XBotModel)
{
    _joint_num = XBotModel.get_joint_num();
    _urdf_string = XBotModel.get_urdf_string();
    _srdf_string = XBotModel.get_srdf_string();
    XBotModel.get_enabled_joint_ids(_ordered_joint_id);
    XBotModel.get_enabled_joint_names(_ordered_joint_name);
    
    for( const std::string& chain_name : XBotModel.get_chain_names() ) {
        XBot::KinematicChain::Ptr actual_chain = std::make_shared<KinematicChain>(chain_name, 
                                                                                  XBotModel);
        _chain_map[chain_name] = actual_chain;
    }
    
}

XBot::IXBotInterface::IXBotInterface(const XBot::IXBotInterface& other):
  _joint_num(other._joint_num),
  _ordered_joint_name(other._ordered_joint_name),
  _ordered_joint_id(other._ordered_joint_id),
  _XBotModel(other._XBotModel),
  _urdf_string(other._urdf_string),
  _srdf_string(other._srdf_string)
{
  
  for( const auto& chain_name_ptr_pair : other._chain_map ){
   
    const std::string& chain_name = chain_name_ptr_pair.first;
    const XBot::KinematicChain::Ptr& other_chainptr = chain_name_ptr_pair.second;
    
    XBot::KinematicChain::Ptr chainptr = std::make_shared<XBot::KinematicChain>();
    *chainptr = *other_chainptr;
    
    _chain_map[chain_name] = chainptr;
    
  }

}

XBot::KinematicChain& XBot::IXBotInterface::operator()(const std::string& chain_name)
{
    if(_chain_map.count(chain_name)) {
        return *_chain_map.at(chain_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a chain with name " << chain_name << " that does not exists!!" << std::endl;
    return _dummy_chain;
}


XBot::KinematicChain& XBot::IXBotInterface::leg(int id)
{
    if(_XBotModel.get_legs_chain().size() > id) {
        const std::string& requested_leg_name = _XBotModel.get_legs_chain().at(id);
        return *_chain_map.at(requested_leg_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a legs with id " << id << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

int XBot::IXBotInterface::legs() const
{
    return _XBotModel.get_legs_chain().size();
}

XBot::KinematicChain& XBot::IXBotInterface::arm(int id)
{
    if(_XBotModel.get_arms_chain().size() > id) {
        const std::string& requested_arm_name = _XBotModel.get_arms_chain().at(id);
        return *_chain_map.at(requested_arm_name);
    }
    std::cerr << "ERROR " << __func__ << " : you are requesting a arms with id " << id << " that does not exists!!" << std::endl;
    return _dummy_chain;
}

int XBot::IXBotInterface::arms() const
{
    return _XBotModel.get_arms_chain().size();
}

bool XBot::IXBotInterface::sync(const XBot::IXBotInterface& other)
{
    for(const auto& c : other._chain_map) {
        const std::string& chain_name = c.first;
        const KinematicChain& chain = *c.second;
        if(_chain_map.count(chain_name)) {
            _chain_map.at(chain_name)->sync(chain);
        }
        else {
            std::cerr << "ERROR " << __func__ << " : you are trying to synchronize IXBotInterfaces with different chains!!" << std::endl;
        }
    }
}


bool XBot::IXBotInterface::hasJoint(const std::string& joint_name) const
{
	return std::find(_ordered_joint_name.begin(), _ordered_joint_name.end(), joint_name) !=  _ordered_joint_name.end();
}





bool XBot::IXBotInterface::getLinkPos(Eigen::VectorXd& q) const
{
    if(q.rows() != _joint_num) {
        q.resize(_joint_num);
    }
    int q_index = 0;
    for( const std::string& chain_name : _XBotModel.get_ordered_chain_names()) {
        for( int i = 0; i < _chain_map.at(chain_name)->getJointNum(); i++) {
            q[q_index++] = _chain_map.at(chain_name)->getLinkPos(i);
        }
    }
    return true;
}

bool XBot::IXBotInterface::getLinkPos(std::map< std::string, double >& q) const
{
	q.clear();
	
	for( const auto& chain_name_ptr_pair : _chain_map ){
		
		const KinematicChain& chain = *chain_name_ptr_pair.second;
		
		for(int j=0; j<chain.getJointNum(); j++){
			q[chain.jointName(j)] = chain.getLinkPos(j);
		}
		
	}
}

bool XBot::IXBotInterface::getLinkPos(std::map< int, double >& q) const
{
	q.clear();
	
	for( const auto& chain_name_ptr_pair : _chain_map ){
		
		const KinematicChain& chain = *chain_name_ptr_pair.second;
		
		for(int j=0; j<chain.getJointNum(); j++){
			q[chain.jointId(j)] = chain.getLinkPos(j);
		}
		
	}
}

bool XBot::IXBotInterface::getPosRef(std::map< std::string, double >& q) const
{
	q.clear();
	
	for( const auto& chain_name_ptr_pair : _chain_map ){
		
		const KinematicChain& chain = *chain_name_ptr_pair.second;
		
		for(int j=0; j<chain.getJointNum(); j++){
			q[chain.jointName(j)] = chain.getPosRef(j);
		}
		
	}
}

bool XBot::IXBotInterface::getPosRef(std::map< int, double >& q) const
{
	q.clear();
	
	for( const auto& chain_name_ptr_pair : _chain_map ){
		
		const KinematicChain& chain = *chain_name_ptr_pair.second;
		
		for(int j=0; j<chain.getJointNum(); j++){
			q[chain.jointId(j)] = chain.getPosRef(j);
		}
		
	}
}

bool XBot::IXBotInterface::getPosRef(Eigen::VectorXd& q) const
{
    if(q.rows() != _joint_num) {
        q.resize(_joint_num);
    }
    int q_index = 0;
    for( const std::string& chain_name : _XBotModel.get_ordered_chain_names()) {
        for( int i = 0; i < _chain_map.at(chain_name)->getJointNum(); i++) {
            q[q_index++] = _chain_map.at(chain_name)->getPosRef(i);
        }
    }
    return true;
}

bool XBot::IXBotInterface::setLinkPos(const std::map< std::string, double >& q)
{
	for( const auto& chainname_ptr_pair : _chain_map ){
		
		KinematicChain& chain = *chainname_ptr_pair.second;
		
		for(int i = 0; i<chain.getJointNum(); i++){
			const std::string& joint_name = chain.jointName(i);
				if(q.count(joint_name)){
					chain.setLinkPos(i, q.at(joint_name));
				}
		}
		
	}
}




XBot::IXBotInterface::~IXBotInterface()
{
}

XBot::IXBotInterface& XBot::IXBotInterface::operator=(const XBot::IXBotInterface& rhs)
{
  IXBotInterface tmp(rhs);
  std::swap(_joint_num, tmp._joint_num);
  std::swap(_XBotModel, tmp._XBotModel);
  std::swap(_urdf_string, tmp._urdf_string);
  std::swap(_srdf_string, tmp._srdf_string);
  std::swap(_ordered_joint_name, tmp._ordered_joint_name);
  std::swap(_ordered_joint_id, tmp._ordered_joint_id);
  std::swap(_chain_map, tmp._chain_map);
  std::swap(_dummy_chain, tmp._dummy_chain);

}


const std::vector< std::string >& XBot::IXBotInterface::getEnabledJointNames() const
{
	return _ordered_joint_name;
}
