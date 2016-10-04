#include <XBotInterface/KinChain.h>

XBot::KinChain::KinChain() : 
    _chain_name("dummy_chain"),
    _joint_num(-1)
{

}

XBot::KinChain::KinChain(std::string chain_name, 
                   std::map<std::string, int> joint_name_id_map) : 
    _chain_name(chain_name),
    _joint_num(0)
{
    // initialize the joint maps
    for(const auto& j : joint_name_id_map) {
        std::string actual_joint_name = j.first;
        int actual_joint_id = j.second;
        XBot::Joint::Ptr actual_joint = std::make_shared<Joint>(actual_joint_name, 
                                                                    actual_joint_id);
        _joint_name_map[actual_joint_name] = actual_joint;
        _joint_id_map[actual_joint_id] = actual_joint;
        _joint_num++;
    }
    // initialize joint vector
    initialize_joint_vector();
    
}


void XBot::KinChain::initialize_joint_vector()
{
    _joint_vector.resize(_joint_num);
    // TBD what else? what order?
}
