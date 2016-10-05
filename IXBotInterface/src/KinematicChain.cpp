#include <XBotInterface/KinematicChain.h>

XBot::KinematicChain::KinematicChain() : 
    _chain_name("dummy_chain"),
    _joint_num(-1)
{

}

XBot::KinematicChain::KinematicChain(const std::string& chain_name,
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
    
}
