#include <XBotInterface/Joint.h>

XBot::Joint::Joint() :
    _joint_name(""),
    _joint_id(-1)
{

}

XBot::Joint::Joint(const std::string& joint_name, 
                   int joint_id) :
    _joint_name(joint_name),
    _joint_id(joint_id)
{

}

const std::string& XBot::Joint::getJointName() const
{
    return _joint_name;
}

int XBot::Joint::getJointId() const
{
    return _joint_id;
}

void XBot::Joint::setJointName(const std::string& joint_name)
{
    _joint_name = joint_name;
}

void XBot::Joint::setJointId(int joint_id)
{
    _joint_id = joint_id;
}



double XBot::Joint::getLinkPos() const
{
    return _link_pos;
}

double XBot::Joint::getMotorPos() const
{
    return _motor_pos;
}

double XBot::Joint::getLinkVel() const
{
    return _link_vel;
}

double XBot::Joint::getMotorVel() const
{
    return _motor_vel;
}

double XBot::Joint::getEffort() const
{
    return _effort;
}

double XBot::Joint::getTemperature() const
{
    return _temperature;
}


void XBot::Joint::setLinkPos(double link_pos)
{
    _link_pos = link_pos;
}

void XBot::Joint::setMotorPos(double motor_pos)
{
    _motor_pos = motor_pos;
}

void XBot::Joint::setLinkVel(double link_vel)
{
    _link_vel = link_vel;
}

void XBot::Joint::setMotorVel(double motor_vel)
{
    _motor_vel = motor_vel;
}

void XBot::Joint::setEffort(double effort)
{
    _effort = effort;
}

void XBot::Joint::setTemperature(double temperature)
{
    _temperature = temperature;
}

double XBot::Joint::getPosRef() const
{
    return _pos_ref;
}

double XBot::Joint::getVelRef() const
{
    return _vel_ref;
}

double XBot::Joint::getEffortRef() const
{
    return _effort_ref;
}

double XBot::Joint::getStiffness() const
{
    return _stiffness;
}

double XBot::Joint::getDamping() const
{
    return _damping;
}

void XBot::Joint::setPosRef(double pos_ref)
{
    _pos_ref = pos_ref;
}

void XBot::Joint::setVelRef(double vel_ref)
{
    _vel_ref = vel_ref;
}

void XBot::Joint::setEffortRef(double effort_ref)
{
    _effort_ref = effort_ref;
}

void XBot::Joint::setStiffness(double stiffness)
{
    _stiffness = stiffness;
}

void XBot::Joint::setDamping(double damping)
{
    _damping = damping;
}

bool XBot::Joint::sync(const XBot::Joint& other)
{
        _joint_name = other._joint_name;
        _joint_id = other._joint_id;
        
        ///////////////////
        // RX from board //
        ///////////////////
        
        _link_pos = other._link_pos;
        _motor_pos = other._motor_pos;
        _link_vel = other._link_vel;
        _motor_vel = other._motor_vel;
        _effort = other._effort;
        _temperature = other._temperature;
        
        /////////////////
        // TX to board //
        /////////////////
        
        _pos_ref = other._pos_ref;
        _vel_ref = other._vel_ref;
        _effort_ref = other._effort_ref;
        _stiffness = other._stiffness;
        _damping = other._damping;
}
















