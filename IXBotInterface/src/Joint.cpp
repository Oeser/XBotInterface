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









