#include <XBotInterface/Joint.h>

XBot::Joint::Joint() :
    _joint_name(""),
    _joint_id(-1)
{

}

XBot::Joint::Joint(std::string joint_name, int joint_id) :
    _joint_name(joint_name),
    _joint_id(joint_id)
{

}

std::string XBot::Joint::getJointName()
{
    return _joint_name;
}

int XBot::Joint::getJointId()
{
    return _joint_id;
}

void XBot::Joint::setJointName(std::string joint_name)
{
    _joint_name = joint_name;
}

void XBot::Joint::setJoitId(int joint_id)
{
    _joint_id = joint_id;
}



double XBot::Joint::getLinkPos()
{
    return _link_pos;
}

double XBot::Joint::getMotorPos()
{
    return _motor_pos;
}

double XBot::Joint::getLinkVel()
{
    return _link_vel;
}

double XBot::Joint::getMotorVel()
{
    return _motor_vel;
}

double XBot::Joint::getEffort()
{
    return _effort;
}

double XBot::Joint::getTemperature()
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









