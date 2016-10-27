#include <XBotInterface/RobotInterface.h>

int main(int argc, char **argv){
    
    using namespace XBot;
    
    std::string path_to_config_file(argv[1]);
    
    /* This example will show you the usage of the XBotInterface library,
     * which provides you with a framework-independent abstraction layer
     * for aby robotic platform. First, the tutorial will go through the
     * basic syntax of the library, then a complete control loop example
     * taken from the domain of inverse kinematics will be shown.
     * 
     * As a first step, we need to instantiate a RobotInterface, which 
     * provides us functionalities to control a robot (like e.g. reading
     * encoders, force-torques, setting references for controllers, ...).
     * To this aim we use the static method RobotInterface::getRobot() which
     * returns a pointer to the required object.
     */
    
    RobotInterface& robot = *RobotInterface::getRobot(path_to_config_file);
    
    /* We now want to update the robot internal state with the latest sensor
     * readings. This is done by a method called sense(). */
    
    robot.sense();
    
    /* Now that our RobotInterface is up to date, let us read 
    
    
    return 0;
    
}