#include <XBotInterface/RobotInterface.h>

int main(int argc, char **argv){
    
    using namespace XBot;
    
    std::string path_to_config_file(argv[1]);
    
    /* This example will show you the usage of the XBotInterface library,
     * which provides  a framework-independent abstraction layer
     * for any robotic platform. First, the tutorial will go through the
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
    
    /* Now that our RobotInterface is up to date, let us read joint states.
     * A feature of the XBotInterface library is that it provides a nice
     * organization of a complex robot as a collection of kinematic chains.
     * For example, a humanoid robot joint state is more conveniently described
     * by the set of its chains states, e.g. left_arm, right_arm, left_leg, 
     * right_leg, torso and neck, as opposed to a huge configuration vector.
     * In XBotInterface, a chain in accessed by the chain() method, which returns
     * a KinematicChain by reference. For example, let's print the chain state via the << operator. 
     */ 
    
    std::cout << robot.chain("left_arm") << std::endl;
    
    // operator() is a short-hand for chain():
    
    std::cout << robot("right_arm") << std::endl;
    
    /* We also support two kinds of "standard kinematic chains", i.e. legs
     * and arms. This means that you can quickly access the i-th arm/leg by
     * writing:
     */
    
    int arm_id = 0;
    int leg_id = 1;
    
    std::cout << robot.arm(arm_id) << std::endl;
    std::cout << robot.leg(leg_id) << std::endl;
    
    /* You
     */
    

    
    return 0;
    
}