#include <XBotInterface/RobotInterface.h>


int main(int argc, char **argv){
 
    std::string path_to_config_file(argv[1]); // from command line

    XBot::RobotInterface& robot = *XBot::RobotInterface::getRobot(path_to_config_file);
    XBot::ModelInterface& model = *XBot::ModelInterface::getModel(path_to_config_file);
    
    XBot::ForceTorqueMap ftmap = robot.getForceTorque();
    XBot::ImuMap imumap = robot.getImu();
    
    while(robot.isRunning()){
     
        robot.sense();
        
        robot.print();
        
        for(auto& pair : ftmap) std::cout << *pair.second << std::endl;
        for(auto& pair : imumap) std::cout << *pair.second << std::endl;
        
        
        usleep(10000);
        
    }
    
    return 0;
    
}