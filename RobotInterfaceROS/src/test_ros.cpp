#include "XBotInterface/RobotInterface.h"


int main(int argc, char **argv){
	
	std::string path_to_cfg("/home/alaurenzi/Code/robotology-superbuild/external/XBotInterface/IXBotInterface/configs/config_centauro.yaml");
	
	std::string path_to_ros_cfg("/home/alaurenzi/Code/robotology-superbuild/external/XBotInterface/RobotInterfaceROS/configs/ROSconfig.yaml");
	
	XBot::RobotInterface& robot = *XBot::RobotInterface::getRobot(path_to_cfg);
	
	if(!robot.init(path_to_ros_cfg, argc, argv)){
		std::cerr << "ERROR Robot interface could not be initialized!" << std::endl;
	}
	
	robot.sense(false);
	
	Eigen::VectorXd qarm;
	robot.arm(0).getLinkPos(qarm);
	
	std::cout << qarm << std::endl;

	
	return 0;
}

