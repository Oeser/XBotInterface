#include "XBotInterface/RobotInterface.h"


int main(int argc, char **argv){
	
	std::string path_to_cfg("/home/alaurenzi/Code/robotology-superbuild/external/XBotInterface/IXBotInterface/configs/config_centauro.yaml");
	
	std::string path_to_ros_cfg("/home/alaurenzi/Code/robotology-superbuild/external/XBotInterface/RobotInterfaceROS/configs/ROSconfig.yaml");
	
	XBot::RobotInterface& robot = *XBot::RobotInterface::getRobot(path_to_cfg, argc, argv);
	
	if(!robot.init(path_to_ros_cfg)){
		std::cerr << "ERROR Robot interface could not be initialized!" << std::endl;
	}
	
	robot.sense(false);
	
	Eigen::VectorXd qarmleft, qarmright;
	std::map<std::string, double> maparmleft, maparmright;
	robot.arm(0).getLinkPos(qarmleft);
	robot.arm(1).getLinkPos(qarmright);
	robot.arm(0).getLinkPos(maparmleft);
	robot.arm(1).getLinkPos(maparmright);
	
	std::cout << "Left arm: \n" << qarmleft << std::endl;
	std::cout << "Right arm: \n" << qarmright << std::endl;
	for(const auto& q : maparmleft) std::cout << q.first << " " << q.second << std::endl;
	for(const auto& q : maparmright) std::cout << q.first << " " << q.second << std::endl;
	
	robot.arm(0).setPosRef(qarmleft/2);
	robot.arm(1).setPosRef(qarmright);
	
	while(true) robot.move(false);
	
	std::cout << "Left arm: \n" << qarmleft << std::endl;

	
	return 0;
}

