#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Logger.hpp>

int main(int argc, char **argv){

    using XBot::Sync;

    auto& logger = *XBot::ConsoleLogger::getLogger("home_logger");

    std::string path_to_config_file(argv[1]); // from command line

    XBot::RobotInterface& robot = *XBot::RobotInterface::getRobot(path_to_config_file);
    XBot::ModelInterface& model = *XBot::ModelInterface::getModel(path_to_config_file);

    Eigen::VectorXd qhome;
    if(!robot.getRobotState("home", qhome)){
        return -1;
    }

    logger.info() << "q homing : " << qhome << logger.endl();

    Eigen::VectorXd qinitial;

    robot.sense();
    robot.getJointPosition(qinitial);


//     for(auto pair : qhome) std::cout << pair.first << ": " << pair.second << std::endl;

    double alpha = 0;



    while(robot.isRunning()){

        alpha += 0.001;
        alpha = alpha > 1 ? 1 : alpha;

        robot.sense();
        robot.setPositionReference(qinitial+alpha*(qhome-qinitial));


        logger.info() << "q ref : " << qinitial+alpha*(qhome-qinitial) << logger.endl();

        robot.move();

        usleep(10000);

    }

    return 0;

}
