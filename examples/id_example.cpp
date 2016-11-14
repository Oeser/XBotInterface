#include <XBotInterface/RobotInterface.h>

double alpha(double tau){ return 0.5*(1-std::cos(tau)); }
double dalpha(double tau){ return 0.5*std::sin(tau); }
double ddalpha(double tau){ return 0.5*std::cos(tau); }

int main(int argc, char **argv){
 
    std::string path_to_config_file(argv[1]); // from command line

    XBot::RobotInterface& robot = *XBot::RobotInterface::getRobot(path_to_config_file, argc, argv);
    XBot::ModelInterface& model = *XBot::ModelInterface::getModel(path_to_config_file);
    
    
    Eigen::VectorXd q_homing;
    robot.getRobotState("home", q_homing);
    
    robot.setPositionReference(q_homing);
    robot.move();
    
    sleep(2);
    
    Eigen::VectorXd qmin, qmax, q_target;
    robot.getJointLimits(qmin, qmax);
    
    q_target = qmin;
    
    srand(robot.getTime());
    
    for( int i = 0; i < qmax.size(); i++ ){
        double rr = rand()/double(RAND_MAX);
        q_target(i) = qmin(i) + rr * (qmax(i)-qmin(i));
    }
    
//     q_target = q_homing*1.5;
    
    
    Eigen::VectorXd q, qref, qdot, qdotref, qddotref, tau, tauref;
    Eigen::MatrixXd B;
    
    double time, t0 = robot.getTime();
    double period = 1;
    double omega = 1;
    
    
    
    robot.sense();
    robot.getJointPosition(q);
    robot.model().getJointPosition(qref);
    
    
    std::cout << q - qref << std::endl;
    
    while(robot.isRunning()){
        
        time = robot.getTime() - t0;
        
        robot.sense();
        robot.getJointPosition(q);
        robot.getJointVelocity(qdot);
        robot.getJointEffort(tau);
        
        qref = q_homing + alpha(time/period) * (q_target - q_homing);
        qdotref = dalpha(time/period)/period * (q_target - q_homing);
        qddotref = ddalpha(time/period)/(period*period) * (q_target - q_homing);
        
        robot.model().setJointAcceleration(qddotref);
        robot.model().update(true,true,true);
        robot.model().computeInverseDynamics(tauref);
        
        robot.setEffortReference(tauref);

        
        robot.setPositionReference(qref);
        robot.move();
        robot.printTracking();
        
    }
    
    
    
    
    
}