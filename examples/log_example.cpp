#include <XBotInterface/Logger.hpp>
#include <Eigen/Dense>

#include <unistd.h>

#define LOG_ITERATION 1e3
#define LOG_PERIOD_MICRO 1e4

int main(int argc, char **argv){

    XBot::Logger logger("test_logger");
    Eigen::VectorXd q;

    q.setConstant(5,5);

    int i = 0;
    while(i < LOG_ITERATION) {
        
        logger.info().log() << q.transpose() << 12 << "test1 " << __func__  << q.transpose() << "test2 " << logger.endl();
        logger.warning().log() << "warn1 " << __func__  << q.transpose() << " warn2 " << logger.endl();
        logger.error().log() << "error  " << __func__  << q.transpose() << " --- " << logger.endl();
        
        usleep(LOG_PERIOD_MICRO);
        i++;
    }
    
    return 0;

}