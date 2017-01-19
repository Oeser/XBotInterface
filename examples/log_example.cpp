#include <XBotInterface/Logger.hpp>
#include <Eigen/Dense>

#include <unistd.h>

#define LOG_ITERATION 1e3
#define LOG_PERIOD_MICRO 1e4

int main(int argc, char **argv){

    XBot::SPDMatLogger logger("m_log", "/tmp/rt_test.m");
    Eigen::VectorXd q;
    Eigen::MatrixXd A;
    

    q.setConstant(5,0.123456789);
    A.setConstant(6,6,0.123456789);

    int i = 0;
    while(i < LOG_ITERATION) {
        
//         logger.info().log() << q.transpose() << 12 << "test1 " << __func__  << q.transpose() << "test2 " << logger.endl();
//         logger.warning().log() << "warn1 " << __func__  << q.transpose() << " warn2 " << logger.endl();
//         logger.error().log() << "error  " << __func__  << q.transpose() << " --- " << logger.endl();
        
        logger.add("q_v", q);
        logger.add("A_v", A);
        logger.add("scalar", double(i)/1.2345);
        
        usleep(LOG_PERIOD_MICRO);
        i++;
    }
    
    return 0;

}
