#include <XBotInterface/RtLog.hpp>
#include <Eigen/Dense>

#include <unistd.h>


int main(int argc, char **argv){

    using namespace XBot::experimental;
    
    Logger logger;
    
    
    logger.info() << "babcabc" << logger.endl();
    
    logger.error() << "cabcabc" << logger.endl();
    
    logger.warning() << "cabcabc" << logger.endl();
    
    logger.success() << "cabcabc" << logger.endl();
    

}
