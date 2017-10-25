#include <XBotInterface/RtLog.hpp>
#include <Eigen/Dense>

#include <unistd.h>


int main(int argc, char **argv){

    using namespace XBot::experimental;
    
    Logger::info() << "babcabc" << Logger::endl();
    
    Logger::error() << "cabcabc" << Logger::endl();
    
    Logger::warning() << "cabcabc" << Logger::endl();
    
    Logger::success() << "cabcabc" << Logger::endl();
    

}
