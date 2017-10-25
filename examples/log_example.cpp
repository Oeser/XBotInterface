#include <XBotInterface/RtLog.hpp>
#include <Eigen/Dense>

#include <unistd.h>


int main(int argc, char **argv){

    using namespace XBot::experimental;
    
    Logger::SetVerbosityLevel(Logger::Severity::MID);
    
    Logger::info() << "babcabc" << Logger::endl();
    
    Logger::error() << "cabcabc" << Logger::endl();
    
    Logger::warning(Logger::Severity::LOW) << "cabcabc" << Logger::endl(); // not printed
    
    Logger::success() << "cabcabc" << Logger::endl();
    
    

}
