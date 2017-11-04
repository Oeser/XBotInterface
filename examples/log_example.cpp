#include <XBotInterface/RtLog.hpp>
#include <Eigen/Dense>

#include <unistd.h>


int main(int argc, char **argv){

    using namespace XBot;
    
    Eigen::MatrixXd q(20, 20);
    
    
    Logger::SetVerbosityLevel(Logger::Severity::MID);
    
    Logger::info() << "babcabc" << Logger::endl(); // not printed
    
    Logger::error() << "cabcabc" << Logger::endl();
    
    Logger::warning() << "cabcabc" << Logger::endl(); 
    
    Logger::success(Logger::Severity::MID) << "cabcabc" << Logger::endl(); // printed
    
    Logger::info(Logger::Severity::MID) << q.transpose().format(Eigen::IOFormat(4)) << Logger::endl(); // printed
    
    

}
