#include <XBotInterface/RtLog.hpp>
#include <Eigen/Dense>

#include <unistd.h>


int main(int argc, char **argv){

    using namespace XBot;
    
    Eigen::MatrixXd q(20, 20);
    
    Logger::SetVerbosityLevel(Logger::Severity::MID);
    
    Logger::error("My error like printf: %d %f %s\n", 3, 2.73, "mamma");
    
    Logger::info() << "babcabc" << Logger::endl(); // not printed
    
    Logger::error() << "cabcabc" << Logger::endl();
    
    Logger::warning() << "cabcabc" << Logger::endl(); 
    
    Logger::success(Logger::Severity::MID) << "cabcabc" << Logger::endl(); // printed
    
    Logger::info(Logger::Severity::MID) << q.transpose().format(Eigen::IOFormat(4)) << Logger::endl(); // printed
    
    Logger::error() << "error line 1\n";
    Logger::log() << "error line 2\n";
    Logger::log() << "error line 3" << Logger::endl();
    
    Logger::error() << "Will this be displayed?" << std::endl;
    Logger::error() << "Then Will this be displayed?" << Logger::endl();
    
    
    
    LoggerClass logger("log_example");
    
    logger.warning() << "my warn" << logger.endl();
    logger.info("My int is %d, my double is %f\n", 1, 3.14);

}
