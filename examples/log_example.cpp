#include <XBotInterface/Logger.hpp>
#include <Eigen/Dense>

#include <unistd.h>

#define LOG_ITERATION 40e3
#define LOG_PERIOD_MICRO 1e4

int main(int argc, char **argv){

    auto& file_logger = *XBot::FileLogger::getLogger("file_logger", "/tmp/test_file_logger");
    auto& console_logger = *XBot::ConsoleLogger::getLogger();

    auto& mfile_logger = *XBot::MatLogger::getLogger("mfile_logger", "/tmp/test_mfile_logger.m");

    Eigen::VectorXd q;
    Eigen::MatrixXd A;


    q.setConstant(5,0.123456789);
    A.setConstant(6,6,0.123456789);

    int i = 0;
    while(i < LOG_ITERATION) {

        file_logger.info() << "Vector q is :" << q.transpose() << file_logger.endl();
        console_logger.warning() << "Vector q is :" << q.transpose() << console_logger.endl();
        console_logger.error() << "Vector q is :" << q.transpose() << console_logger.endl();

        mfile_logger.add("q_v", q);
        mfile_logger.add("A_v", A);
        mfile_logger.add("scalar", double(i)/1.2345);

//         usleep(LOG_PERIOD_MICRO);
        i++;
    }

    return 0;

}
