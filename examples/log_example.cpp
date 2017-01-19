#include <XBotInterface/Logger.hpp>
#include <Eigen/Dense>

#include <unistd.h>

#define LOG_ITERATION 1e3
#define LOG_PERIOD_MICRO 1e4

int main(int argc, char **argv){

    auto& file_logger = *XBot::FileLogger::getLogger("file_logger", "/tmp/test_file_logger.txt");
    auto& console_logger = *XBot::ConsoleLogger::getLogger();
    auto& mfile_logger = *XBot::MatLogger::getLogger("mfile_logger", "/tmp/test_mfile_logger.txt");
    Eigen::VectorXd q;
    Eigen::MatrixXd A;


    q.setConstant(5,0.123456789);
    A.setConstant(6,6,0.123456789);

    int i = 0;
    while(i < LOG_ITERATION) {

        file_logger.info() << q.transpose() << 12 << "test1 " << __func__  << q.transpose() << "test2 " << file_logger.endl();
        console_logger.warning() << "warn1 " << __func__  << q.transpose() << " warn2 " << console_logger.endl();
        console_logger.error() << "error  " << __func__  << q.transpose() << " --- " << console_logger.endl();

        mfile_logger.add("q_v", q);
        mfile_logger.add("A_v", A);
        mfile_logger.add("scalar", double(i)/1.2345);

        usleep(LOG_PERIOD_MICRO);
        i++;
    }

    return 0;

}
