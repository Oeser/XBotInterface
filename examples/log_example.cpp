#include <XBotInterface/Logger.hpp>
#include <Eigen/Dense>

#include <unistd.h>

#define LOG_ITERATION 4e1
#define LOG_PERIOD_MICRO 1e5

int main(int argc, char **argv){

    auto& file_logger = *XBot::FileLogger::getLogger("file_logger", "/tmp/test_file_logger");
    auto& console_logger = *XBot::ConsoleLogger::getLogger();

    auto& mfile_logger = *XBot::MatLogger::getLogger("/tmp/test_mfile_logger.mat");

    Eigen::VectorXd q;
    Eigen::MatrixXd A;


    mfile_logger.createVectorVariable("q_v", 5, 1, 30);
    mfile_logger.createMatrixVariable("A_v", 6, 6, 1, 30);

    q.setConstant(5,0.123456789);
    A.setConstant(6,6,0.123456789);

    int i = 0;
    while(i < LOG_ITERATION) {

        file_logger.info() << "Vector q is :" << q.transpose() << file_logger.endl();
        console_logger.warning() << "Vector q is :" << q.transpose() << console_logger.endl();
//         console_logger.error() << "Vector q is :" << q.transpose() << console_logger.endl();

        mfile_logger.add("q_v", q*std::sin(i/100.0));
        mfile_logger.add("A_v", A*i);
//         mfile_logger.add("scalar", double(i)/1.2345);

//         usleep(LOG_PERIOD_MICRO);
        i++;
    }

    return 0;

}
