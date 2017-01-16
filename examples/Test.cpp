#include <XBotInterface/Test.h>
#include <Eigen/Dense>

int main(int argc, char **argv){

    LoggerTest test;
    Eigen::VectorXd q;

    q.setConstant(5,5);




    test.info().log() << q.transpose() << 12 << "atatata" << __func__  << q.transpose() << "qhwahsd" << test.info().endl();

    test.info().log() << "atatata" << __func__  << q.transpose() << "qhwahsd" << test.info().endl();

    return 0;

}