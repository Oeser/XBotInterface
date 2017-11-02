#include <gtest/gtest.h>
#include <XBotInterface/Utils.h>

TEST (testUtils, checkFilterDcGainDouble) { 
    
    XBot::Utils::SecondOrderFilter<double> filter;
    
    double ts = 0.01;
    double cutoff = 10;
    double omega = 2*3.1415*cutoff; // 10 Hz cutoff
    double eps = 1.0;
    
    filter.setOmega(omega);
    filter.setTimeStep(ts);
    filter.setDamping(eps);
    
    filter.reset(0.0);
    
    double time = 0;
    
    double input = 1.0;
    double output;
    
    while(time < 1.0/cutoff*10){
        output = filter.process(input);
        ASSERT_EQ( output, filter.getOutput());
        time += ts;
    }
    
    EXPECT_NEAR(filter.getOutput(), 1.0, 1e-3);
    
    
    filter.reset(2.0);
    time = 0;
    
    while(time < 1.0/cutoff*10){
        output = filter.process(input);
        ASSERT_EQ( output, filter.getOutput());
        time += ts;
    }
    
    EXPECT_NEAR(filter.getOutput(), 1.0, 1e-3);
    
}
 
 
TEST (testUtils, checkFilterDcGainEigen) { 
    
    XBot::Utils::SecondOrderFilter<Eigen::Vector3d> filter;
    
    double ts = 0.01;
    double cutoff = 10;
    double omega = 2*3.1415*cutoff; // 10 Hz cutoff
    double eps = 1.0;
    
    filter.setOmega(omega);
    filter.setTimeStep(ts);
    filter.setDamping(eps);
    
    filter.reset(Eigen::Vector3d::Zero());
    
    double time = 0;
    
    Eigen::Vector3d input(1,1,1);
    Eigen::Vector3d output;
    
    while(time < 1.0/cutoff*10){
        output = filter.process(input);
        ASSERT_TRUE( (output - filter.getOutput()).norm() == 0);
        time += ts;
    }
    
    EXPECT_NEAR((filter.getOutput() - Eigen::Vector3d(1,1,1)).norm(), 0.0, 1e-3);
    
    
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}