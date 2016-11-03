#include <gtest/gtest.h>
#include <XBotInterface/IXBotInterface.h>
#include <cstdlib>
#include <time.h>

namespace {

class testJoint: public ::testing::Test {
    
    
    
protected:

    
    
     testJoint(){
         
         
     }

     virtual ~testJoint() {
     }

     virtual void SetUp() {
         
         std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
         std::string relative_path = "/external/XBotInterface/configs/config_centauro.yaml";
         xbint.init(robotology_root+relative_path);
     }

     virtual void TearDown() {
     }
     
     XBot::IXBotInterface xbint;
};

// Test joint limits
TEST_F ( testJoint, checkJointLimits )
{
    
    srand(time(NULL));
    
    for( const auto& j : xbint.getEnabledJointNames() ){
        
        
        double qmin = 0, qmax = 0;
        qmax = xbint.getUrdf().getJoint(j)->limits->upper;
        qmin = xbint.getUrdf().getJoint(j)->limits->lower;
        
        double q_out_of_range_1 = qmax + 0.1;
        double q_out_of_range_2 = qmin - 0.1;
        
        double q_in_range = qmin + rand()/double(RAND_MAX)*(qmax-qmin);
        
        EXPECT_TRUE(xbint.getJointByName(j)->checkJointLimits(q_in_range));
        EXPECT_FALSE(xbint.getJointByName(j)->checkJointLimits(q_out_of_range_1));
        EXPECT_FALSE(xbint.getJointByName(j)->checkJointLimits(q_out_of_range_2));
        
        if( xbint.getJointByName(j)->getJointId() == 11 ) {
            std::cout << "q min : " << qmin << std::endl;
            std::cout << "q max : " << qmax << std::endl;
            std::cout << "q random : " << q_in_range << std::endl;
        }
    }

}

TEST_F(testJoint, checkVelocityLimits){
 
    srand(time(NULL));
    
    for( const auto& j : xbint.getEnabledJointNames() ){
        
        
        double qdotmax;
        qdotmax = xbint.getUrdf().getJoint(j)->limits->velocity;
        
        double qdot_out_of_range_1 = qdotmax + 0.1;
        double qdot_out_of_range_2 = -qdotmax - 0.1;
        
        double qdot_in_range = -qdotmax + rand()/double(RAND_MAX)*(2*qdotmax);
        
        EXPECT_TRUE(xbint.getJointByName(j)->checkVelocityLimit(qdot_in_range));
        EXPECT_FALSE(xbint.getJointByName(j)->checkVelocityLimit(qdot_out_of_range_1));
        EXPECT_FALSE(xbint.getJointByName(j)->checkVelocityLimit(qdot_out_of_range_2));
        
        if( xbint.getJointByName(j)->getJointId() == 11 ) {
            std::cout << "qdot max : " << qdotmax << std::endl;
            std::cout << "qdot random : " << qdot_in_range << std::endl;
        }
    }
    
}


TEST_F(testJoint, checkEffortLimits){
 
    srand(time(NULL));
    
    for( const auto& j : xbint.getEnabledJointNames() ){
        
        
        double taumax;
        taumax = xbint.getUrdf().getJoint(j)->limits->effort;
        
        double tau_out_of_range_1 = taumax + 0.1;
        double tau_out_of_range_2 = -taumax - 0.1;
        
        double tau_in_range = -taumax + rand()/double(RAND_MAX)*(2*taumax);
        
        EXPECT_TRUE(xbint.getJointByName(j)->checkEffortLimit(tau_in_range));
        EXPECT_FALSE(xbint.getJointByName(j)->checkEffortLimit(tau_out_of_range_1));
        EXPECT_FALSE(xbint.getJointByName(j)->checkEffortLimit(tau_out_of_range_2));
        
        if( xbint.getJointByName(j)->getJointId() == 11 ) {
            std::cout << "tau max : " << taumax << std::endl;
            std::cout << "tau random : " << tau_in_range << std::endl;
        }
    }
    
}

} //namespace

int main ( int argc, char **argv )
{
     ::testing::InitGoogleTest ( &argc, argv );
     return RUN_ALL_TESTS();
}
