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
         std::string relative_path = "/external/XBotInterface/configs/config_walkman.yaml";
         xbint.init(robotology_root+relative_path);
     }

     virtual void TearDown() {
     }
     
     XBot::IXBotInterface xbint;
};

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
        
        if( xbint.getJointByName(j)->getJointId() == 21 ) {
            std::cout << "q min : " << qmin << std::endl;
            std::cout << "q max : " << qmax << std::endl;
            std::cout << "q random : " << q_in_range << std::endl;
        }
    }

}



} //namespace

int main ( int argc, char **argv )
{
     ::testing::InitGoogleTest ( &argc, argv );
     return RUN_ALL_TESTS();
}
