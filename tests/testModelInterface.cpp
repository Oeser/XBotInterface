#include <XBotInterface/ModelInterface.h>
#include <gtest/gtest.h>
#include <cstdlib>
#include <time.h>

class testModelInterface : public testing::Test {
  
protected:
    
virtual void SetUp(){
    
    std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
    std::string relative_path = "/external/XBotInterface/configs/config_walkman.yaml";
    model_ptr = XBot::ModelInterface::getModel(robotology_root+relative_path);
    path_to_cfg = robotology_root + relative_path;

}

virtual void TearDown();

XBot::ModelInterface::Ptr model_ptr;
std::string path_to_cfg;
    
};


int main ( int argc, char **argv )
{
     testing::InitGoogleTest ( &argc, argv );
     return RUN_ALL_TESTS();
}
