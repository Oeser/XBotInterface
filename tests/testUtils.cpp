#include <gtest/gtest.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/RtLog.hpp>
#include <thread>
#include <functional>

using XBot::Logger;

namespace {

class testUtils: public ::testing::Test {
    
public:
    
    
    void test_mt_log_func(int th_id) {
        for(int i = 0; i < 5000; i++){
            Logger::info() << "Thread #" << th_id << " printing to logger!" << Logger::endl();
            Logger::error("Thread #%d printing to logger", th_id);
            usleep(1000);
        }
     }

protected:

     testUtils(){
         
         
     }

     virtual ~testUtils() {
     }

     virtual void SetUp() {
         
     }

     virtual void TearDown() {
     }
     
     
     
};



TEST_F (testUtils, checkFilterDcGainDouble) { 
    
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
 
 
TEST_F (testUtils, checkFilterDcGainEigen) { 
    
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



// TEST_F(testUtils, checkMtLogger){
//     
//     std::vector<std::thread> th;
//     
//     auto f = std::bind(&testUtils::test_mt_log_func, this, std::placeholders::_1);
//     
//     for(int i = 0; i < 5; i++){
//         th.push_back(std::thread(f, i+1));
//     }
//     
//     for(auto& t : th){
//         t.join();
//     }
// }



TEST_F( testUtils, checkLimitedDeque ){
 
    int N = 10;
    
    XBot::Utils::LimitedDeque<int> deque(N);
    
//     EXPECT_THROW( deque.back(), std::out_of_range );
    EXPECT_FALSE( deque.pop_back() );
    EXPECT_TRUE( deque.is_empty() );
    
    for(int i = 0; i < (N-1); i++){
        EXPECT_EQ(deque.size(), i);
        deque.push_back();
        EXPECT_EQ(deque.size(), i+1);
        deque.back() = i;
        EXPECT_FALSE(deque.is_full());
        EXPECT_FALSE(deque.is_empty());
    }
    
    deque.push_back();
    deque.back() = N-1;
    
    EXPECT_TRUE( deque.is_full() );
    EXPECT_EQ( deque.size(), deque.capacity() );
    EXPECT_FALSE( deque.is_empty() );
    
    for(int i = N-1; i >= 0; i--){
        std::cout << deque.back() << std::endl;
        EXPECT_EQ( deque.back(), i );
        EXPECT_TRUE( deque.pop_back() );
        EXPECT_EQ( deque.size(), i );
    }
    
    EXPECT_TRUE( deque.is_empty() );
    
    
    
    for(int i = 0; i < (1.5*N); i++){
        EXPECT_EQ(deque.size(), std::min(N, i));
        deque.push_back();
        EXPECT_EQ(deque.size(), std::min(N, i+1));
        deque.back() = i;
        std::cout << deque.back() << std::endl;
        EXPECT_FALSE(deque.is_empty());
    }
    
    EXPECT_EQ(deque.size(), N);
    
    for(int i = 0; i < N; i++){
        std::cout << "Iter " << i << std::endl;
        EXPECT_EQ(deque.back(), int(1.5*N)-1-i);
        std::cout << deque.back() << std::endl;
        EXPECT_EQ(deque.size(), N-i);
        EXPECT_TRUE(deque.pop_back());
        EXPECT_EQ(deque.size(), N-i-1);
    }
    
    EXPECT_TRUE(deque.is_empty());
    
    
    
    
    
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}