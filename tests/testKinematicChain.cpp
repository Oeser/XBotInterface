#include <gtest/gtest.h>
#include <XBotInterface/XBotInterface.h>
#include <cstdlib>
#include <sys/time.h>




class testKinematicChain: public testing::Test {
    

protected:

     testKinematicChain(){}

     virtual ~testKinematicChain(){}
     virtual void SetUp() {
         
         std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
         std::string relative_path = "/external/XBotInterface/configs/config_walkman.yaml";
         xbint.init(robotology_root+relative_path);
         path_to_cfg = robotology_root + relative_path;
     }

     virtual void TearDown() {
     }
     
     XBot::XBotInterface xbint;
     std::string path_to_cfg;
     
};

TEST_F(testKinematicChain, checkCopyConstructor){
                
    Eigen::VectorXd random_x, random_x_copy;
    struct timeval t1;
    
    // i want to set random values on one chain and copy to another.
    for(const auto& c1 : xbint.getChainMap()) {
        
        // changing the ssed
        gettimeofday(&t1, NULL);
        srand(t1.tv_usec * t1.tv_sec);
        
        // set random values in some field of the chain
        XBot::KinematicChain current_chain = *c1.second;
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setDamping(random_x);
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setEffortReference(random_x);
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setJointPosition(random_x);
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setJointVelocity(random_x);
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setJointEffort(random_x);
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setPositionReference(random_x);
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setMotorPosition(random_x);
        
        // copy constructor
        XBot::KinematicChain copy_chain(current_chain);
        
        current_chain.getDamping(random_x);
        copy_chain.getDamping(random_x_copy);    
        EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
        
        current_chain.getJointEffort(random_x);
        copy_chain.getJointEffort(random_x_copy);
        EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
        
        current_chain.getJointEffort(random_x);
        copy_chain.getJointEffort(random_x_copy);
        EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
        
        current_chain.getJointPosition(random_x);
        copy_chain.getJointPosition(random_x_copy);
        EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
        
        current_chain.getEffortReference(random_x);
        copy_chain.getEffortReference(random_x_copy);
        EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
        
        current_chain.getPositionReference(random_x);
        copy_chain.getPositionReference(random_x_copy);
        EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
        
        current_chain.getMotorPosition(random_x);
        copy_chain.getMotorPosition(random_x_copy);
        EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
        
        
        // Modify the copy and check that the original is not modified
        
        random_x_copy.setRandom(copy_chain.getJointNum());
        copy_chain.setJointEffort(random_x_copy);
        current_chain.getJointEffort(random_x);
        
        EXPECT_FALSE( (random_x.array() == random_x_copy.array()).all() );
    }              
}

TEST_F(testKinematicChain, checkCopyAssignment) {
    
    Eigen::VectorXd random_x, random_x_copy;
    struct timeval t1;
    
    
    // i want to set random values on one chain and copy to all the others.
    for(const auto& c1 : xbint.getChainMap()) {
        
        // changing the ssed
        gettimeofday(&t1, NULL);
        srand(t1.tv_usec * t1.tv_sec);
        
        // set random values in some field of the chain
        XBot::KinematicChain current_chain = *c1.second;
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setDamping(random_x);
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setEffortReference(random_x);
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setJointPosition(random_x);
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setJointVelocity(random_x);
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setJointEffort(random_x);
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setPositionReference(random_x);
        
        random_x.setRandom(current_chain.getJointNum());
        current_chain.setMotorPosition(random_x);
        
        // iterate over the other chains
        for(const auto& c2 : xbint.getChainMap()) {
            if(c2.first != current_chain.getChainName()) {
                XBot::KinematicChain copy_chain = *c2.second;
                // operator = 
                copy_chain = current_chain;
                
                current_chain.getDamping(random_x);
                copy_chain.getDamping(random_x_copy);    
                EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
                
                current_chain.getJointEffort(random_x);
                copy_chain.getJointEffort(random_x_copy);
                EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
                
                current_chain.getJointEffort(random_x);
                copy_chain.getJointEffort(random_x_copy);
                EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
                
                current_chain.getJointPosition(random_x);
                copy_chain.getJointPosition(random_x_copy);
                EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
                
                current_chain.getEffortReference(random_x);
                copy_chain.getEffortReference(random_x_copy);
                EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
                
                current_chain.getPositionReference(random_x);
                copy_chain.getPositionReference(random_x_copy);
                EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
                
                current_chain.getMotorPosition(random_x);
                copy_chain.getMotorPosition(random_x_copy);
                EXPECT_TRUE( (random_x.array() == random_x_copy.array()).all() );
                
                
                // Modify the copy and check that the original is not modified
                
                random_x_copy.setRandom(copy_chain.getJointNum());
                copy_chain.setJointEffort(random_x_copy);
                current_chain.getJointEffort(random_x);
                
                EXPECT_FALSE( (random_x.array() == random_x_copy.array()).all() );
            }
        }
    }
}



TEST_F(testKinematicChain, checkJointGetters){
            

    
    for(const auto& c : xbint.getChainMap()) {
        
        Eigen::VectorXd x;
        x.setRandom( c.second->getJointNum() );
        c.second->setJointPosition(x);
        std::map<int, double> x_map;
        c.second->getJointPosition(x_map);
        
        for( int i = 0; i < c.second->getJointNum(); i++ ){
            XBot::Joint j = *c.second->getJoint(i);
            EXPECT_TRUE( j.getJointName() == xbint.getJointByName(j.getJointName())->getJointName() );
            
            int id = xbint.getJointByName(j.getJointName())->getJointId();
            EXPECT_TRUE( id == xbint.getJointByID(id)->getJointId() );
            
            ASSERT_TRUE( x_map.count(id) == 1 );
            EXPECT_TRUE( x_map.at(id) == x(i) );
        
        }
    }
    
    
}

#define GETTER_SETTER_TEST(chain, getFieldName, setFieldName) \
chain. setFieldName (x); \
\
y.setZero(y.size()); \
y_id_map.clear(); \
y_name_map.clear(); \
chain. getFieldName (y); \
chain. getFieldName (y_id_map); \
chain. getFieldName (y_name_map); \
\
EXPECT_TRUE( (x.array() == y.array()).all() ); \
EXPECT_TRUE( x_id_map == y_id_map ); \
EXPECT_TRUE( x_name_map == y_name_map ); \
\
chain. setFieldName (x_id_map); \
\
y.setZero(y.size()); \
y_id_map.clear(); \
y_name_map.clear(); \
chain. getFieldName (y); \
chain. getFieldName (y_id_map); \
chain. getFieldName (y_name_map); \
\
EXPECT_TRUE( (x.array() == y.array()).all() ); \
EXPECT_TRUE( x_id_map == y_id_map ); \
EXPECT_TRUE( x_name_map == y_name_map ); \
\
chain. setFieldName (x_name_map); \
\
y.setZero(y.size()); \
y_id_map.clear(); \
y_name_map.clear(); \
chain. getFieldName (y); \
chain. getFieldName (y_id_map); \
chain. getFieldName (y_name_map); \
\
EXPECT_TRUE( (x.array() == y.array()).all() ); \
EXPECT_TRUE( x_id_map == y_id_map ); \
EXPECT_TRUE( x_name_map == y_name_map ); 

    


TEST_F(testKinematicChain, checkGettersSetters){
 
    
    Eigen::VectorXd x, y;
    std::map<int, double> x_id_map, y_id_map;
    std::map<std::string, double> x_name_map, y_name_map;
    
    for(const auto& c : xbint.getChainMap()) {
        x_id_map.clear();
        y_id_map.clear();
        x_name_map.clear();
        y_name_map.clear();
        
        XBot::KinematicChain& chain = *c.second;
        x.setRandom( chain.getJointNum() );
        for(int i = 0; i < x.size(); i++){
            x_id_map[chain.getJointId(i)] = x(i);
            x_name_map[chain.getJointName(i)] = x(i);
        }
        GETTER_SETTER_TEST(chain, getDamping, setDamping)
        GETTER_SETTER_TEST(chain, getStiffness, setStiffness)
        GETTER_SETTER_TEST(chain, getJointPosition, setJointPosition)
        GETTER_SETTER_TEST(chain, getPositionReference, setPositionReference)
        GETTER_SETTER_TEST(chain, getJointVelocity, setJointVelocity)
        GETTER_SETTER_TEST(chain, getVelocityReference, setVelocityReference)
        GETTER_SETTER_TEST(chain, getJointEffort, setJointEffort)
        GETTER_SETTER_TEST(chain, getEffortReference, setEffortReference)
        GETTER_SETTER_TEST(chain, getMotorPosition, setMotorPosition)
        GETTER_SETTER_TEST(chain, getMotorVelocity, setMotorVelocity)
        GETTER_SETTER_TEST(chain, getTemperature, setTemperature)
    }
    
    
    
    
    
}


TEST_F(testKinematicChain, checkSyncFrom){
    
    Eigen::VectorXd x, y;
    struct timeval t1;
    
    XBot::XBotInterface xbint2 = xbint;
    
    for(const auto& c1 : xbint.getChainMap()) {
        
        // changing the ssed
        gettimeofday(&t1, NULL);
        srand(t1.tv_usec * t1.tv_sec);
        
        // set random values in some field of the chain
        XBot::KinematicChain current_chain = *c1.second;
        
         for(const auto& c2 : xbint2.getChainMap()) {
            if(c2.first == current_chain.getChainName()) {
                XBot::KinematicChain chain = *c2.second;
            
                x.setRandom(chain.getJointNum());
                current_chain.setDamping(x);
                x.setRandom(chain.getJointNum());
                current_chain.setStiffness(x);
                x.setRandom(chain.getJointNum());
                current_chain.setJointEffort(x);
                x.setRandom(chain.getJointNum());
                current_chain.setJointPosition(x);
                x.setRandom(chain.getJointNum());
                current_chain.setJointVelocity(x);
                x.setRandom(chain.getJointNum());
                current_chain.setMotorPosition(x);
                x.setRandom(chain.getJointNum());
                current_chain.setMotorVelocity(x);
                x.setRandom(chain.getJointNum());
                current_chain.setPositionReference(x);
                x.setRandom(chain.getJointNum());
                current_chain.setVelocityReference(x);
                x.setRandom(chain.getJointNum());
                current_chain.setEffortReference(x);
                
                chain.syncFrom(current_chain);
                
                Eigen::VectorXd y;
                
                chain.getDamping(x);
                current_chain.getDamping(y);
                EXPECT_TRUE( (x.array() == y.array()).all() );
                
                chain.getStiffness(x);
                current_chain.getStiffness(y);
                EXPECT_TRUE( (x.array() == y.array()).all() );
                
                chain.getJointPosition(x);
                current_chain.getJointPosition(y);
                EXPECT_TRUE( (x.array() == y.array()).all() );
                
                chain.getJointVelocity(x);
                current_chain.getJointVelocity(y);
                EXPECT_TRUE( (x.array() == y.array()).all() );
                
                chain.getJointEffort(x);
                current_chain.getJointEffort(y);
                EXPECT_TRUE( (x.array() == y.array()).all() );
                
                chain.getMotorPosition(x);
                current_chain.getMotorPosition(y);
                EXPECT_TRUE( (x.array() == y.array()).all() );
                
                chain.getMotorVelocity(x);
                current_chain.getMotorVelocity(y);
                EXPECT_TRUE( (x.array() == y.array()).all() );
                
                chain.getTemperature(x);
                current_chain.getTemperature(y);
                EXPECT_TRUE( (x.array() == y.array()).all() );
                
                chain.getPositionReference(x);
                current_chain.getPositionReference(y);
                EXPECT_TRUE( (x.array() == y.array()).all() );
                
                chain.getVelocityReference(x);
                current_chain.getVelocityReference(y);
                EXPECT_TRUE( (x.array() == y.array()).all() );
                
                chain.getEffortReference(x);
                current_chain.getEffortReference(y);
                EXPECT_TRUE( (x.array() == y.array()).all() );
            
            }
            
        }
        
    }

   
    
}

TEST_F(testKinematicChain, checkJointLimits){
    
    struct timeval t1;

    
    for(const auto& c : xbint.getChainMap()) {
        
        Eigen::VectorXd q_in_range, tau_in_range, qdot_in_range;
        Eigen::VectorXd q_out_range, tau_out_range, qdot_out_range;
        std::vector<std::string> bad_joints, expected_bad_joints;
        Eigen::VectorXd qmin, qmax, qdotmax, taumax;
        XBot::Joint j;

    
        // changing the ssed
        gettimeofday(&t1, NULL);
        srand(t1.tv_usec * t1.tv_sec);
        
        // set random values in some field of the chain
        XBot::KinematicChain current_chain = *c.second;
                
        Eigen::VectorXd alpha( current_chain.getJointNum() );
        Eigen::VectorXd beta(alpha);
        
        current_chain.getJointLimits(qmin, qmax);
        current_chain.getEffortLimits(taumax);
        current_chain.getVelocityLimits(qdotmax);
        
        for( int i = 0; i < current_chain.getJointNum(); i++ ){
            j = *current_chain.getJoint(i);
            
            EXPECT_EQ(xbint.getUrdf().getJoint(j.getJointName())->limits->lower, qmin(i));
            EXPECT_EQ(xbint.getUrdf().getJoint(j.getJointName())->limits->upper, qmax(i));
            EXPECT_EQ(xbint.getUrdf().getJoint(j.getJointName())->limits->effort, taumax(i));
            EXPECT_EQ(xbint.getUrdf().getJoint(j.getJointName())->limits->velocity, qdotmax(i));
        }
        

        int trials_number = 100;
        
        for(int trial = 0; trial < trials_number; trial++){
            
            expected_bad_joints.clear();
            for( int i = 0; i < current_chain.getJointNum(); i++ ){
                alpha(i) = rand()/double(RAND_MAX);
                beta(i) = 2.5*(alpha(i) - 0.5);
                // force at least one beta outside [0,1]
                if(i == 0) {
                    beta(i) = 1.1;
                }
                if(beta(i) < 0 || beta(i) > 1){
                    expected_bad_joints.push_back(current_chain.getJoint(i)->getJointName());
                }
            }
            
            std::cout << "BAD JOINT NUM: " << expected_bad_joints.size() << std::endl;
            
            
            q_in_range = qmin.array() + alpha.array()*(qmax-qmin).array();
            q_out_range = qmin.array() + beta.array()*(qmax-qmin).array();
            tau_in_range = -taumax.array() + alpha.array()*(2*taumax).array();
            tau_out_range = -taumax.array() + beta.array()*(2*taumax).array();
            qdot_in_range = -qdotmax.array() + alpha.array()*(2*qdotmax).array();
            qdot_out_range = -qdotmax.array() + beta.array()*(2*qdotmax).array();
            
            EXPECT_TRUE( (q_in_range.array() >= qmin.array()).all() && (q_in_range.array() <= qmax.array()).all() );
            
            bad_joints.clear();
            EXPECT_TRUE( current_chain.checkJointLimits(q_in_range, bad_joints) );
            EXPECT_TRUE( current_chain.checkJointLimits(q_in_range) );
            EXPECT_TRUE( bad_joints.size() == 0 );
            EXPECT_FALSE( current_chain.checkJointLimits(q_out_range, bad_joints) );
            EXPECT_FALSE( current_chain.checkJointLimits(q_out_range) );
            EXPECT_TRUE( bad_joints == expected_bad_joints );
            
            bad_joints.clear();
            EXPECT_TRUE( current_chain.checkEffortLimits(tau_in_range, bad_joints) );
            EXPECT_TRUE( current_chain.checkEffortLimits(tau_in_range) );
            EXPECT_TRUE( bad_joints.size() == 0 );
            EXPECT_FALSE( current_chain.checkEffortLimits(tau_out_range, bad_joints) );
            EXPECT_FALSE( current_chain.checkEffortLimits(tau_out_range) );
            EXPECT_TRUE( bad_joints == expected_bad_joints );
            
            bad_joints.clear();
            EXPECT_TRUE( current_chain.checkVelocityLimits(qdot_in_range, bad_joints) );
            EXPECT_TRUE( current_chain.checkVelocityLimits(qdot_in_range) );
            EXPECT_TRUE( bad_joints.size() == 0 );
            EXPECT_FALSE( current_chain.checkVelocityLimits(qdot_out_range, bad_joints) );
            EXPECT_FALSE( current_chain.checkVelocityLimits(qdot_out_range) );
            EXPECT_TRUE( bad_joints == expected_bad_joints );
        
        }
    }
    
}

int main ( int argc, char **argv )
{
     testing::InitGoogleTest ( &argc, argv );
     return RUN_ALL_TESTS();
}





