#include <gtest/gtest.h>
#include <XBotInterface/IXBotInterface.h>
#include <cstdlib>
#include <time.h>



class testXbotInterface: public testing::Test {
    

protected:

     testXbotInterface(){}

     virtual ~testXbotInterface(){}
     virtual void SetUp() {
         
         std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
         std::string relative_path = "/external/XBotInterface/configs/config_walkman.yaml";
         xbint.init(robotology_root+relative_path);
         path_to_cfg = robotology_root + relative_path;
     }

     virtual void TearDown() {
     }
     
     XBot::IXBotInterface xbint;
     std::string path_to_cfg;
     
};


TEST_F(testXbotInterface, checkCopyConstructor){
    
    Eigen::VectorXd x, x_copy;
    
    srand(time(NULL));
    
    // Set some fields of xbint with random values
    
    x.setRandom(xbint.getJointNum());
    xbint.setDamping(x);
    
    x.setRandom(xbint.getJointNum());
    xbint.setEffortReference(x);
    
    x.setRandom(xbint.getJointNum());
    xbint.setJointPosition(x);
    
    x.setRandom(xbint.getJointNum());
    xbint.setJointVelocity(x);
    
    x.setRandom(xbint.getJointNum());
    xbint.setJointEffort(x);
    
    x.setRandom(xbint.getJointNum());
    xbint.setPositionReference(x);
    
    x.setRandom(xbint.getJointNum());
    xbint.setMotorPosition(x);
    
    
    // Copy constructor
    XBot::IXBotInterface xbint_copy(xbint);
    

    xbint.getDamping(x);
    xbint_copy.getDamping(x_copy);    
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    xbint.getJointEffort(x);
    xbint_copy.getJointEffort(x_copy);
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    xbint.getJointEffort(x);
    xbint_copy.getJointEffort(x_copy);
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    xbint.getJointPosition(x);
    xbint_copy.getJointPosition(x_copy);
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    xbint.getEffortReference(x);
    xbint_copy.getEffortReference(x_copy);
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    xbint.getPositionReference(x);
    xbint_copy.getPositionReference(x_copy);
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    xbint.getMotorPosition(x);
    xbint_copy.getMotorPosition(x_copy);
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    
    // Modify the copy and check that the original is not modified
    
    x_copy.setRandom(xbint.getJointNum());
    xbint_copy.setJointEffort(x_copy);
    xbint.getJointEffort(x);
    
    EXPECT_FALSE( (x.array() == x_copy.array()).all() );
    
}


TEST_F(testXbotInterface, checkCopyAssignment){
    
    Eigen::VectorXd x, x_copy;
    
    srand(time(NULL));
    
    // Set some fields of xbint with random values
    
    x.setRandom(xbint.getJointNum());
    xbint.setDamping(x);
    
    x.setRandom(xbint.getJointNum());
    xbint.setEffortReference(x);
    
    x.setRandom(xbint.getJointNum());
    xbint.setJointPosition(x);
    
    x.setRandom(xbint.getJointNum());
    xbint.setJointVelocity(x);
    
    x.setRandom(xbint.getJointNum());
    xbint.setJointEffort(x);
    
    x.setRandom(xbint.getJointNum());
    xbint.setPositionReference(x);
    
    x.setRandom(xbint.getJointNum());
    xbint.setMotorPosition(x);
    
    
    // Copy constructor
    XBot::IXBotInterface xbint_copy;
    xbint_copy = xbint;
    

    xbint.getDamping(x);
    xbint_copy.getDamping(x_copy);    
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    xbint.getJointEffort(x);
    xbint_copy.getJointEffort(x_copy);
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    xbint.getJointEffort(x);
    xbint_copy.getJointEffort(x_copy);
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    xbint.getJointPosition(x);
    xbint_copy.getJointPosition(x_copy);
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    xbint.getEffortReference(x);
    xbint_copy.getEffortReference(x_copy);
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    xbint.getPositionReference(x);
    xbint_copy.getPositionReference(x_copy);
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    xbint.getMotorPosition(x);
    xbint_copy.getMotorPosition(x_copy);
    EXPECT_TRUE( (x.array() == x_copy.array()).all() );
    
    
    // Modify the copy and check that the original is not modified
    
    x_copy.setRandom(xbint.getJointNum());
    xbint_copy.setJointEffort(x_copy);
    xbint.getJointEffort(x);
    
    EXPECT_FALSE( (x.array() == x_copy.array()).all() );
    
}


TEST_F(testXbotInterface, checkJointGetters){
    
    for( const auto& j : xbint.getEnabledJointNames() ){
    
        EXPECT_TRUE( j == xbint.getJointByName(j)->getJointName() );
        
        int id = xbint.getJointByName(j)->getJointId();
        EXPECT_TRUE( id == xbint.getJointByID(id)->getJointId() );
    
    }
    
    Eigen::VectorXd x;
    x.setRandom( xbint.getJointNum() );
    xbint.setJointPosition(x);
    std::map<int, double> x_map;
    xbint.getJointPosition(x_map);
    
    for( int idx = 0; idx < xbint.getJointNum(); idx++ ){
        
        int joint_id = xbint.getJointByEigenIdx(idx)->getJointId();
        
        ASSERT_TRUE( x_map.count(joint_id) == 1 );
        EXPECT_TRUE( x_map.at(joint_id) == x(idx) );
        
    }
    
    
}

#define GETTER_SETTER_TEST(getFieldName, setFieldName) \
xbint. setFieldName (x); \
\
y.setZero(y.size()); \
y_id_map.clear(); \
y_name_map.clear(); \
xbint. getFieldName (y); \
xbint. getFieldName (y_id_map); \
xbint. getFieldName (y_name_map); \
\
EXPECT_TRUE( (x.array() == y.array()).all() ); \
EXPECT_TRUE( x_id_map == y_id_map ); \
EXPECT_TRUE( x_name_map == y_name_map ); \
\
xbint. setFieldName (x_id_map); \
\
y.setZero(y.size()); \
y_id_map.clear(); \
y_name_map.clear(); \
xbint. getFieldName (y); \
xbint. getFieldName (y_id_map); \
xbint. getFieldName (y_name_map); \
\
EXPECT_TRUE( (x.array() == y.array()).all() ); \
EXPECT_TRUE( x_id_map == y_id_map ); \
EXPECT_TRUE( x_name_map == y_name_map ); \
\
xbint. setFieldName (x_name_map); \
\
y.setZero(y.size()); \
y_id_map.clear(); \
y_name_map.clear(); \
xbint. getFieldName (y); \
xbint. getFieldName (y_id_map); \
xbint. getFieldName (y_name_map); \
\
EXPECT_TRUE( (x.array() == y.array()).all() ); \
EXPECT_TRUE( x_id_map == y_id_map ); \
EXPECT_TRUE( x_name_map == y_name_map ); 

    


TEST_F(testXbotInterface, checkGettersSetters){
 
    
    Eigen::VectorXd x, y;
    std::map<int, double> x_id_map, y_id_map;
    std::map<std::string, double> x_name_map, y_name_map;
    
    x.setRandom( xbint.getJointNum() );
    for(int i = 0; i < x.size(); i++){
        x_id_map[xbint.getJointByEigenIdx(i)->getJointId()] = x(i);
        x_name_map[xbint.getJointByEigenIdx(i)->getJointName()] = x(i);
    }
    
    GETTER_SETTER_TEST(getDamping, setDamping)
    GETTER_SETTER_TEST(getStiffness, setStiffness)
    GETTER_SETTER_TEST(getJointPosition, setJointPosition)
    GETTER_SETTER_TEST(getPositionReference, setPositionReference)
    GETTER_SETTER_TEST(getJointVelocity, setJointVelocity)
    GETTER_SETTER_TEST(getVelocityReference, setVelocityReference)
    GETTER_SETTER_TEST(getJointEffort, setJointEffort)
    GETTER_SETTER_TEST(getEffortReference, setEffortReference)
    GETTER_SETTER_TEST(getMotorPosition, setMotorPosition)
    GETTER_SETTER_TEST(getMotorVelocity, setMotorVelocity)
    GETTER_SETTER_TEST(getTemperature, setTemperature)
    
}


TEST_F(testXbotInterface, checkSyncFrom){
    
    XBot::IXBotInterface other;
    other.init(path_to_cfg);
    
    Eigen::VectorXd x;
    
    x.setRandom(xbint.getJointNum());
    other.setDamping(x);
    x.setRandom(xbint.getJointNum());
    other.setStiffness(x);
    x.setRandom(xbint.getJointNum());
    other.setJointEffort(x);
    x.setRandom(xbint.getJointNum());
    other.setJointPosition(x);
    x.setRandom(xbint.getJointNum());
    other.setJointVelocity(x);
    x.setRandom(xbint.getJointNum());
    other.setMotorPosition(x);
    x.setRandom(xbint.getJointNum());
    other.setMotorVelocity(x);
    x.setRandom(xbint.getJointNum());
    other.setPositionReference(x);
    x.setRandom(xbint.getJointNum());
    other.setVelocityReference(x);
    x.setRandom(xbint.getJointNum());
    other.setEffortReference(x);
    
    xbint.syncFrom(other);
    
    Eigen::VectorXd y;
    
    xbint.getDamping(x);
    other.getDamping(y);
    EXPECT_TRUE( (x.array() == y.array()).all() );
    
    xbint.getStiffness(x);
    other.getStiffness(y);
    EXPECT_TRUE( (x.array() == y.array()).all() );
    
    xbint.getJointPosition(x);
    other.getJointPosition(y);
    EXPECT_TRUE( (x.array() == y.array()).all() );
    
    xbint.getJointVelocity(x);
    other.getJointVelocity(y);
    EXPECT_TRUE( (x.array() == y.array()).all() );
    
    xbint.getJointEffort(x);
    other.getJointEffort(y);
    EXPECT_TRUE( (x.array() == y.array()).all() );
    
    xbint.getMotorPosition(x);
    other.getMotorPosition(y);
    EXPECT_TRUE( (x.array() == y.array()).all() );
    
    xbint.getMotorVelocity(x);
    other.getMotorVelocity(y);
    EXPECT_TRUE( (x.array() == y.array()).all() );
    
    xbint.getTemperature(x);
    other.getTemperature(y);
    EXPECT_TRUE( (x.array() == y.array()).all() );
    
    xbint.getPositionReference(x);
    other.getPositionReference(y);
    EXPECT_TRUE( (x.array() == y.array()).all() );
    
    xbint.getVelocityReference(x);
    other.getVelocityReference(y);
    EXPECT_TRUE( (x.array() == y.array()).all() );
    
    xbint.getEffortReference(x);
    other.getEffortReference(y);
    EXPECT_TRUE( (x.array() == y.array()).all() );
    
    
}

TEST_F(testXbotInterface, checkJointLimits){
    
    srand(time(NULL));
    
    Eigen::VectorXd qmin, qmax, qdotmax, taumax;
    xbint.getJointLimits(qmin, qmax);
    xbint.getEffortLimits(taumax);
    xbint.getVelocityLimits(qdotmax);
    
    for(const auto& j : xbint.getEnabledJointNames()){
        
        int idx = xbint.getEigenID(j);
        
        EXPECT_EQ(xbint.getUrdf().getJoint(j)->limits->lower, qmin(idx));
        EXPECT_EQ(xbint.getUrdf().getJoint(j)->limits->upper, qmax(idx));
        EXPECT_EQ(xbint.getUrdf().getJoint(j)->limits->effort, taumax(idx));
        EXPECT_EQ(xbint.getUrdf().getJoint(j)->limits->velocity, qdotmax(idx));
    }
    
    Eigen::VectorXd q_in_range, tau_in_range, qdot_in_range;
    Eigen::VectorXd q_out_range, tau_out_range, qdot_out_range;
    std::vector<std::string> bad_joints, expected_bad_joints;
    
    Eigen::VectorXd alpha( xbint.getJointNum() );
    Eigen::VectorXd beta(alpha);
    
    int trials_number = 100;
    
    for(int trial = 0; trial < trials_number; trial++){
    
        expected_bad_joints.clear();
        for( int i = 0; i < xbint.getJointNum(); i++ ){
            alpha(i) = rand()/double(RAND_MAX);
            beta(i) = 2.5*(alpha(i) - 0.5);
            if(beta(i) < 0 || beta(i) > 1){
                expected_bad_joints.push_back(xbint.getJointByEigenIdx(i)->getJointName());
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
        EXPECT_TRUE( xbint.checkJointLimits(q_in_range, bad_joints) );
        EXPECT_TRUE( xbint.checkJointLimits(q_in_range) );
        EXPECT_TRUE( bad_joints.size() == 0 );
        EXPECT_FALSE( xbint.checkJointLimits(q_out_range, bad_joints) );
        EXPECT_FALSE( xbint.checkJointLimits(q_out_range) );
        EXPECT_TRUE( bad_joints == expected_bad_joints );
        
        bad_joints.clear();
        EXPECT_TRUE( xbint.checkEffortLimits(tau_in_range, bad_joints) );
        EXPECT_TRUE( xbint.checkEffortLimits(tau_in_range) );
        EXPECT_TRUE( bad_joints.size() == 0 );
        EXPECT_FALSE( xbint.checkEffortLimits(tau_out_range, bad_joints) );
        EXPECT_FALSE( xbint.checkEffortLimits(tau_out_range) );
        EXPECT_TRUE( bad_joints == expected_bad_joints );
        
        bad_joints.clear();
        EXPECT_TRUE( xbint.checkVelocityLimits(qdot_in_range, bad_joints) );
        EXPECT_TRUE( xbint.checkVelocityLimits(qdot_in_range) );
        EXPECT_TRUE( bad_joints.size() == 0 );
        EXPECT_FALSE( xbint.checkVelocityLimits(qdot_out_range, bad_joints) );
        EXPECT_FALSE( xbint.checkVelocityLimits(qdot_out_range) );
        EXPECT_TRUE( bad_joints == expected_bad_joints );
    
    }
    
}







