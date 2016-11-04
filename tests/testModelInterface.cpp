#include <XBotInterface/ModelInterface.h>
#include <gtest/gtest.h>
#include <cstdlib>
#include <time.h>

class testModelInterface : public testing::Test {
  
protected:
    
virtual void SetUp(){
    
    std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
    std::string relative_path = "/external/XBotInterface/configs/config_centauro.yaml";
    
    path_to_cfg = robotology_root + relative_path;
    
    model_ptr = XBot::ModelInterface::getModel(path_to_cfg);

}

virtual void TearDown(){}

XBot::ModelInterface::Ptr model_ptr;
std::string path_to_cfg;
    
};

void computeCartesianError(const Eigen::Affine3d& ref, const Eigen::Affine3d& actual, Eigen::VectorXd& error)
{
    error.resize(6);
    
    Eigen::Quaterniond q(actual.linear()), q_d(ref.linear());
    Eigen::Vector3d orientation_error = q.w()*q_d.vec() - q_d.w()*q.vec() - q_d.vec().cross(q.vec());
    Eigen::Vector3d position_error = ref.translation() - actual.translation();
    
    error << position_error, orientation_error;
}


TEST_F(testModelInterface, checkBasicIK){
 
    XBot::ModelInterface& model = *model_ptr;
    
    std::string end_effector_name = model.arm(0).tipLinkName();
    
    Eigen::VectorXd q_home;
    model.getRobotState("home", q_home);
    model.setJointPosition(q_home);
    
    
    
    Eigen::Affine3d desired_pose, actual_pose, initial_pose;
    Eigen::VectorXd cartesian_error(6), xdot(6), qdot, q;
    Eigen::MatrixXd J;

    
    model.getPose(end_effector_name, initial_pose);
    
    int max_iter = 100;
    int iter = 0;
    double dt = 0.01;
    
    while( iter < max_iter){
        
        // Compute desired end-effector pose
        desired_pose.translation() = initial_pose.translation() + Eigen::Vector3d(0,0,1)*0.2;
        
        // Compute actual pose
        model.getPose(end_effector_name, actual_pose);

        // Compute cartesian error and rotate it to world frame
        computeCartesianError(desired_pose, actual_pose, cartesian_error);
        std::cout << "ERROR: \n" << cartesian_error << std::endl;
        
        // Compute end-effector velocity
        xdot = 100*cartesian_error;
        
        // Compute jacobian
        model.getJacobian(end_effector_name, J);

        // Set columns to zero except for arm(0)
        for( const std::string& chain_name : model.getChainNames() ){
            if( chain_name != model.arm(0).chainName() ){
                
                std::vector<int> indices_to_be_disabled;
                model.getDofIndex(chain_name, indices_to_be_disabled);
                
                for(int idx : indices_to_be_disabled){
                    J.col(idx).setZero();
                }
            }
        }
        
        qdot = J.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(xdot);
         
        // Integration step
        q += qdot*dt;
        
        // Update model state
        model.setJointPosition(q);
        model.update();
        
        iter++;
    
    }
    
    std::cout << cartesian_error << std::endl;
    EXPECT_NEAR( cartesian_error.norm(), 0, 0.01 );
    
}




int main ( int argc, char **argv )
{
     testing::InitGoogleTest ( &argc, argv );
     return RUN_ALL_TESTS();
}
