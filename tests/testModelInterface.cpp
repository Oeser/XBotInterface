#include <XBotInterface/ModelInterface.h>
#include <gtest/gtest.h>
#include <cstdlib>
#include <time.h>

class testModelInterface : public testing::Test {

protected:

virtual void SetUp(){

    std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
    std::string relative_path = "/external/XBotInterface/tests/configs/centauro/configs/config_centauro_upperbody.yaml";

    path_to_cfg = robotology_root + relative_path;
    path_to_full_body_cfg = robotology_root + "/external/XBotInterface/tests/configs/centauro/configs/config_centauro_full_body.yaml";

    std::cout << "PATH TO CFG: " << path_to_cfg << std::endl;

    model_ptr = XBot::ModelInterface::getModel(path_to_cfg);
    fb_model_ptr = XBot::ModelInterface::getModel(path_to_full_body_cfg);

}

virtual void TearDown(){}

XBot::ModelInterface::Ptr model_ptr, fb_model_ptr;
std::string path_to_cfg, path_to_full_body_cfg;

};

inline void computeCartesianError(const Eigen::Affine3d& ref, const Eigen::Affine3d& actual, Eigen::VectorXd& error)
{
    error.resize(6);

    Eigen::Quaterniond q(actual.linear()), q_d(ref.linear());
    Eigen::Vector3d orientation_error = q.w()*q_d.vec() - q_d.w()*q.vec() - q_d.vec().cross(q.vec());
    Eigen::Vector3d position_error = ref.translation() - actual.translation();

    error << position_error, orientation_error;
}


TEST_F(testModelInterface, checkBasicIK){

    XBot::ModelInterface& model = *model_ptr;

    std::string end_effector_name = model.arm(0).getTipLinkName();

    Eigen::VectorXd q_home;
    model.getRobotState("home", q_home);
    model.setJointPosition(q_home);
    model.update();

    std::cout << "HOMING: \n" << q_home << std::endl;



    Eigen::Affine3d desired_pose, actual_pose, initial_pose;
    Eigen::VectorXd cartesian_error(6), xdot(6), qdot, q;
    Eigen::MatrixXd J;


    model.getPose(end_effector_name, initial_pose);
    model.getJointPosition(q);

    int max_iter = 100;
    int iter = 0;
    double dt = 0.01;

    while( iter < max_iter){

        // Compute desired end-effector pose
        desired_pose.translation() = initial_pose.translation() + Eigen::Vector3d(0,0,1)*0.2*(iter/double(max_iter));
        desired_pose.linear() = initial_pose.linear();

        std::cout << "DESIRED POSE\n" << "Position:\n" << desired_pose.translation() << "\nOrientation:\n" << desired_pose.linear() << std::endl;

        // Compute actual pose
        model.getPose(end_effector_name, actual_pose);

        std::cout << "ACTUAL POSE\n" << "Position:\n" << actual_pose.translation() << "\nOrientation:\n" << actual_pose.linear() << std::endl;


        // Compute cartesian error and rotate it to world frame
        computeCartesianError(desired_pose, actual_pose, cartesian_error);
        std::cout << "ERROR: \n" << cartesian_error << std::endl;

        // Compute end-effector velocity
        xdot = 100*cartesian_error;

        // Compute jacobian
        model.getJacobian(end_effector_name, J);

        // Set columns to zero except for arm(0)
        for( const std::string& chain_name : model.getChainNames() ){
            if( chain_name != model.arm(0).getChainName() ){

                std::vector<int> indices_to_be_disabled;
                model.getDofIndex(chain_name, indices_to_be_disabled);

                for(int idx : indices_to_be_disabled){
                    J.col(idx).setZero();
                }
            }
        }

        std::cout << "JACOBIAN:\n" << J << std::endl;

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

TEST_F(testModelInterface, checkNumericalJacobian){

    XBot::ModelInterface& model = *model_ptr;

    Eigen::VectorXd q, q1, q2;
    q.setRandom(model.getJointNum());
    double step_size = 0.0001;


    Eigen::MatrixXd J, Jhat;
    Eigen::Affine3d T, T1, T2, dT;
    Eigen::Vector3d p, p1, p2, dPhi;
    Eigen::Matrix3d R, R1, R2, S;

    for( const std::string& j : model.getEnabledJointNames() ){

        std::string link_name = model.getUrdf().getJoint(j)->child_link_name;

        model.setJointPosition(q);
        model.update();

        model.getPose(link_name, T);
        p = T.translation();
        R = T.linear();

        model.getJacobian(link_name, J);
        Jhat = J;

        for( int i = 1; i < model.getJointNum(); i++ ){

            q2 = q;
            q2(i) += step_size/2;

            q1 = q;
            q1(i) -= step_size/2;

            model.setJointPosition(q1);
            model.update();
            model.getPose(link_name, T1);
            p1 = T1.translation();
            R1 = T1.linear();

            model.setJointPosition(q2);
            model.update();
            model.getPose(link_name, T2);
            p2 = T2.translation();
            R2 = T2.linear();

            Jhat.col(i).head(3) = (p2-p1)/step_size;
            S = (R2-R1)*R.transpose();
            Jhat(3,i) = S(2,1)/step_size;
            Jhat(4,i) = S(0,2)/step_size;
            Jhat(5,i) = S(1,0)/step_size;


        }

        std::cout << "Link: " << link_name << ",  (J-Jhat).norm()/J.size() = " << (J-Jhat).norm()/J.size() << std::endl;

        EXPECT_NEAR( (J-Jhat).norm()/J.size() , 0, 0.001 );

    }

}

TEST_F(testModelInterface, checkPoseConsistency){

    XBot::ModelInterface& model = *model_ptr;
    Eigen::VectorXd q;
    q.setRandom(model.getJointNum());
    model.setJointPosition(q);
    model.update();

    std::vector<std::string> link_names;

    for( const auto& j : model.getEnabledJointNames() ){
        link_names.push_back(model.getUrdf().getJoint(j)->child_link_name);
    }

    Eigen::Affine3d b_T_a, c_T_a, c_T_b;

    for( const std::string& link_a : link_names ){
        for( const std::string& link_b : link_names ){
            for( const std::string& link_c : link_names ){


                model.getPose(link_a, link_b, b_T_a);
                model.getPose(link_b, link_c, c_T_b);
                model.getPose(link_a, link_c, c_T_a);

                EXPECT_NEAR( ((c_T_b*b_T_a).translation() - c_T_a.translation()).norm()/3, 0.0, 0.0001 );
                EXPECT_NEAR( ((c_T_b*b_T_a).linear() - c_T_a.linear()).norm()/9, 0.0, 0.0001 );


            }

            model.getPose(link_a, link_b, b_T_a);
            model.getPose(link_b, c_T_b);
            model.getPose(link_a, c_T_a);

            EXPECT_NEAR( ((c_T_b*b_T_a).translation() - c_T_a.translation()).norm()/3, 0.0, 0.0001 );
            EXPECT_NEAR( ((c_T_b*b_T_a).linear() - c_T_a.linear()).norm()/9, 0.0, 0.0001 );

        }
    }



}

TEST_F(testModelInterface, checkJacobianMask){

    Eigen::MatrixXd J;


    auto chains = model_ptr->getChainNames();

    J.setRandom(6, model_ptr->getJointNum());

    for( int iter = 0; iter < 100; iter++){

        int random_chain = rand()%chains.size();

        model_ptr->maskJacobian(chains[random_chain], J);

        for(int i = 0; i < model_ptr->chain(chains[random_chain]).getJointNum(); i++){
            EXPECT_EQ( J.col(model_ptr->getDofIndex(model_ptr->chain(chains[random_chain]).getJointName(i))).norm(), 0.0 );
        }

    }



}

TEST_F(testModelInterface, checkRelativeJacobian){

    // TODO

}

TEST_F( testModelInterface, checkCOM ){

    XBot::ModelInterface& model = *model_ptr;

    for(int iter=0; iter < 100; iter++){

    Eigen::VectorXd q, q1, q2, qdot;
    q.setRandom(model.getJointNum());
    qdot.setRandom(model.getJointNum());


    model.setJointPosition(q);
    model.setJointVelocity(qdot);
    model.update();

    Eigen::MatrixXd Jcom, Jcom_hat;
    Eigen::Vector3d com, com1, com2, dcom;
    model.getCOMJacobian(Jcom);
    model.getCOM(com);
    model.getCOMVelocity(dcom);

    EXPECT_EQ( Jcom.rows(), 3 );
    EXPECT_EQ( Jcom.cols(), model.getJointNum() );


    Eigen::Vector3d dcom1;
    dcom1 = Jcom*qdot;

    EXPECT_NEAR( (dcom-dcom1).norm(), 0, 0.0001 );

    double step_size = 0.000001;
    Jcom_hat.resize(Jcom.rows(), Jcom.cols());

    for(int i = 0; i < model.getJointNum(); i++){

        q1 = q;
        q2 = q;
        q2(i) += step_size/2;
        q1(i) -= step_size/2;

        model.setJointPosition(q2);
        model.update();
        model.getCOM(com2);

        model.setJointPosition(q1);
        model.update();
        model.getCOM(com1);


        Jcom_hat.col(i) = (com2-com1)/step_size;


    }


    EXPECT_NEAR( (Jcom-Jcom_hat).norm()/Jcom.size(), 0, 0.00001 );
    EXPECT_NEAR( (dcom-Jcom_hat*qdot).norm(), 0, 0.0001);

    }



}


TEST_F( testModelInterface, checkJdotQdot ){

    XBot::ModelInterface& model = *model_ptr;
    std::string ee_name = model.arm(0).getTipLinkName();

    Eigen::VectorXd q, qdot;
    Eigen::Matrix<double,6,1> jdotqdot, jdotqdot_numerical;
    Eigen::MatrixXd J1, J2, Jdot;

    for(int i = 0; i < 100; i++){

        q.setRandom(model.getJointNum());
        qdot.setRandom(q.size());

        model.setJointPosition(q);
        model.setJointVelocity(qdot);
        model.update();

        model.computeJdotQdot(ee_name, Eigen::Vector3d::Zero(), jdotqdot);

        double dt = 1e-6;

        model.setJointPosition(q + 0.5*dt*qdot);
        model.update();
        model.getJacobian(ee_name, J2);

        model.setJointPosition(q - 0.5*dt*qdot);
        model.update();
        model.getJacobian(ee_name, J1);

        Jdot = (J2 - J1)/dt;
        jdotqdot_numerical = Jdot * qdot;

//         std::cout << "JdotQdot error: " << (jdotqdot-jdotqdot_numerical).norm()/model.getJointNum() << std::endl;

        EXPECT_NEAR( (jdotqdot-jdotqdot_numerical).norm()/model.getJointNum(), 0.0,  1e-6 );

    }

}

TEST_F( testModelInterface, checkSetFloatingBasePose ){


    for(int i = 0; i < 100; i++){

        Eigen::AngleAxisd rot;

        rot.axis().setRandom();
        rot.axis() /= rot.axis().norm();

        Eigen::Matrix2d random;
        random.setRandom();
        rot.angle() = random(0);

        Eigen::Affine3d pose;
        pose.linear() = rot.toRotationMatrix();
        pose.translation().setRandom();

        fb_model_ptr->setFloatingBasePose(pose);
        fb_model_ptr->update();

        Eigen::Affine3d actual_pose;
        fb_model_ptr->getPose("pelvis", actual_pose);

//         std::cout << "****\n" << pose.matrix() << "\n" << actual_pose.matrix() << std::endl;

        EXPECT_TRUE( pose.isApprox(actual_pose, 0.0001) );

    }


}

TEST_F( testModelInterface, checkSetFloatingBasePoseTwist ){


    for(int i = 0; i < 100; i++){

        Eigen::Vector6d twist, actual_twist;
        Eigen::VectorXd q, qdot;
        q.setRandom(fb_model_ptr->getJointNum());
        qdot.setRandom(fb_model_ptr->getJointNum());

        twist.setRandom();

        fb_model_ptr->setJointPosition(q);
        fb_model_ptr->setJointVelocity(qdot);
        fb_model_ptr->update();
        fb_model_ptr->setFloatingBaseTwist(twist);
        fb_model_ptr->update();

        fb_model_ptr->getVelocityTwist("pelvis", actual_twist);

//         std::cout << "****\n" << twist << "\n**\n" << actual_twist << std::endl;

        EXPECT_NEAR( (twist-actual_twist).norm(), 0, 0.0001 );

    }


}

int main ( int argc, char **argv )
{
     testing::InitGoogleTest ( &argc, argv );
     return RUN_ALL_TESTS();
}
