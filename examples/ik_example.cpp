#include <XBotInterface/RobotInterface.h>

void computeCartesianError(const Eigen::Affine3d& ref, const Eigen::Affine3d& actual, Eigen::VectorXd& error);

int main(int argc, char **argv){
    
    using namespace XBot;
    
    std::string path_to_config_file(argv[1]);
    
    /* This example will show you the usage of the XBotInterface library,
     * which provides  a framework-independent abstraction layer
     * for any robotic platform. First, the tutorial will go through the
     * basic syntax of the library, then a complete control loop example
     * taken from the domain of inverse kinematics will be shown.
     * 
     * As a first step, we need to instantiate a RobotInterface, which 
     * provides us functionalities to control a robot (like e.g. reading
     * encoders, force-torques, setting references for controllers, ...).
     * To this aim we use the static method RobotInterface::getRobot() which
     * returns a pointer to the required object.
     * 
     * Note that RobotInterface is implemented as a singleton. This means
     * that you can call getRobot multiple times in your code, but you will
     * always get a handle (a pointer) to the same object.
     */
    
    RobotInterface& robot = *RobotInterface::getRobot(path_to_config_file);
    
    /* We now want to update the robot internal state with the latest sensor
     * readings. This is done by a method called sense(). */
    
    robot.sense();
    
    /* Now that our RobotInterface is up to date, let us read joint states.
     * A feature of the XBotInterface library is that it provides a nice
     * organization of a complex robot as a collection of kinematic chains.
     * For example, a humanoid robot joint state is more conveniently described
     * by the set of its chains states, e.g. left_arm, right_arm, left_leg, 
     * right_leg, torso and neck, as opposed to a huge configuration vector.
     * In XBotInterface, a chain in accessed by the chain() method, which returns
     * a KinematicChain by reference. For example, let's print the left arm state
     * via the << operator. 
     */ 
    
    std::cout << robot.chain("left_arm") << std::endl;
    
    // operator() is a short-hand for chain():
    
    std::cout << robot("right_arm") << std::endl;
    
    /* We also support two kinds of "standard kinematic groups", i.e. legs
     * and arms. This means that you can quickly access the i-th arm/leg by
     * writing:
     */
    
    int arm_id = 0;
    int leg_id = 1;
    
    std::cout << robot.arm(arm_id) << std::endl;
    std::cout << robot.leg(leg_id) << std::endl;
    
    /* Arms and legs are ordered as defined in the robot SRDF which is specified
     * inside the YAML configuration file. */
    
    /* Some functionalities provided by the KinematicChain API are the following:
     */
    
    // Getter for the chain name
    std::cout << "First arm name is :" << robot.arm(0).chainName() << std::endl; 
    
    // Base link name
    std::cout << "Right arm name is :" << robot("right_arm").baseLinkName() << std::endl; 
    
    // Tip link name
    std::cout << "Right arm name is :" << robot("right_arm").tipLinkName() << std::endl; 

    // Number of joints
    std::cout << "Torso is made of " << robot.chain("torso").getJointNum() << " joints" << std::endl;

    /* Let's now turn back to our goal, which is to read the robot state. This can be done
     * either chain-by-chain, or for the whole robot at once. In either cases, the joint state
     * information can be obtained in three different forms:
     * 
     *  - as a joint name -> joint value std::map
     *  - as a joint ID -> joint value std::map
     *  - as an Eigen::VectorXd, ordered from the chain base link to tip link
     * 
     * Joint positions, velocities and efforts can be obtained by the methods
     * getJointPosition(), getJointVelocity() and getJointEffort() respectively.
     * They work both with chains and the whole robot, and with the three
     * representations described above.
     */
    
    std::map<int, double> left_arm_position;
    robot.arm(0).getJointPosition(left_arm_position);
    
    std::map<std::string, double> right_arm_velocity;
    robot.chain("right_arm").getJointVelocity(right_arm_velocity);
    
    Eigen::VectorXd torso_torque;
    robot("torso").getJointEffort(torso_torque);
    
    std::map<std::string, double> robot_torque;
    robot.getJointPosition(robot_torque);
    
    Eigen::VectorXd robot_position;
    robot.getJointPosition(robot_position);
    
    /* RobotInterface::sense not only updates joint states, but also every other
     * sensor like, for example, force-torque sensors. XBotInterface provides
     * two ways of getting FT data, i.e.:
     * 
     *  - getter for a FT name -> pointer to FT std::map
     *  - pointer to FT by parent link name
     */
    
    /* A map of all robot FTs */
    auto ft_map = robot.getForceTorque(); // ft_map is a std::map<std::string, ForceTorqueSensor::ConstPtr>
    
    /* For example, let's obtain the FT sensor attached to the 
     * tip link of the second leg */
    ForceTorqueSensor::ConstPtr leg1_ft;
    if(!robot.getForceTorque(robot.leg(1).tipLinkName(), leg1_ft)){
        std::cerr << "ERROR! No ft was found on " << robot.leg(1).tipLinkName() << " link!" << std::endl;
    }
    
    /* FT sensors can be printed with operator<< */
    std::cout << *leg1_ft << std::endl;
    
    /* FT measurement can be accessed via the getForce/getTorque/getWrench methods.
     * Both Eigen and KDL are supported.
     */
    
    KDL::Wrench leg1_wrench;
    leg1_ft->getWrench(leg1_wrench);
    
    std::cout << "The force-torque sensor attached to " << robot.leg(1).tipLinkName() << " has name " << leg1_ft->sensorName() << " and its output is \n" << leg1_wrench << std::endl;
    
    
    /* Let us now move from sensors to actuators, so to say. We now want to
     * set the robot upper body (torso + arms) to some homing configuration.
     * Ordinarily, we would get our homing from (e.g.) a config file, but
     * for the sake of this example let us get it in a randomized way
     */
    
    Eigen::VectorXd q_left, q_right, q_torso;
    
    q_left.setRandom( robot.arm(0).getJointNum() );
    q_right.setRandom( robot.arm(1).getJointNum() );
    q_torso.setRandom( robot("torso").getJointNum() );
    
    /* Set the resulting random homing as a reference for controllers */
    
    robot("left_arm").setPositionReference(q_left);
    robot("right_arm").setPositionReference(q_right);
    robot("torso").setPositionReference(q_torso);
    
    /* Now check that our random configuration will not break
     * the robot, by checking it against joint limits. The 
     * "bad_joints" vector will contain the joints whose
     * values are beyond the limits.
     */
    
    std::vector<std::string> bad_joints;
    robot("left_arm").checkJointLimits(q_left, bad_joints);
    robot("right_arm").checkJointLimits(q_right, bad_joints);
    robot("torso").checkJointLimits(q_torso, bad_joints);
    
    /* If there are bad joints, let us change their reference value to the 
     * middle of the range, i.e. (qmin+qmax)/2
     */
    
    std::map<std::string, double> bad_joints_new_ref;
    
    for( const std::string& joint_name : bad_joints ){
        
        double qmin, qmax;
        
        robot.getJointByName(joint_name)->getJointLimits(qmin, qmax);
        
        bad_joints_new_ref[joint_name] = (qmin+qmax)/2;
        
    }
    
    /* Change only reference for joints specified in the map */
    robot.setPositionReference(bad_joints_new_ref); 
    
    /* To send the reference that we set to the controller, we just
     * do robot.move()
     */
    
    robot.move();
    
    /* Sleep for a while to allow the robot execute the commanded reference */
    sleep(3);
    
    /* Now that we know how to use the XBotInterface library to get
     * joint-space information about the robot, let us move to
     * Cartesian space. Inside an instance of RobotInterface there is
     * an internal kinematic/dynamic model which is synchronized to 
     * the robot state whenever a sense() is called. The internal model
     * is an instance of the ModelInterface class. ModelInterface shares
     * lots of functionalities with RobotInterface, as for example the 
     * semantic organization of a robot into chains. In order to access
     * the internal model, you just write robot.model().
     */
    
    // If sense() has just been called, this is exacly the same as
    // robot.chain("left_arm").getJointPosition(left_arm_position);
    robot.model().chain("left_arm").getJointPosition(left_arm_position);
    
    /* ModelInterface provides methods to compute common kinematic/dynamic+
     * quantities such as forward kinematics, jacobians, center of mass, 
     * inverse dynamics, gravity compensation, ...
     * The output can be obtained either in terms of Eigen::Matrix/Vector/Affine3d
     * or in terms of KDL::Vector/Frame/Jacobian/...
     * 
     * Just for making an example, assume that we are interested in the pose of 
     * the left arm tip link w.r.t. the torso base link reference frame. This
     * is computed as follows
     */
    
    KDL::Frame b_T_ee;
    robot.model().getPose(robot("left_arm").tipLinkName(), 
                          robot("torso").baseLinkName(),
                          b_T_ee);
    
    /* We can print out the transform between the frames... */
    std::cout << "Left arm tip link to torso base link transform: \n" << b_T_ee << std::endl;
    
    /* To obtain a pose w.r.t. to the world frame, just omit the second argument */
    Eigen::Affine3d world_T_ee;
    robot.model().getPose(robot("left_arm").tipLinkName(),
                          world_T_ee);
    
    /* While RobotInterface is a singleton, meaning that you can only
     * have one, ModelInterface is not. You can instantiate as many
     * models as you wish by calling the static method ModelInterface::getModel()
     */
    
    ModelInterface& model = *ModelInterface::getModel(path_to_config_file);
    
    /* Note that models cannot be copied (you would get an error, ask a developer
     * if you're curious why). However, you can synchronize a model with any other
     * one or with a robot by calling ModelInterface::syncFrom(). 
     */
    
    model.syncFrom(robot); // model state is now the same as the robot state
    
    /* External models are useful to do computations on states that are different
     * from the current state of the robot. Moreover, models can also act as references
     * for controllers via the RobotInterface::setReferenceFrom() method
     */
    
    robot.setReferenceFrom(model);
    robot.move(); // robot should not move since model was aligned to robot state
    
    /* Finally, let us turn to a somewhat realistic example of use of the
     * XBotInterface library. We will write a simple inverse kinematics based
     * control loop which allows a generic robot equipped with a torso and and
     * two arms to move according to a defined cartesian trajectory. The employed
     * algorithm is the most simple one, i.e. the jacobian-transposed base IK.
     * First of all define the end-effector frame and the frame according 
     * to which we want to set cartesian references: */
    
    std::string base_frame = robot("torso").baseLinkName();
    std::string end_effector = robot("right_arm").tipLinkName();
    
    /* We want to move the end effector in a way such that its
     * orientation remains constant, while the vertical position of
     * the origin moves up and down cyclically. To do so, first we 
     * need to put the initial end-effector pose inside some variable */
    
    Eigen::Affine3d initial_pose;
    
    // get the latest sensor data (internal model is also updated)
    robot.sense(); 
    
    // save the initial end-effector pose inside initial_pose
    robot.model().getPose(base_frame, end_effector, initial_pose);
    
    /* Since we will use the external model for IK computations,
     * first we align it to the actual robot configuration. */
    
    model.syncFrom(robot);
    
    /* Also obtain the vector of the whole model configuration,
     * which will be useful inside the control loop. */
    
    Eigen::VectorXd q;
    
    model.getJointPosition(q);
    
    // define period and length of the trajectory
    double period = 3;
    double length = 0.2;
    
    // initial time 
    double t0 = robot.getTime();
    double time = t0;
    double previous_iteration_time = time;
    
    /* RobotInterface::isRunning returns true as long as some
     * shutdown signal is not received */
    
    while( robot.isRunning() ){ 
        
        double time = robot.getTime() - t0;
        double dt = time - previous_iteration_time;
    
        Eigen::Affine3d desired_pose, actual_pose;
        
        // Set the desired end-effector pose at current time
        desired_pose.linear() = initial_pose.linear();
        desired_pose.translation() = initial_pose.translation() + 0.5*length*std::sin(2*3.1415/period*time);
        
        // Compute the pose corresponding to the model state
        model.getPose(base_frame, end_effector, actual_pose);
        
        // Compute the cartesian error
        Eigen::VectorXd cartesian_error;
        computeCartesianError(desired_pose, actual_pose, cartesian_error);
        
        // Set a cartesian velocity which is proportional to the error
        double ik_gain = 100;
        Eigen::VectorXd xdot = ik_gain * cartesian_error;
        
        // Compute the jacobian matrix
        Eigen::MatrixXd J;
        model.getJacobian(end_effector, J);
        
        // Compute the required joint velocities
        Eigen::VectorXd qdot = J.transpose()*cartesian_error;
        
        // Integrate the computed velocities
        q += qdot * dt;
        
        // Update the model
        model.setJointPosition(q);
        model.update();
        
        // Use the model state as a reference for controllers
        robot.setReferenceFrom(model);
        robot.move();
        
        
    }
    
    
    
    
    

    
    
    return 0;
    
}