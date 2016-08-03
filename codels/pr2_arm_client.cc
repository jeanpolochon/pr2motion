#include <pr2_arm_client.hh>

RobotArm::RobotArm()
  : traj_client_(NULL),
    traj_mode_(SOFT_MOTION),
    max_vel_(0.5),
    max_acc_(0.5),
    max_jerk_(0.5),
    time_slot_(0.5)
{
}

RobotArm::~RobotArm(){
  delete traj_client_;
}

RobotArm::ERROR RobotArm::isConnected(){
  ERROR result = OK;
  if(traj_client_==NULL)
    result=INIT_NOT_DONE;
  else
    if(!traj_client_->isServerConnected())
      result=SERVER_NOT_CONNECTED;
  return result;
}


RobotArm::ERROR RobotArm::init(RobotArm::SIDE side){
  ERROR result=OK;
  bool wait_result=false;
  int nb_iter=0;
  int nb_iter_max=7;

  // get arm_joint_limits from the urdf model
  // Pr2 Model
  Pr2Model pr2_model;
  Pr2Model::ERROR pr2model_init_result;
  pr2model_init_result=pr2_model.getRobotModel();
  if(pr2model_init_result!=Pr2Model::OK) {
    return CANNOT_READ_LIMITS;
  } else {
    if (side == RIGHT) {
      if((pr2_model.Pr2Model::checkJointName("r_shoulder_pan_joint")==Pr2Model::OK)) {
	r_shoulder_pan_joint_limit_lower_=pr2_model.getJointLimitLower("r_shoulder_pan_joint");
	r_shoulder_pan_joint_limit_upper_=pr2_model.getJointLimitUpper("r_shoulder_pan_joint");
	r_shoulder_pan_joint_limit_velocity_=pr2_model.getJointLimitVelocity("r_shoulder_pan_joint");
	r_shoulder_pan_joint_limit_effort_=pr2_model.getJointLimitEffort("r_shoulder_pan_joint");
      } else {
	return UNKNOWN_JOINT;
      }
      if((pr2_model.Pr2Model::checkJointName("r_shoulder_lift_joint")==Pr2Model::OK)) {
	r_shoulder_lift_joint_limit_lower_=pr2_model.getJointLimitLower("r_shoulder_lift_joint");
	r_shoulder_lift_joint_limit_upper_=pr2_model.getJointLimitUpper("r_shoulder_lift_joint");
	r_shoulder_lift_joint_limit_velocity_=pr2_model.getJointLimitVelocity("r_shoulder_lift_joint");
	r_shoulder_lift_joint_limit_effort_=pr2_model.getJointLimitEffort("r_shoulder_lift_joint");
      } else {
	return UNKNOWN_JOINT;
      }
      if((pr2_model.Pr2Model::checkJointName("r_upper_arm_roll_joint")==Pr2Model::OK)) {
	r_upper_arm_roll_joint_limit_lower_=pr2_model.getJointLimitLower("r_upper_arm_roll_joint");
	r_upper_arm_roll_joint_limit_upper_=pr2_model.getJointLimitUpper("r_upper_arm_roll_joint");
	r_upper_arm_roll_joint_limit_velocity_=pr2_model.getJointLimitVelocity("r_upper_arm_roll_joint");
	r_upper_arm_roll_joint_limit_effort_=pr2_model.getJointLimitEffort("r_upper_arm_roll_joint");
      } else {
	return UNKNOWN_JOINT;
      }
      if((pr2_model.Pr2Model::checkJointName("r_elbow_flex_joint")==Pr2Model::OK)) {
	r_elbow_flex_joint_limit_lower_=pr2_model.getJointLimitLower("r_elbow_flex_joint");
	r_elbow_flex_joint_limit_upper_=pr2_model.getJointLimitUpper("r_elbow_flex_joint");
	r_elbow_flex_joint_limit_velocity_=pr2_model.getJointLimitVelocity("r_elbow_flex_joint");
	r_elbow_flex_joint_limit_effort_=pr2_model.getJointLimitEffort("r_elbow_flex_joint");
      } else {
	return UNKNOWN_JOINT;
      }
      //CONTINUOUS JOINT
      if((pr2_model.Pr2Model::checkJointName("r_forearm_roll_joint")==Pr2Model::OK)) {
	r_forearm_roll_joint_limit_velocity_=pr2_model.getJointLimitVelocity("r_forearm_roll_joint");
	r_forearm_roll_joint_limit_effort_=pr2_model.getJointLimitEffort("r_forearm_roll_joint");
      } else {
	return UNKNOWN_JOINT;
      }
      if((pr2_model.Pr2Model::checkJointName("r_wrist_flex_joint")==Pr2Model::OK)) {
	r_wrist_flex_joint_limit_lower_=pr2_model.getJointLimitLower("r_wrist_flex_joint");
	r_wrist_flex_joint_limit_upper_=pr2_model.getJointLimitUpper("r_wrist_flex_joint");
	r_wrist_flex_joint_limit_velocity_=pr2_model.getJointLimitVelocity("r_wrist_flex_joint");
	r_wrist_flex_joint_limit_effort_=pr2_model.getJointLimitEffort("r_wrist_flex_joint");
      } else {
	return UNKNOWN_JOINT;
      }
      //CONTINUOUS JOINT
      if((pr2_model.Pr2Model::checkJointName("r_wrist_roll_joint")==Pr2Model::OK)) {
	r_wrist_roll_joint_limit_velocity_=pr2_model.getJointLimitVelocity("r_wrist_roll_joint");
	r_wrist_roll_joint_limit_effort_=pr2_model.getJointLimitEffort("r_wrist_roll_joint");
      } else {
	return UNKNOWN_JOINT;
      }
    } else {
      if((pr2_model.Pr2Model::checkJointName("l_shoulder_pan_joint")==Pr2Model::OK)) {
	l_shoulder_pan_joint_limit_lower_=pr2_model.getJointLimitLower("l_shoulder_pan_joint");
	l_shoulder_pan_joint_limit_upper_=pr2_model.getJointLimitUpper("l_shoulder_pan_joint");
	l_shoulder_pan_joint_limit_velocity_=pr2_model.getJointLimitVelocity("l_shoulder_pan_joint");
	l_shoulder_pan_joint_limit_effort_=pr2_model.getJointLimitEffort("l_shoulder_pan_joint");
      } else {
	return UNKNOWN_JOINT;
      }
      if((pr2_model.Pr2Model::checkJointName("l_shoulder_lift_joint")==Pr2Model::OK)) {
	l_shoulder_lift_joint_limit_lower_=pr2_model.getJointLimitLower("l_shoulder_lift_joint");
	l_shoulder_lift_joint_limit_upper_=pr2_model.getJointLimitUpper("l_shoulder_lift_joint");
	l_shoulder_lift_joint_limit_velocity_=pr2_model.getJointLimitVelocity("l_shoulder_lift_joint");
	l_shoulder_lift_joint_limit_effort_=pr2_model.getJointLimitEffort("l_shoulder_lift_joint");
      } else {
	return UNKNOWN_JOINT;
      }
      if((pr2_model.Pr2Model::checkJointName("l_upper_arm_roll_joint")==Pr2Model::OK)) {
	l_upper_arm_roll_joint_limit_lower_=pr2_model.getJointLimitLower("l_upper_arm_roll_joint");
	l_upper_arm_roll_joint_limit_upper_=pr2_model.getJointLimitUpper("l_upper_arm_roll_joint");
	l_upper_arm_roll_joint_limit_velocity_=pr2_model.getJointLimitVelocity("l_upper_arm_roll_joint");
	l_upper_arm_roll_joint_limit_effort_=pr2_model.getJointLimitEffort("l_upper_arm_roll_joint");
      } else {
	return UNKNOWN_JOINT;
      }
      if((pr2_model.Pr2Model::checkJointName("l_elbow_flex_joint")==Pr2Model::OK)) {
	l_elbow_flex_joint_limit_lower_=pr2_model.getJointLimitLower("l_elbow_flex_joint");
	l_elbow_flex_joint_limit_upper_=pr2_model.getJointLimitUpper("l_elbow_flex_joint");
	l_elbow_flex_joint_limit_velocity_=pr2_model.getJointLimitVelocity("l_elbow_flex_joint");
	l_elbow_flex_joint_limit_effort_=pr2_model.getJointLimitEffort("l_elbow_flex_joint");
      } else {
	return UNKNOWN_JOINT;
      }
      //CONTINUOUS JOINT
      if((pr2_model.Pr2Model::checkJointName("l_forearm_roll_joint")==Pr2Model::OK)) {
	l_forearm_roll_joint_limit_velocity_=pr2_model.getJointLimitVelocity("l_forearm_roll_joint");
	l_forearm_roll_joint_limit_effort_=pr2_model.getJointLimitEffort("l_forearm_roll_joint");
      } else {
	return UNKNOWN_JOINT;
      }
      if((pr2_model.Pr2Model::checkJointName("l_wrist_flex_joint")==Pr2Model::OK)) {
	l_wrist_flex_joint_limit_lower_=pr2_model.getJointLimitLower("l_wrist_flex_joint");
	l_wrist_flex_joint_limit_upper_=pr2_model.getJointLimitUpper("l_wrist_flex_joint");
	l_wrist_flex_joint_limit_velocity_=pr2_model.getJointLimitVelocity("l_wrist_flex_joint");
	l_wrist_flex_joint_limit_effort_=pr2_model.getJointLimitEffort("l_wrist_flex_joint");
      } else {
	return UNKNOWN_JOINT;
      }
      //CONTINUOUS JOINT
      if((pr2_model.Pr2Model::checkJointName("l_wrist_roll_joint")==Pr2Model::OK)) {
	l_wrist_roll_joint_limit_velocity_=pr2_model.getJointLimitVelocity("l_wrist_roll_joint");
	l_wrist_roll_joint_limit_effort_=pr2_model.getJointLimitEffort("l_wrist_roll_joint");
      } else {
	return UNKNOWN_JOINT;
      }
    }
  }

  if(traj_client_==NULL) {
    // client creation
    if (side == RIGHT) {
      traj_client_= new TrajClient("r_arm_controller/joint_trajectory_action", true);
      arm_side_= RIGHT;
    } else {
      traj_client_= new TrajClient("l_arm_controller/joint_trajectory_action", true);
      arm_side_=LEFT;
    }
  }

  if(traj_client_!=NULL){
    if(!(traj_client_->isServerConnected())) {
      // wait for server answer
      while((wait_result==false)||(nb_iter<nb_iter_max)) {
	wait_result=traj_client_->waitForServer(ros::Duration(5.0));
	nb_iter=nb_iter+1;
	ROS_INFO("Waiting for the arm_controller/joint_trajectory_action action server to come up");
      }
      // no response = init failed
      if(wait_result==false){
	result=INIT_FAILED;
      } else {
	traj_mode_=RobotArm::SOFT_MOTION;
	clearTrajectory();
	max_acc_=0.5;
	max_vel_=0.5;
	max_jerk_=0.5;
	time_slot_=0.5;
      }
    }
  } else {
    ROS_INFO("RobotArm::init Not able to create the ArmClient");
    result= INIT_FAILED;
  }
  return result;
}

void RobotArm::clearTrajectory()
{
  arm_traj_.trajectory.points.resize(0);
  arm_traj_.trajectory.joint_names.resize(0);
}

RobotArm::ERROR RobotArm::setTraj(pr2_controllers_msgs::JointTrajectoryGoal * intraj){
  ERROR result = OK;
  // path nb_points
  int points_vector_size;
  int joint_names_vector_size;
  points_vector_size =  intraj->trajectory.points.size();
  joint_names_vector_size = intraj->trajectory.joint_names.size();
  arm_traj_.trajectory.points.resize(points_vector_size);
  arm_traj_.trajectory.joint_names.resize(joint_names_vector_size);
  for (size_t ind=0; ind<joint_names_vector_size; ind++) {
    arm_traj_.trajectory.joint_names[ind]=intraj->trajectory.joint_names[ind];
  }
  for ( size_t ind=0; ind < points_vector_size ; ind++) {
    arm_traj_.trajectory.points[ind].positions.resize(joint_names_vector_size);
    arm_traj_.trajectory.points[ind].velocities.resize(joint_names_vector_size);
    arm_traj_.trajectory.points[ind].accelerations.resize(joint_names_vector_size);
    arm_traj_.trajectory.points[ind].effort.resize(joint_names_vector_size);
    for ( size_t ind2=0; ind2 < joint_names_vector_size ; ind2++) {
      arm_traj_.trajectory.points[ind].positions[ind2]=intraj->trajectory.points[ind].positions[ind2];
      arm_traj_.trajectory.points[ind].velocities[ind2]=0.0;
      arm_traj_.trajectory.points[ind].accelerations[ind2]=0.0;
      arm_traj_.trajectory.points[ind].effort[ind2]=0.0;
    }
  }
  return result;
}

RobotArm::ERROR RobotArm::setMax(double max_vel, double max_acc, double max_jerk){
  ERROR result=OK;
  if((max_vel<0.0)||(max_vel>1.0))
    result=INVALID_PARAM;
  else
    max_vel_=max_vel;

  if((max_acc<0.0)||(max_acc>1.0))
    result=INVALID_PARAM;
  else
    max_acc_=max_acc;

  if((max_jerk<0.0)||(max_jerk>1.0))
    result=INVALID_PARAM;
  else
    max_jerk_=max_jerk;
  return result;
}


RobotArm::ERROR RobotArm::setT(double t){
  ERROR result=OK;
  if((t<0.0)||(t>2.0))
    result=INVALID_PARAM;
  else
    time_slot_=t;
  return result;
}


RobotArm::ERROR RobotArm::validateTrajectory()
{
  return validateTrajectory(&arm_traj_);
}

RobotArm::ERROR RobotArm::validateTrajectory(pr2_controllers_msgs::JointTrajectoryGoal * goal_cmd){
  ERROR result = OK;
  // right arm joint indice
  int r_shoulder_pan_joint_indice = 0;
  int r_shoulder_lift_joint_indice = 0;
  int r_upper_arm_roll_joint_indice = 0;
  int r_elbow_flex_joint_indice = 0;
  int r_forearm_roll_joint_indice = 0;
  int r_wrist_flex_joint_indice = 0;
  int r_wrist_roll_joint_indice = 0;

  // left arm joint indice
  int l_shoulder_pan_joint_indice = 0;
  int l_shoulder_lift_joint_indice = 0;
  int l_upper_arm_roll_joint_indice = 0;
  int l_elbow_flex_joint_indice = 0;
  int l_forearm_roll_joint_indice = 0;
  int l_wrist_flex_joint_indice = 0;
  int l_wrist_roll_joint_indice = 0;

  // path nb_points
  int points_vector_size;
  points_vector_size=goal_cmd->trajectory.points.size();

  if(points_vector_size==0){
    result = INVALID_TRAJ;
    return result;
  } else if(points_vector_size==1) {
    // we do not check the first point to avoid to be blocked in an invalid position
    // so if the trajectory as only one point, return
    return result;
  } else {

    // joint vector size
    int joint_names_vector_size = 0;
    joint_names_vector_size = goal_cmd->trajectory.joint_names.size();

    ROS_INFO("RobotArm::validateTrajectory Validate Traj with points_vector_size = %d, joint_names_vector_size = %d \n", points_vector_size, joint_names_vector_size);

    // get the correct index for all needed joints in the path
    for (size_t ind=0; ind<joint_names_vector_size; ind++) {
      if(goal_cmd->trajectory.joint_names[ind].compare("r_shoulder_pan_joint")==0){
	r_shoulder_pan_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("r_shoulder_lift_joint")==0){
	r_shoulder_lift_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("r_upper_arm_roll_joint")==0){
	r_upper_arm_roll_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("r_elbow_flex_joint")==0){
	r_elbow_flex_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("r_forearm_roll_joint")==0){
	r_forearm_roll_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("r_wrist_flex_joint")==0){
	r_wrist_flex_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("r_wrist_roll_joint")==0){
	r_wrist_roll_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("l_shoulder_pan_joint")==0){
	l_shoulder_pan_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("l_shoulder_lift_joint")==0){
	l_shoulder_lift_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("l_upper_arm_roll_joint")==0){
	l_upper_arm_roll_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("l_elbow_flex_joint")==0){
	l_elbow_flex_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("l_forearm_roll_joint")==0){
	l_forearm_roll_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("l_wrist_flex_joint")==0){
	l_wrist_flex_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("l_wrist_roll_joint")==0){
	l_wrist_roll_joint_indice = ind;
      }
    }

    // check each joint limits
    // the trajectory is considered as invalid if one of the joint position is out of the limits
    if(arm_side_ == RIGHT) {
      // we do not consider the first point to avoid to be blocked in an invalid position
      // that's why ind=1 at the beginning
      for (size_t ind=1;ind<goal_cmd->trajectory.points.size();ind++){
	if((goal_cmd->trajectory.points[ind].positions[r_shoulder_pan_joint_indice]<r_shoulder_pan_joint_limit_lower_) || (goal_cmd->trajectory.points[ind].positions[r_shoulder_pan_joint_indice]>r_shoulder_pan_joint_limit_upper_) || (goal_cmd->trajectory.points[ind].velocities[r_shoulder_pan_joint_indice]>r_shoulder_pan_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with r_shoulder_pan_joint bounds, value is %f limits [%f,%f] and velocity is %f max %f\n", goal_cmd->trajectory.points[ind].positions[r_shoulder_pan_joint_indice],r_shoulder_pan_joint_limit_lower_,r_shoulder_pan_joint_limit_upper_,goal_cmd->trajectory.points[ind].velocities[r_shoulder_pan_joint_indice],r_shoulder_pan_joint_limit_velocity_);
	  result = INVALID_TRAJ;
	}
	//r_shoulder_lift_joint
	if((goal_cmd->trajectory.points[ind].positions[r_shoulder_lift_joint_indice]<r_shoulder_lift_joint_limit_lower_) || (goal_cmd->trajectory.points[ind].positions[r_shoulder_lift_joint_indice]>r_shoulder_lift_joint_limit_upper_) || (goal_cmd->trajectory.points[ind].velocities[r_shoulder_lift_joint_indice]>r_shoulder_lift_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with r_shoulder_lift_joint bounds, value is %f limits [%f,%f] and velocity is %f max %f\n",goal_cmd->trajectory.points[ind].positions[r_shoulder_lift_joint_indice],r_shoulder_lift_joint_limit_lower_,r_shoulder_lift_joint_limit_upper_,goal_cmd->trajectory.points[ind].velocities[r_shoulder_lift_joint_indice],r_shoulder_lift_joint_limit_velocity_);
	  result = INVALID_TRAJ;
	}
	//r_upper_arm_roll_joint
	if((goal_cmd->trajectory.points[ind].positions[r_upper_arm_roll_joint_indice]<r_upper_arm_roll_joint_limit_lower_) || (goal_cmd->trajectory.points[ind].positions[r_upper_arm_roll_joint_indice]>r_upper_arm_roll_joint_limit_upper_) || (goal_cmd->trajectory.points[ind].velocities[r_upper_arm_roll_joint_indice]>r_upper_arm_roll_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with r_upper_arm_roll_joint bounds, current values are : position %f (limits low %f up %f) velocity %f (limit %f) \n", goal_cmd->trajectory.points[ind].positions[r_upper_arm_roll_joint_indice],r_upper_arm_roll_joint_limit_lower_, r_upper_arm_roll_joint_limit_upper_, goal_cmd->trajectory.points[ind].velocities[r_upper_arm_roll_joint_indice], r_upper_arm_roll_joint_limit_velocity_);
	  result = INVALID_TRAJ;
	}
	//r_elbow_flex_joint
	if((goal_cmd->trajectory.points[ind].positions[r_elbow_flex_joint_indice]<r_elbow_flex_joint_limit_lower_) || (goal_cmd->trajectory.points[ind].positions[r_elbow_flex_joint_indice]>r_elbow_flex_joint_limit_upper_) || (goal_cmd->trajectory.points[ind].velocities[r_elbow_flex_joint_indice]>r_elbow_flex_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with r_elbow_flex_joint bounds, current values are : position %f (limits low %f up %f) velocity %f (limit %f) \n", goal_cmd->trajectory.points[ind].positions[r_elbow_flex_joint_indice],r_elbow_flex_joint_limit_lower_, r_elbow_flex_joint_limit_upper_, goal_cmd->trajectory.points[ind].velocities[r_elbow_flex_joint_indice], r_elbow_flex_joint_limit_velocity_);
	  result = INVALID_TRAJ;
	}
	//r_forearm_roll_joint
	//CONTINUOUS
	if((goal_cmd->trajectory.points[ind].velocities[r_forearm_roll_joint_indice]>r_forearm_roll_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with r_forearm_roll_joint bounds \n");
	  result = INVALID_TRAJ;
	}
	//r_wrist_flex_joint
	if((goal_cmd->trajectory.points[ind].positions[r_wrist_flex_joint_indice]<r_wrist_flex_joint_limit_lower_) || (goal_cmd->trajectory.points[ind].positions[r_wrist_flex_joint_indice]>r_wrist_flex_joint_limit_upper_) || (goal_cmd->trajectory.points[ind].velocities[r_wrist_flex_joint_indice]>r_wrist_flex_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with r_wrist_flex_joint bounds, current values are : position %f (limits low %f up %f) velocity %f (limit %f) for point %d\n", goal_cmd->trajectory.points[ind].positions[r_wrist_flex_joint_indice],r_wrist_flex_joint_limit_lower_, r_wrist_flex_joint_limit_upper_, goal_cmd->trajectory.points[ind].velocities[r_wrist_flex_joint_indice], r_wrist_flex_joint_limit_velocity_, ind);
	  result = INVALID_TRAJ;
	}
	//r_wrist_roll_joint
	//CONTINUOUS
	if((goal_cmd->trajectory.points[ind].velocities[r_wrist_roll_joint_indice]>r_wrist_roll_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with r_wrist_roll_joint bounds \n");
	  result = INVALID_TRAJ;
	}
      }

    } else if (arm_side_ == LEFT) {
      // we do not consider the first point to avoid to be blocked in an invalid position
      // that's why ind=1 at the beginning
      for (size_t ind=1;ind<goal_cmd->trajectory.points.size();ind++){
	//l_shoulder_pan_joint
	if((goal_cmd->trajectory.points[ind].positions[l_shoulder_pan_joint_indice]<l_shoulder_pan_joint_limit_lower_) || (goal_cmd->trajectory.points[ind].positions[l_shoulder_pan_joint_indice]>l_shoulder_pan_joint_limit_upper_) || (goal_cmd->trajectory.points[ind].velocities[l_shoulder_pan_joint_indice]>l_shoulder_pan_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with l_shoulder_pan_joint bounds \n");
	  result = INVALID_TRAJ;
	}
	//l_shoulder_lift_joint
	if((goal_cmd->trajectory.points[ind].positions[l_shoulder_lift_joint_indice]<l_shoulder_lift_joint_limit_lower_) || (goal_cmd->trajectory.points[ind].positions[l_shoulder_lift_joint_indice]>l_shoulder_lift_joint_limit_upper_) || (goal_cmd->trajectory.points[ind].velocities[l_shoulder_lift_joint_indice]>l_shoulder_lift_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with l_shoulder_lift_joint bounds \n");
	  result = INVALID_TRAJ;
	}
	//l_upper_arm_roll_joint
	if((goal_cmd->trajectory.points[ind].positions[l_upper_arm_roll_joint_indice]<l_upper_arm_roll_joint_limit_lower_) || (goal_cmd->trajectory.points[ind].positions[l_upper_arm_roll_joint_indice]>l_upper_arm_roll_joint_limit_upper_) || (goal_cmd->trajectory.points[ind].velocities[l_upper_arm_roll_joint_indice]>l_upper_arm_roll_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with l_upper_arm_roll_joint bounds \n");
	  result = INVALID_TRAJ;
	}
	//l_elbow_flex_joint
	if((goal_cmd->trajectory.points[ind].positions[l_elbow_flex_joint_indice]<l_elbow_flex_joint_limit_lower_) || (goal_cmd->trajectory.points[ind].positions[l_elbow_flex_joint_indice]>l_elbow_flex_joint_limit_upper_) || (goal_cmd->trajectory.points[ind].velocities[l_elbow_flex_joint_indice]>l_elbow_flex_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with l_elbow_flex_joint bounds, current values are : position %f (limits low %f up %f) velocity %f (limit %f) \n", goal_cmd->trajectory.points[ind].positions[l_elbow_flex_joint_indice],l_elbow_flex_joint_limit_lower_, l_elbow_flex_joint_limit_upper_, goal_cmd->trajectory.points[ind].velocities[l_elbow_flex_joint_indice], l_elbow_flex_joint_limit_velocity_);
	  result = INVALID_TRAJ;
	}
	//l_forearm_roll_joint
	//CONTINUOUS
	if((goal_cmd->trajectory.points[ind].velocities[l_forearm_roll_joint_indice]>l_forearm_roll_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with l_forearm_roll_joint bounds \n");
	  result = INVALID_TRAJ;
	}
	//l_wrist_flex_joint
	if((goal_cmd->trajectory.points[ind].positions[l_wrist_flex_joint_indice]<l_wrist_flex_joint_limit_lower_) || (goal_cmd->trajectory.points[ind].positions[l_wrist_flex_joint_indice]>l_wrist_flex_joint_limit_upper_) || (goal_cmd->trajectory.points[ind].velocities[l_wrist_flex_joint_indice]>l_wrist_flex_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with l_wrist_flex_joint bounds, current values are : position %f (limits low %f up %f) velocity %f (limit %f) \n", goal_cmd->trajectory.points[ind].positions[l_wrist_flex_joint_indice],l_wrist_flex_joint_limit_lower_, l_wrist_flex_joint_limit_upper_, goal_cmd->trajectory.points[ind].velocities[l_wrist_flex_joint_indice], l_wrist_flex_joint_limit_velocity_);
	  result = INVALID_TRAJ;
	}
	//l_wrist_roll_joint
	//CONTINUOUS
	if((goal_cmd->trajectory.points[ind].velocities[l_wrist_roll_joint_indice]>l_wrist_roll_joint_limit_velocity_)){
	  ROS_INFO("RobotArm::validateTraj pb with l_wrist_roll_joint bounds \n");
	  result = INVALID_TRAJ;
	}
      }
    } else {
      result = INVALID_PARAM;
    }
  }
  return result;
}

bool RobotArm::move_isDone() {
  return (traj_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState RobotArm::move_getState() {
  return traj_client_->getState();
}

void RobotArm::move()
{
  move(&arm_traj_);
}

void RobotArm::move(pr2_controllers_msgs::JointTrajectoryGoal * goal_cmd){
  ROS_INFO("RobotArm::move Sending move arm goal");
  traj_client_->sendGoal(*goal_cmd);
  // 			  boost::bind(&Torso::move_doneCb, this, _1, _2),
  // 			  boost::bind(&Torso::move_activeCb, this),
  // 			  boost::bind(&Torso::move_feedbackCb, this, _1));
}

void RobotArm::cancelCmd(){
  traj_client_->cancelAllGoals();
}

void RobotArm::gettestPath(){
  gettestPath(&arm_traj_);
}

void RobotArm::gettestPath(pr2_controllers_msgs::JointTrajectoryGoal * goal_cmd){
  //arm extension trajectory
  if (arm_side_ == RIGHT)  {
    goal_cmd->trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal_cmd->trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal_cmd->trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal_cmd->trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal_cmd->trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal_cmd->trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal_cmd->trajectory.joint_names.push_back("r_wrist_roll_joint");
    goal_cmd->trajectory.points.resize(5);
    int ind=0;
    goal_cmd->trajectory.points[ind].positions.resize(7);
    goal_cmd->trajectory.points[ind].positions[0] = 0.0;
    goal_cmd->trajectory.points[ind].positions[1] = 0.0;
    goal_cmd->trajectory.points[ind].positions[2] = 0.0;
    goal_cmd->trajectory.points[ind].positions[3] = -0.5;
    goal_cmd->trajectory.points[ind].positions[4] = 0.0;
    goal_cmd->trajectory.points[ind].positions[5] = -0.2;
    goal_cmd->trajectory.points[ind].positions[6] = 0.0;
    goal_cmd->trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j< 7; ++j)
      {
	goal_cmd->trajectory.points[ind].velocities[j]=0.0;
      }
    goal_cmd->trajectory.points[ind].time_from_start = ros::Duration(ind+1.0);

    ind+=1;
    goal_cmd->trajectory.points[ind].positions.resize(7);
    goal_cmd->trajectory.points[ind].positions[0] = -1.0;
    goal_cmd->trajectory.points[ind].positions[1] = 0.0;
    goal_cmd->trajectory.points[ind].positions[2] = 0.0;
    goal_cmd->trajectory.points[ind].positions[3] = -0.5;
    goal_cmd->trajectory.points[ind].positions[4] = 0.0;
    goal_cmd->trajectory.points[ind].positions[5] = -0.2;
    goal_cmd->trajectory.points[ind].positions[6] = 0.0;
    goal_cmd->trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j< 7; ++j)
      {
    	goal_cmd->trajectory.points[ind].velocities[j]=0.0;
      }
    goal_cmd->trajectory.points[ind].time_from_start = ros::Duration(ind+1.0);

    ind+=1;
    goal_cmd->trajectory.points[ind].positions.resize(7);
    goal_cmd->trajectory.points[ind].positions[0] = -1.0;
    goal_cmd->trajectory.points[ind].positions[1] = 0.2;
    goal_cmd->trajectory.points[ind].positions[2] = 0.0;
    goal_cmd->trajectory.points[ind].positions[3] = -1.0;
    goal_cmd->trajectory.points[ind].positions[4] = 1.5;
    goal_cmd->trajectory.points[ind].positions[5] = -1.0;
    goal_cmd->trajectory.points[ind].positions[6] = 0.5;
    goal_cmd->trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j< 7; ++j)
      {
   	goal_cmd->trajectory.points[ind].velocities[j]=0.0;
      }
    goal_cmd->trajectory.points[ind].time_from_start = ros::Duration(ind+1.0);
    ind+=1;
    goal_cmd->trajectory.points[ind].positions.resize(7);
    goal_cmd->trajectory.points[ind].positions[0] = -1.3;
    goal_cmd->trajectory.points[ind].positions[1] = 1.0;
    goal_cmd->trajectory.points[ind].positions[2] = -2.0;
    goal_cmd->trajectory.points[ind].positions[3] = -1.0;
    goal_cmd->trajectory.points[ind].positions[4] = 1.5;
    goal_cmd->trajectory.points[ind].positions[5] = -1.0;
    goal_cmd->trajectory.points[ind].positions[6] = 0.5;
    goal_cmd->trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j< 7; ++j)
      {
   	goal_cmd->trajectory.points[ind].velocities[j]=0.0;
      }
    goal_cmd->trajectory.points[ind].time_from_start = ros::Duration(ind+1.0);
    ind+=1;
    goal_cmd->trajectory.points[ind].positions.resize(7);
    goal_cmd->trajectory.points[ind].positions[0] = -2.0;
    goal_cmd->trajectory.points[ind].positions[1] = 1.2;
    goal_cmd->trajectory.points[ind].positions[2] = -0.1;
    goal_cmd->trajectory.points[ind].positions[3] = -1.2;
    goal_cmd->trajectory.points[ind].positions[4] = 1.5;
    goal_cmd->trajectory.points[ind].positions[5] = -0.3;
    goal_cmd->trajectory.points[ind].positions[6] = 0.5;
    goal_cmd->trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j< 7; ++j)
      {
   	goal_cmd->trajectory.points[ind].velocities[j]=0.0;
      }
    goal_cmd->trajectory.points[ind].time_from_start = ros::Duration(ind+1.0);


  } else {
    goal_cmd->trajectory.joint_names.push_back("l_shoulder_pan_joint");
    goal_cmd->trajectory.joint_names.push_back("l_shoulder_lift_joint");
    goal_cmd->trajectory.joint_names.push_back("l_upper_arm_roll_joint");
    goal_cmd->trajectory.joint_names.push_back("l_elbow_flex_joint");
    goal_cmd->trajectory.joint_names.push_back("l_forearm_roll_joint");
    goal_cmd->trajectory.joint_names.push_back("l_wrist_flex_joint");
    goal_cmd->trajectory.joint_names.push_back("l_wrist_roll_joint");
    goal_cmd->trajectory.points.resize(5);
    int ind=0;
    goal_cmd->trajectory.points[ind].positions.resize(7);
    goal_cmd->trajectory.points[ind].positions[0] = 0.0;
    goal_cmd->trajectory.points[ind].positions[1] = 0.0;
    goal_cmd->trajectory.points[ind].positions[2] = 0.0;
    goal_cmd->trajectory.points[ind].positions[3] = -0.5;
    goal_cmd->trajectory.points[ind].positions[4] = 0.0;
    goal_cmd->trajectory.points[ind].positions[5] = -0.2;
    goal_cmd->trajectory.points[ind].positions[6] = 0.0;
    goal_cmd->trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j< 7; ++j)
      {
	goal_cmd->trajectory.points[ind].velocities[j]=0.0;
      }
    goal_cmd->trajectory.points[ind].time_from_start = ros::Duration(ind+1.0);

    ind+=1;
    goal_cmd->trajectory.points[ind].positions.resize(7);
    goal_cmd->trajectory.points[ind].positions[0] = 1.0;
    goal_cmd->trajectory.points[ind].positions[1] = 0.0;
    goal_cmd->trajectory.points[ind].positions[2] = 0.0;
    goal_cmd->trajectory.points[ind].positions[3] = -0.5;
    goal_cmd->trajectory.points[ind].positions[4] = 0.0;
    goal_cmd->trajectory.points[ind].positions[5] = -0.2;
    goal_cmd->trajectory.points[ind].positions[6] = 0.0;
    goal_cmd->trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j< 7; ++j)
      {
    	goal_cmd->trajectory.points[ind].velocities[j]=0.0;
      }
    goal_cmd->trajectory.points[ind].time_from_start = ros::Duration(ind+1.0);

    ind+=1;
    goal_cmd->trajectory.points[ind].positions.resize(7);
    goal_cmd->trajectory.points[ind].positions[0] = 1.0;
    goal_cmd->trajectory.points[ind].positions[1] = 0.2;
    goal_cmd->trajectory.points[ind].positions[2] = 0.0;
    goal_cmd->trajectory.points[ind].positions[3] = -1.0;
    goal_cmd->trajectory.points[ind].positions[4] = 1.5;
    goal_cmd->trajectory.points[ind].positions[5] = -1.0;
    goal_cmd->trajectory.points[ind].positions[6] = 0.5;
    goal_cmd->trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j< 7; ++j)
      {
   	goal_cmd->trajectory.points[ind].velocities[j]=0.0;
      }
    goal_cmd->trajectory.points[ind].time_from_start = ros::Duration(ind+1.0);
    ind+=1;
    goal_cmd->trajectory.points[ind].positions.resize(7);
    goal_cmd->trajectory.points[ind].positions[0] = 1.3;
    goal_cmd->trajectory.points[ind].positions[1] = 1.0;
    goal_cmd->trajectory.points[ind].positions[2] = 2.0;
    goal_cmd->trajectory.points[ind].positions[3] = -1.0;
    goal_cmd->trajectory.points[ind].positions[4] = 1.5;
    goal_cmd->trajectory.points[ind].positions[5] = -1.0;
    goal_cmd->trajectory.points[ind].positions[6] = 0.5;
    goal_cmd->trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j< 7; ++j)
      {
   	goal_cmd->trajectory.points[ind].velocities[j]=0.0;
      }
    goal_cmd->trajectory.points[ind].time_from_start = ros::Duration(ind+1.0);
    ind+=1;
    goal_cmd->trajectory.points[ind].positions.resize(7);
    goal_cmd->trajectory.points[ind].positions[0] = 2.0;
    goal_cmd->trajectory.points[ind].positions[1] = 1.2;
    goal_cmd->trajectory.points[ind].positions[2] = -0.1;
    goal_cmd->trajectory.points[ind].positions[3] = -1.2;
    goal_cmd->trajectory.points[ind].positions[4] = 1.5;
    goal_cmd->trajectory.points[ind].positions[5] = -0.3;
    goal_cmd->trajectory.points[ind].positions[6] = 0.5;
    goal_cmd->trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j< 7; ++j)
      {
   	goal_cmd->trajectory.points[ind].velocities[j]=0.0;
      }
    goal_cmd->trajectory.points[ind].time_from_start = ros::Duration(ind+1.0);
  }
}

RobotArm::ERROR RobotArm::computeTrajectoryG()
{
  return computeTrajectoryG(&arm_traj_,&arm_traj_);
}

RobotArm::ERROR RobotArm::computeTrajectoryG(pr2_controllers_msgs::JointTrajectoryGoal * path_cmd, pr2_controllers_msgs::JointTrajectoryGoal  * traj_cmd){
  ERROR result = OK;

  // path nb_points
  int points_vector_size;
  points_vector_size=path_cmd->trajectory.points.size();
  // joint vector size
  int joint_names_vector_size = 0;
  joint_names_vector_size = path_cmd->trajectory.joint_names.size();

  ROS_INFO("RobotArm::computeTrajectoryG Trajectory GATECH with points_vector_size = %d, joint_names_vector_size = %d \n", points_vector_size, joint_names_vector_size);
  if(points_vector_size==0)
    return INVALID_TRAJ;

  // data needed for TrajectoryG computation
  list<VectorXd> waypoints;
  VectorXd waypoint(joint_names_vector_size);
  VectorXd maxAcceleration(joint_names_vector_size);
  VectorXd maxVelocity(joint_names_vector_size);

  // fill the way points data
  for ( size_t ind=0; ind < points_vector_size ; ind++) {
    for (size_t jind=0; jind<joint_names_vector_size; jind++) {
      waypoint[jind]=path_cmd->trajectory.points[ind].positions[jind];
    }
    waypoints.push_back(waypoint);
  }
  // get arm_velocity_limits from the urdf model
  // Pr2 Model
  Pr2Model pr2_model;
  Pr2Model::ERROR pr2model_init_result;
  pr2model_init_result=pr2_model.getRobotModel();
  if(pr2model_init_result!=Pr2Model::OK) {
    return CANNOT_READ_LIMITS;
  }
  for (size_t jind=0; jind<joint_names_vector_size; jind++) {
    maxAcceleration[jind] = max_acc_;
    if((pr2_model.Pr2Model::checkJointName(path_cmd->trajectory.joint_names[jind])==Pr2Model::OK)) {
      maxVelocity[jind] = pr2_model.getJointLimitVelocity(path_cmd->trajectory.joint_names[jind]);
    } else {
      return UNKNOWN_JOINT;
    }
  }
  for (size_t jind=0; jind<joint_names_vector_size; jind++) {
    traj_cmd->trajectory.joint_names[jind]=path_cmd->trajectory.joint_names[jind];
  }
  //compute the trajectory
  TrajectoryG trajectory(PathG(waypoints, time_slot_), maxVelocity, maxAcceleration);
  trajectory.outputPhasePlaneTrajectory();
  if(trajectory.isValid()) {
    double duration = trajectory.getDuration();
    ROS_INFO("RobotArm::computeTrajectoryG Trajectory duration: %f seconds\n", duration);
    // compute the new trajectory number of points
    int traj_nb_points=0;
    for(double t = 0.0; t < duration; t += time_slot_) {
      traj_nb_points=traj_nb_points+1;
    }
    traj_cmd->trajectory.points.resize(traj_nb_points);
    ROS_INFO("RobotArm::computeTrajectoryG the new number of points of the trajectory is %d \n", traj_nb_points);
    double tind=0.0;
    // fill the trajectory
    for(int n=0;n<traj_nb_points;n=n+1) {
      traj_cmd->trajectory.points[n].time_from_start = ros::Duration(tind);
      traj_cmd->trajectory.points[n].positions.resize(joint_names_vector_size);
      traj_cmd->trajectory.points[n].velocities.resize(joint_names_vector_size);
      traj_cmd->trajectory.points[n].accelerations.resize(joint_names_vector_size);
      traj_cmd->trajectory.points[n].effort.resize(joint_names_vector_size);
      for (size_t jind=0; jind<joint_names_vector_size; jind++) {
	traj_cmd->trajectory.points[n].positions[jind] = trajectory.getPosition(tind)[jind];
	traj_cmd->trajectory.points[n].velocities[jind] = trajectory.getVelocity(tind)[jind];
	traj_cmd->trajectory.points[n].accelerations[jind] = 0.0;
	traj_cmd->trajectory.points[n].effort[jind] = 0.0;
      }
      tind=tind+time_slot_;
    }
    ROS_INFO("RobotArm::computeTrajectoryG Trajectory Gatech Filled");
  } else {
    result = TRAJECTORY_COMPUTATION_FAILED;
    ROS_INFO("RobotArm::computeTrajectoryG Trajectory generation failed \n");
  }
  return result;
}


RobotArm::ERROR RobotArm::computeTrajectorySoftMotion()
{
  return computeTrajectorySoftMotion(&arm_traj_,&arm_traj_);
}

RobotArm::ERROR RobotArm::computeTrajectorySoftMotion(pr2_controllers_msgs::JointTrajectoryGoal * path_cmd, pr2_controllers_msgs::JointTrajectoryGoal * traj_cmd){
  ERROR result = OK;

  // path nb_points
  int points_vector_size;

  // joint vector size
  int joint_names_vector_size = 0;

  points_vector_size=path_cmd->trajectory.points.size();

  if(points_vector_size==0)
    return INVALID_TRAJ;

  joint_names_vector_size = path_cmd->trajectory.joint_names.size();

  ROS_INFO("RobotArm::computeTrajectorySoftMotion Trajectory Soft Motion with points_vector_size = %d, joint_names_vector_size = %d \n", points_vector_size, joint_names_vector_size);

  std::deque< std::vector<double> > via_points;
  via_points.resize(points_vector_size);
  for(int i=0; i<points_vector_size;i++) {
    via_points[i].resize(joint_names_vector_size);
    for (size_t jind=0; jind<joint_names_vector_size; jind++) {
      via_points[i][jind] = path_cmd->trajectory.points[i].positions[jind];
    }
  }

  for (size_t jind=0; jind<joint_names_vector_size; jind++) {
    traj_cmd->trajectory.joint_names[jind] = path_cmd->trajectory.joint_names[jind];
  }
  // initialize the kinematical constraints
  SM_LIMITS limit;
  std::vector<SM_LIMITS> limits;
  // get arm_velocity_limits from the urdf model
  // Pr2 Model
  Pr2Model pr2_model;
  Pr2Model::ERROR pr2model_init_result;
  pr2model_init_result=pr2_model.getRobotModel();
  if(pr2model_init_result!=Pr2Model::OK) {
    return CANNOT_READ_LIMITS;
  }

  for (int i=0;i<joint_names_vector_size;i++) {
    limit.maxJerk = max_jerk_;
    limit.maxAcc = max_acc_;
    if((pr2_model.Pr2Model::checkJointName(path_cmd->trajectory.joint_names[i])==Pr2Model::OK)) {
      limit.maxVel = pr2_model.getJointLimitVelocity(path_cmd->trajectory.joint_names[i]);
    } else {
      return UNKNOWN_JOINT;
    }
    limits.push_back(limit);
  }
  SM_TRAJ traj;
  traj.clear();
  traj.resize(joint_names_vector_size);
  // 2 different computation of trajectories
  // Trajectory Type 1 : Compute a trajectory that stops at each via point
  //traj.computeTrajViaPoints(via_points,limits,SM_TRAJ::SM_STOP_AT_VIA_POINT);
  // Trajectory Type 2 : Compute a smooth trajectory
  //traj.computeTrajViaPoints(via_points,limits, SM_TRAJ::SM_SMOOTH_APPROACH_VIA_POINT);
  if(traj.computeTrajViaPoints(via_points,limits, SM_TRAJ::SM_SMOOTH_APPROACH_VIA_POINT)==0)
    {
      //trajectory computation ok
      double duration = traj.getDuration(); // get the execution time of the computed trajectory

      // get the new trajectory nb_points
      int n=0;
      for(double t = 0.0; t < duration; t += time_slot_) {
	n = n+1;
      }
      int nb_points_smtraj=n;

      ROS_INFO("RobotArm::computeTrajectorySoftMotion Trajectory Soft Motion with duration %f and nb_points =%d \n", duration, n);

      traj_cmd->trajectory.points.resize(nb_points_smtraj);
      n=0;
      double tind=0;
      std::vector<SM_COND> cond(joint_names_vector_size);

      // fill the trajectory
      for(int n = 0; n < nb_points_smtraj; n = n+1) {
	traj.getMotionCond(tind,cond);
	traj_cmd->trajectory.points[n].positions.resize(joint_names_vector_size);
	traj_cmd->trajectory.points[n].velocities.resize(joint_names_vector_size);
	traj_cmd->trajectory.points[n].accelerations.resize(joint_names_vector_size);
	traj_cmd->trajectory.points[n].effort.resize(joint_names_vector_size);
	for (size_t jind=0; jind<joint_names_vector_size; jind++) {
	  traj_cmd->trajectory.points[n].positions[jind] = cond[jind].x;
	  traj_cmd->trajectory.points[n].velocities[jind] = cond[jind].v;
	  traj_cmd->trajectory.points[n].accelerations[jind] = cond[jind].a;
	  traj_cmd->trajectory.points[n].effort[jind] = 0.0;
	}
	tind=tind+time_slot_;
      }
      ROS_INFO("RobotArm::computeTrajectorySoftMotion Trajectory Soft Motion Filled\n");
    } else {
    result=TRAJECTORY_COMPUTATION_FAILED;
    ROS_INFO("RobotArm::computeTrajectorySoftMotion Trajectory Soft Motion Failed\n");
  }
  return result;
}

RobotArm::ERROR RobotArm::setTrajMode(RobotArm::TRAJ_MODE mode)
{
  ERROR result=OK;
  if(mode>=NB_MODE)
    result = INVALID_PARAM;
  else
    traj_mode_=mode;
  return result;
}

RobotArm::ERROR RobotArm::computeTrajectory()
{
  ERROR result=OK;
  switch(traj_mode_){
  case SOFT_MOTION:
    return computeTrajectorySoftMotion();
  case GATECH:
    return computeTrajectoryG();
  case PATH:
    return result;
  default:
    result=INVALID_PARAM;
    return result;
  }
}
