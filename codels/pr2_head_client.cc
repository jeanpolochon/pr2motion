#include <pr2_head_client.hh>

RobotHead::RobotHead()
  : pan_min_(-2.8),
    pan_max_(2.8),
    tilt_min_(-0.37),
    tilt_max_(1.29),
    point_head_client_(NULL),
    traj_head_client_(NULL),
    min_duration_(1.0),
    min_duration_min_(0.0),
    max_velocity_(0.8)
{
}

RobotHead::~RobotHead(){
  delete point_head_client_;
  delete traj_head_client_;
}

RobotHead::ERROR RobotHead::setMaxVelocity(double max_velocity){
  ERROR result=OK;
  if(max_velocity<max_velocity_max_) {
    max_velocity_=max_velocity;
  } else {
    ROS_INFO("RobotHead::setMaxVelocity : max_velocity_max = %f where you propose max_velocity = %f, keep max_velocity %f", max_velocity_max_, max_velocity, max_velocity_);
    result = INVALID_PARAM;
  }
  return result;
}

double RobotHead::getMaxVelocity(){
  return max_velocity_;
}

RobotHead::ERROR RobotHead::setMinDuration(double min_duration_input){
  ERROR result=OK;
  ros::Duration min_duration(min_duration_input);
  if(min_duration>min_duration_min_) {
    min_duration_=min_duration;
  } else {
    ROS_INFO("RobotHead::setMinDuration : min_duration_min = %f where you propose min_duration = %f, keep min_duration %f", min_duration_min_.toSec(), min_duration.toSec(), min_duration_.toSec());
    result = INVALID_PARAM;
  }
  return result;
}

double RobotHead::getMinDuration(){
  return min_duration_.toSec();
}

RobotHead::ERROR RobotHead::init(){
  ERROR result=OK;

  // get head_joint_limits from the urdf model
  // Pr2 Model
  Pr2Model pr2_model;
  Pr2Model::ERROR pr2model_init_result;
  pr2model_init_result=pr2_model.getRobotModel();
  if(pr2model_init_result!=Pr2Model::OK) {
    return CANNOT_READ_LIMITS;
  } else {
    if((pr2_model.Pr2Model::checkJointName("head_pan_joint")==Pr2Model::OK)&&(pr2_model.Pr2Model::checkJointName("head_tilt_joint")==Pr2Model::OK)){
      pan_min_=pr2_model.getJointLimitLower("head_pan_joint");
      pan_max_=pr2_model.getJointLimitUpper("head_pan_joint");
      tilt_min_=pr2_model.getJointLimitLower("head_tilt_joint");
      tilt_max_=pr2_model.getJointLimitUpper("head_tilt_joint");
      max_velocity_max_=pr2_model.getJointLimitVelocity("head_tilt_joint")>pr2_model.getJointLimitVelocity("head_pan_joint")?pr2_model.getJointLimitVelocity("head_pan_joint"):pr2_model.getJointLimitVelocity("head_tilt_joint");
      ROS_INFO("RobotHead pan_min %f, pax_max %f, tilt_min_ %f, tilt_max_ %f, max_velocity_max_ %f\n",pan_min_, pan_max_, tilt_min_, tilt_max_, max_velocity_max_);
      // reset max_velocity if it has been set too high
      if(max_velocity_>max_velocity_max_){
	max_velocity_=0.9*max_velocity_max_;
      }
    } else {
      return UNKNOWN_JOINT;
    }
  }

  if(point_head_client_==NULL)
    //Initialize the client for the Action interface to the head controller
    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

  if(point_head_client_!=NULL){
    //check if the client is already connected
    if(!point_head_client_->isServerConnected()) {
      //wait for head controller action server to come up
      ROS_INFO("Waiting for the point_head_action server to come up");
      point_head_client_->waitForServer(ros::Duration(5.0));
      if(!point_head_client_->isServerConnected()) {
	result=SERVER_NOT_CONNECTED;
      }
    }
  } else {
    ROS_INFO("RobotHead:: Not able to create the PointHeadClient");
    result= INIT_FAILED;
  }

  if(traj_head_client_==NULL)
    //Initialize the client for the Action interface to the head controller
    traj_head_client_ = new TrajHeadClient("/head_traj_controller/joint_trajectory_action", true);

  if(traj_head_client_!=NULL){
    //check if the client is already connected
    if(!traj_head_client_->isServerConnected()) {
      //wait for head controller action server to come up
      ROS_INFO("Waiting for the traj_head_action server to come up");
      traj_head_client_->waitForServer(ros::Duration(5.0));
      if(!traj_head_client_->isServerConnected()) {
	result=SERVER_NOT_CONNECTED;
      }
    }
  } else {
    ROS_INFO("RobotHead:: Not able to create the TrajHeadClient");
    result= INIT_FAILED;
  }

  return result;
}

RobotHead::ERROR RobotHead::isConnected(){
  ERROR result = OK;
  if((point_head_client_==NULL)||(traj_head_client_==NULL))
    result=INIT_NOT_DONE;
  else
    if((!point_head_client_->isServerConnected())||(!traj_head_client_->isServerConnected()))
      result=SERVER_NOT_CONNECTED;
  return result;
}

RobotHead::ERROR RobotHead::checkCmdLimits(double pan_desired_position, double tilt_desired_position)
{
  ERROR result=OK;
  if(pan_desired_position>pan_max_ || pan_desired_position<pan_min_) {
    ROS_INFO("RobotHead::checkCmdLimits head desired position out of bounds with pan = %f (limits [%f , %f] ) \n", pan_desired_position,pan_min_, pan_max_);
    result = INVALID_PARAM;
  }
  if(tilt_desired_position>tilt_max_ || tilt_desired_position<tilt_min_) {
    ROS_INFO("RobotHead::checkCmdLimits head desired position out of bounds with tilt = %f (limits [%f , %f] ) \n", tilt_desired_position,tilt_min_, tilt_max_);
    result = INVALID_PARAM;
  }
  return result;
}


// to be able to use this callback function, you need to
// create a nodehandle and a subscriber to read the desired position
//ros::NodeHandle n;
//ros::Subscriber sub = n.subscribe("/head_traj_controller/state", 1000, &RobotHead::listenerCallback, this);
//if(sub)
//  printf("the callback is valid \n");
void RobotHead::listenerCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
  // count
  int count=0;

  // check pan boundaries
  if(msg->joint_names[0].compare("head_pan_joint")==0) {
    count = count+1;
    ROS_INFO("head desired position with pan = %f (limits [%f , %f] ) \n", msg->desired.positions[0],pan_min_, pan_max_);
    if(msg->desired.positions[0]>pan_max_ || msg->desired.positions[0]<pan_min_) {
      ROS_INFO("head desired position out of bound with pan = %f (limits [-2.8 , 2.8] ) \n", msg->desired.positions[0]);
      point_head_client_->cancelAllGoals();
    }
  }

  if(msg->joint_names[1].compare("head_pan_joint")==0) {
    count = count + 1;
    ROS_INFO("head desired position with pan = %f (limits [%f , %f] ) \n", msg->desired.positions[1],pan_min_, pan_max_);
    if(msg->desired.positions[1]>pan_max_ || msg->desired.positions[1]<pan_min_) {
      ROS_INFO("head desired position out of bound with pan = %f (limits [-2.8 , 2.8] ) \n", msg->desired.positions[1]);
      point_head_client_->cancelAllGoals();
    }
  }

  // check tilt boundaries
  if(msg->joint_names[0].compare("head_tilt_joint")==0) {
    count = count +1;
    ROS_INFO("head desired position with tilt = %f (limits [%f, %f])\n", msg->desired.positions[0], tilt_min_, tilt_max_);
    if(msg->desired.positions[0]>tilt_max_ || msg->desired.positions[0]<tilt_min_) {
      ROS_INFO("head desired position out of bound with tilt = %f (limits [-0.37, 1.29])\n", msg->desired.positions[0]);
      point_head_client_->cancelAllGoals();
    }
  }

  if(msg->joint_names[1].compare("head_tilt_joint")==0) {
    count = count +1;
    ROS_INFO("head desired position with tilt = %f (limits [%f, %f])\n", msg->desired.positions[1], tilt_min_, tilt_max_);
    if(msg->desired.positions[1]>tilt_max_ || msg->desired.positions[1]<tilt_min_) {
      ROS_INFO("head desired position out of bound with tilt = %f (limits [-0.37, 1.29])\n", msg->desired.positions[1]);
      point_head_client_->cancelAllGoals();
    }
  }

  // check if we have read pan and tilt data
  if(count !=2)
    ROS_INFO("ERROR: we do not get all head bounds !!!");

}


bool RobotHead::lookAt_isDone() {
  return (point_head_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState RobotHead::lookAt_getState() {
  return point_head_client_->getState();
}

void RobotHead::lookAt_doneCb(const actionlib::SimpleClientGoalState& state,
			      const pr2_controllers_msgs::PointHeadActionResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void RobotHead::lookAt_activeCb()
{
  ROS_INFO("Goal just went active");
}

void RobotHead::lookAt_feedbackCb(const pr2_controllers_msgs::PointHeadActionFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback\n");
}

void RobotHead::lookAt(std::string frame_id, double x, double y, double z){
  //the goal message
  pr2_controllers_msgs::PointHeadGoal goal_cmd;

  //the target point, expressed in the requested frame
  geometry_msgs::PointStamped point;
  point.header.frame_id = frame_id;
  point.point.x = x; point.point.y = y; point.point.z = z;
  goal_cmd.target = point;

  //we are pointing the high-def camera frame
  //(pointing_axis defaults to X-axis)
  goal_cmd.pointing_frame = "head_plate_frame";
  goal_cmd.pointing_axis.x = 1.0;
  goal_cmd.pointing_axis.y = 0.0;
  goal_cmd.pointing_axis.z = 0.0;

  //Either or both of the min_duration or the max_velocity can be left unspecified,
  // in which case they are ignored.
  // If both are unspecified, the head will reach its goal as fast as possible.

  // min duration
  goal_cmd.min_duration = min_duration_;
  // but go no faster than max_velocity_default_;
  goal_cmd.max_velocity = max_velocity_;

  //send the goal
  point_head_client_->sendGoal(goal_cmd);

  //send the goal with callback (never achieved to make it work with that client)
  // point_head_client_->sendGoal(goal_cmd,
  //   boost::bind(&RobotHead::lookAt_doneCb, this, _1, _2),
  //   boost::bind(&RobotHead::lookAt_activeCb, this),
  //   boost::bind(&RobotHead::lookAt_feedbackCb, this, _1));

}

void RobotHead::cancelCmd(){
  point_head_client_->cancelAllGoals();
  traj_head_client_->cancelAllGoals();
}

bool RobotHead::movePanTilt_isDone() {
  return (traj_head_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState RobotHead::movePanTilt_getState() {
  return traj_head_client_->getState();
}

void RobotHead::movePanTilt(){
  movePanTilt(&head_traj_);
}

void RobotHead::movePanTilt(pr2_controllers_msgs::JointTrajectoryGoal * goal_cmd){
  ROS_INFO("RobotHead::move Sending move head goal");
  traj_head_client_->sendGoal(*goal_cmd);
  // 			  boost::bind(&Torso::move_doneCb, this, _1, _2),
  // 			  boost::bind(&Torso::move_activeCb, this),
  // 			  boost::bind(&Torso::move_feedbackCb, this, _1));
}


void RobotHead::clearTrajectory(){
  head_traj_.trajectory.points.resize(0);
  head_traj_.trajectory.joint_names.resize(0);
}

RobotHead::ERROR RobotHead::validateTrajectory(){
  return validateTrajectory(&head_traj_);
}

RobotHead::ERROR RobotHead::validateTrajectory(pr2_controllers_msgs::JointTrajectoryGoal * goal_cmd){
  ERROR result = OK;
  int head_pan_joint_indice = 0;
  int head_tilt_joint_indice = 0;

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

    ROS_INFO("RobotHead::validateTrajectory Validate Traj with points_vector_size = %d, joint_names_vector_size = %d \n", points_vector_size, joint_names_vector_size);

    // get the correct index for all needed joints in the path
    for (size_t ind=0; ind<joint_names_vector_size; ind++) {
      if(goal_cmd->trajectory.joint_names[ind].compare("head_pan_joint")==0){
	head_pan_joint_indice = ind;
      }
      if(goal_cmd->trajectory.joint_names[ind].compare("head_tilt_joint")==0){
	head_tilt_joint_indice = ind;
      }
    }

    // check each joint limits
    // the trajectory is considered as invalid if one of the joint position is out of the limits
    // we do not consider the first point to avoid to be blocked in an invalid position
    // that's why ind=1 at the beginning
    for (size_t ind=1;ind<goal_cmd->trajectory.points.size();ind++){
      if((goal_cmd->trajectory.points[ind].positions[head_pan_joint_indice]<pan_min_) || (goal_cmd->trajectory.points[ind].positions[head_pan_joint_indice]>pan_max_) || (goal_cmd->trajectory.points[ind].velocities[head_pan_joint_indice]>max_velocity_max_)){
	  ROS_INFO("RobotHead::validateTraj pb with head_pan_joint bounds, value is %f limits [%f,%f] and velocity is %f max %f\n", goal_cmd->trajectory.points[ind].positions[head_pan_joint_indice],pan_min_,pan_max_,goal_cmd->trajectory.points[ind].velocities[head_pan_joint_indice],max_velocity_max_);
	  result = INVALID_TRAJ;
      }
      if((goal_cmd->trajectory.points[ind].positions[head_tilt_joint_indice]<tilt_min_) || (goal_cmd->trajectory.points[ind].positions[head_tilt_joint_indice]>tilt_max_) || (goal_cmd->trajectory.points[ind].velocities[head_tilt_joint_indice]>max_velocity_max_)){
	ROS_INFO("RobotHead::validateTraj pb with head_tilt_joint bounds, value is %f limits [%f,%f] and velocity is %f max %f\n", goal_cmd->trajectory.points[ind].positions[head_tilt_joint_indice],tilt_min_,tilt_max_,goal_cmd->trajectory.points[ind].velocities[head_tilt_joint_indice],max_velocity_max_);
	result = INVALID_TRAJ;
      }
    }
  }
  return result;
}

RobotHead::ERROR RobotHead::setTraj(pr2_controllers_msgs::JointTrajectoryGoal * intraj){
  ERROR result = OK;
  // path nb_points
  int points_vector_size;
  int joint_names_vector_size;
  points_vector_size =  intraj->trajectory.points.size();
  joint_names_vector_size = intraj->trajectory.joint_names.size();
  head_traj_.trajectory.points.resize(points_vector_size);
  head_traj_.trajectory.joint_names.resize(joint_names_vector_size);
  for (size_t ind=0; ind<joint_names_vector_size; ind++) {
    head_traj_.trajectory.joint_names[ind]=intraj->trajectory.joint_names[ind];
  }
  for ( size_t ind=0; ind < points_vector_size ; ind++) {
    head_traj_.trajectory.points[ind].positions.resize(joint_names_vector_size);
    head_traj_.trajectory.points[ind].velocities.resize(joint_names_vector_size);
    head_traj_.trajectory.points[ind].accelerations.resize(joint_names_vector_size);
    head_traj_.trajectory.points[ind].effort.resize(joint_names_vector_size);
    for ( size_t ind2=0; ind2 < joint_names_vector_size ; ind2++) {
      head_traj_.trajectory.points[ind].positions[ind2]=intraj->trajectory.points[ind].positions[ind2];
      head_traj_.trajectory.points[ind].velocities[ind2]=0.0;
      head_traj_.trajectory.points[ind].accelerations[ind2]=0.0;
      head_traj_.trajectory.points[ind].effort[ind2]=0.0;
    }
  }
  return result;
}
