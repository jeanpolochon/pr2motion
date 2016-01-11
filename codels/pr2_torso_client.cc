#include <pr2_torso_client.hh>


Torso::Torso()
  : position_min_(0.012),
    position_max_(0.30),
    min_duration_min_(1.0),
    min_duration_max_(15.0),
    min_duration_default_(2.0),
    max_velocity_min_(0.0),
    max_velocity_max_(1.0),
    max_velocity_default_(max_velocity_max_/2.0),
    is_connected_(false),
    torso_client_(NULL)
{
}
  
Torso::~Torso(){
  delete torso_client_;
}

Torso::ERROR Torso::init(){    
  ERROR result = OK;

  // get torso_joint_limits from the urdf model
  // Pr2 Model
  Pr2Model pr2_model;
  Pr2Model::ERROR pr2model_init_result;
  pr2model_init_result=pr2_model.getRobotModel();
  if(pr2model_init_result!=Pr2Model::OK) {
    return CANNOT_READ_LIMITS;
  } else {
    if(pr2_model.Pr2Model::checkJointName("torso_lift_joint")==Pr2Model::OK){
      position_min_=pr2_model.getJointLimitLower("torso_lift_joint");
      position_max_=pr2_model.getJointLimitUpper("torso_lift_joint");
      max_velocity_max_=pr2_model.getJointLimitVelocity("torso_lift_joint");
    } else {
      return UNKNOWN_JOINT;
    }
  }

  // test if the client has been already created or not
  if(torso_client_==NULL)
    torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);

  // if the client is created
  if(torso_client_!=NULL) {
    //check if the client is already connected
    if(!torso_client_->isServerConnected()) {
      // wait for the connection
      while(!torso_client_->waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the torso action server to come up");
      }
      if(!torso_client_->isServerConnected()) {
	result=SERVER_NOT_CONNECTED;
      }
    }
  } else {
    ROS_INFO("Not able to create the TorsoClient");
    result= INIT_FAILED;
  }
  return result;
}
    
Torso::ERROR Torso::isConnected(){
  ERROR result = OK;
  if(torso_client_==NULL)
    result=INIT_NOT_DONE;
  else
    if(!torso_client_->isServerConnected())
      result=SERVER_NOT_CONNECTED;
  return result;
}

double Torso::getMaxVelocityDefault(){
  return max_velocity_default_;
}

ros::Duration Torso::getMinDurationDefault(){
  return min_duration_default_;
}

Torso::ERROR Torso::checkParamLimits(ros::Duration duration, double max_velocity){
  ERROR result=OK;
  if( (duration>min_duration_max_) || (duration<min_duration_max_))
    result = INVALID_PARAM;
  if( (max_velocity>max_velocity_max_) || (max_velocity<max_velocity_min_))
    result = INVALID_PARAM;
  return result;
}

Torso::ERROR Torso::checkCmdLimits(double position){
  Torso::ERROR result=OK;
  if( (position<position_min_) || (position>position_max_))
    result = INVALID_PARAM;
  return result;
}

bool Torso::move_isDone() {
  return (torso_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState Torso::move_getState() {
  // INTERMEDIATE STATES : PENDING ACTIVE
  // TERMINAL STATES : REJECTED SUCCEEDED ABORTED RECALLED PREEMPTED LOST 	
  return torso_client_->getState();
}

void Torso::move_doneCb(const actionlib::SimpleClientGoalState& state,
	      const pr2_controllers_msgs::SingleJointPositionResultConstPtr& result)
  {
    // pr2_controllers_msgs/SingleJointPositionResult result
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  }
void Torso::move_activeCb()
  {
    ROS_INFO("Goal just went active");
  }
void Torso::move_feedbackCb(const pr2_controllers_msgs::SingleJointPositionFeedbackConstPtr& feedback)
  {
    // pr2_controllers_msgs/SingleJointPositionFeedback feedback
    ROS_INFO("Got Feedback\n");
  }
Torso::ERROR Torso::move(pr2_controllers_msgs::SingleJointPositionGoal goal_cmd){
  // pr2_controllers_msgs::SingleJointPositionGoal 
  // float64 position
  // duration min_duration
  // float64 max_velocity
  ERROR result = OK;
  result = checkParamLimits(goal_cmd.min_duration,goal_cmd.max_velocity);

  if(result != OK)
    return result;

  result = checkCmdLimits(goal_cmd.position);

  if(result != OK)
    return result;

  torso_client_->sendGoal(goal_cmd, 
			  boost::bind(&Torso::move_doneCb, this, _1, _2), 
			  boost::bind(&Torso::move_activeCb, this),
			  boost::bind(&Torso::move_feedbackCb, this, _1));

  return result;
}

void Torso::cancelCmd(){
  torso_client_->cancelAllGoals();
}
