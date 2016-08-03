#include <pr2_gripper_client.hh>

GripperSimple::GripperSimple()
  : gripper_client_(NULL),
    open_position_(0.08),
    open_max_effort_(-1.0),
    close_position_(0.0),
    close_max_effort_(50.0)
{
}
GripperSimple::~GripperSimple(){
    delete gripper_client_;
}

GripperSimple::ERROR  GripperSimple::init(GripperSimple::SIDE side){
  ERROR result = OK;

  // get arm_joint_limits from the urdf model
  // Pr2 Model
  Pr2Model pr2_model;
  Pr2Model::ERROR pr2model_init_result;
  pr2model_init_result=pr2_model.getRobotModel();
  if(pr2model_init_result!=Pr2Model::OK) {
    return CANNOT_READ_LIMITS;
  } else {
    if (side == RIGHT) {
      if((pr2_model.Pr2Model::checkJointName("r_gripper_joint")==Pr2Model::OK)) {
	open_position_min_=pr2_model.getJointLimitLower("r_gripper_joint");
	open_position_max_=pr2_model.getJointLimitUpper("r_gripper_joint");
	close_position_min_=open_position_min_;
	close_position_max_=open_position_max_;
	open_max_effort_max_=pr2_model.getJointLimitEffort("r_gripper_joint");
	close_max_effort_max_=open_max_effort_max_;
	if((open_position_>open_position_max_)||(open_position_<open_position_min_)){
	  open_position_=0.5*open_position_max_;
	}
	if((close_position_>close_position_max_)||(close_position_<close_position_min_)){
	  close_position_=close_position_min_;
	}
      } else {
	return UNKNOWN_JOINT;
      }
    } else {
      if((pr2_model.Pr2Model::checkJointName("l_gripper_joint")==Pr2Model::OK)) {
	open_position_min_=pr2_model.getJointLimitLower("l_gripper_joint");
	open_position_max_=pr2_model.getJointLimitUpper("l_gripper_joint");
	close_position_min_=open_position_min_;
	close_position_max_=open_position_max_;
	open_max_effort_max_=pr2_model.getJointLimitEffort("l_gripper_joint");
	close_max_effort_max_=open_max_effort_max_;
	if((open_position_>open_position_max_)||(open_position_<open_position_min_)){
	  open_position_=0.5*open_position_max_;
	}
	if((close_position_>close_position_max_)||(close_position_<close_position_min_)){
	  close_position_=close_position_min_;
	}
      } else {
	return UNKNOWN_JOINT;
      }
    }
  }

  if(gripper_client_==NULL) {
    if (side==RIGHT) {
      //Initialize the client for the Action interface to the gripper controller
      //and tell the action client that we want to spin a thread by default
      gripper_client_ = new GripperSimpleClient("r_gripper_controller/gripper_action", true);
    } else {
      gripper_client_ = new GripperSimpleClient("l_gripper_controller/gripper_action", true);
    }
  }

  if(gripper_client_!=NULL) {
    if(!gripper_client_->isServerConnected()){
      //wait for the gripper action server to come up
      ROS_INFO("Waiting for the gripper controller to come up\n");
      gripper_client_->waitForServer(ros::Duration(5.0));
      if(!gripper_client_->isServerConnected()) {
	result=SERVER_NOT_CONNECTED;
      }
    }
  } else {
    ROS_INFO("Not able to create the GripperClient");
    result= INIT_FAILED;
  }
  return result;
}

GripperSimple::ERROR GripperSimple::isConnected(){
  ERROR result = OK;
  if(gripper_client_==NULL)
    result=INIT_NOT_DONE;
  else
    if(!gripper_client_->isServerConnected())
      result=SERVER_NOT_CONNECTED;
  return result;
}


  // CLOSE
bool GripperSimple::close_isDone() {
  return (gripper_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState GripperSimple::close_getState() {
   return gripper_client_->getState();
 }

void GripperSimple::close_doneCb(const actionlib::SimpleClientGoalState& state,
	      const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  }

void GripperSimple::close_activeCb()
{
  ROS_INFO("Goal just went active");
}

void GripperSimple::close_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr& feedback)
{
 ROS_INFO("Got Feedback position %f effort %f stalled %d readched_goal %d\n",feedback->position, feedback->effort, feedback->stalled, feedback->reached_goal);
}

void GripperSimple::close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal close_cmd;
    // open.command.position = 0.09;    // position open (9 cm)
    close_cmd.command.position = close_position_;
    // open.command.max_effort = -1.0;  // unlimited motor effort
    close_cmd.command.max_effort = close_max_effort_;
    ROS_INFO("Sending close goal");
    //gripper_client_->sendGoal(close, &close_doneCb, &close_activeCb,&close_feedbackCb);
    gripper_client_->sendGoal(close_cmd,
			      boost::bind(&GripperSimple::close_doneCb, this, _1, _2),
			      boost::bind(&GripperSimple::close_activeCb, this),
			      boost::bind(&GripperSimple::close_feedbackCb, this, _1));
  }

void GripperSimple::close_cancel(){
  gripper_client_->cancelAllGoals();
}

  // OPEN
bool GripperSimple::open_isDone() {
  return (gripper_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState GripperSimple::open_getState() {
  return gripper_client_->getState();
}
void GripperSimple::open_doneCb(const actionlib::SimpleClientGoalState& state,
	      const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  }

void GripperSimple::open_activeCb()
  {
    ROS_INFO("Goal just went active");
  }

void GripperSimple::open_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr& feedback)
  {

    ROS_INFO("Got Feedback position %f effort %f stalled %d readched_goal %d\n",feedback->position, feedback->effort, feedback->stalled, feedback->reached_goal);
  }

void GripperSimple::open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open_cmd;
    // open.command.position = 0.09;    // position open (9 cm)
    open_cmd.command.position = open_position_;
    // open.command.max_effort = -1.0;  // unlimited motor effort
    open_cmd.command.max_effort = open_max_effort_;
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open_cmd,
			      boost::bind(&GripperSimple::open_doneCb, this, _1, _2),
			      boost::bind(&GripperSimple::open_activeCb, this),
			      boost::bind(&GripperSimple::open_feedbackCb, this, _1));
  }

void GripperSimple::open_cancel(){
  gripper_client_->cancelAllGoals();
}

GripperSimple::ERROR GripperSimple::setOpenPosition(double open_position){
  ERROR result=OK;
  if((open_position<open_position_max_)&&(open_position>open_position_min_)){
    open_position_=open_position;
  } else {
    ROS_INFO("GripperSimple::setOpenPosition : open_position_max = %f, open_position_min = %f, where you propose open_position = %f, keep open_position %f", open_position_max_, open_position_min_, open_position, open_position_);
    result = INVALID_PARAM;
  }
  return result;
}

double GripperSimple::getOpenPosition(){
  return open_position_;
}

GripperSimple::ERROR GripperSimple::setClosePosition(double close_position){
  ERROR result=OK;
  if((close_position<close_position_max_)&&(close_position>close_position_min_)){
    close_position_=close_position;
  } else {
    ROS_INFO("GripperSimple::setClosePosition : close_position_max = %f, close_position_min = %f, where you propose close_position = %f, keep close_position %f", close_position_max_, close_position_min_, close_position, close_position_);
    result = INVALID_PARAM;
  }
  return result;
}

double GripperSimple::getClosePosition(){
  return close_position_;
}

GripperSimple::ERROR GripperSimple::setOpenMaxEffort(double open_max_effort){
  ERROR result=OK;
  if(((open_max_effort<open_max_effort_max_)&&(open_max_effort>0))||(open_max_effort == -1.0)){
    open_max_effort_=open_max_effort;
  } else {
    ROS_INFO("GripperSimple::setOpenMaxEffort : open_max_effort_max = %f, where you propose open_max_effort = %f, keep open_max_effort %f", open_max_effort_max_, open_max_effort, open_max_effort_);
    result = INVALID_PARAM;
  }
  return result;
}

double GripperSimple::getOpenMaxEffort(){
  return open_max_effort_;
}

GripperSimple::ERROR GripperSimple::setCloseMaxEffort(double close_max_effort){
  ERROR result=OK;
  if(((close_max_effort<close_max_effort_max_)&&(close_max_effort>0))||(close_max_effort == -1.0)){
    close_max_effort_=close_max_effort;
  } else {
    ROS_INFO("GripperSimple::setCloseMaxEffort : close_max_effort_max = %f, where you propose close_max_effort = %f, keep close_max_effort %f", close_max_effort_max_, close_max_effort, close_max_effort_);
    result = INVALID_PARAM;
  }
  return result;
}

double GripperSimple::getCloseMaxEffort(){
  return close_max_effort_;
}
