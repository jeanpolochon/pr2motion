#include <pr2_gripper_sensor_client.hh>

Gripper::Gripper()
  : gripper_client_(NULL),
    open_position_(0.08),
    open_max_effort_(-1.0),
    close_position_(0.0),
    close_max_effort_(50.0),
    findtwo_contact_conditions_(pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::BOTH),
    findtwo_zero_finger_tip_sensors_(true),
    place_trigger_conditions_(pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC),
    place_acceleration_trigger_magnitude_(5.0),
    place_slip_trigger_magnitude_(.05),
    grab_hardness_gain_(0.03),
    fingertip_force_(10)
{
}

Gripper::~Gripper(){
  delete gripper_client_;
  delete contact_client_;
  delete slip_client_;
  delete event_detector_client_;
  delete grab_client_;
  delete release_client_;
  delete force_client_;
}

Gripper::ERROR Gripper::init(Gripper::SIDE side){
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

  if((gripper_client_==NULL)||(contact_client_==NULL)||(slip_client_==NULL)||(event_detector_client_==NULL)||(grab_client_==NULL)||(release_client_==NULL)||(force_client_==NULL)){
    if (side==RIGHT) {
      //Initialize the client for the Action interface to the gripper controller
      //and tell the action client that we want to spin a thread by default
      gripper_client_ = new GripperClient("r_gripper_sensor_controller/gripper_action", true);
      contact_client_  = new ContactClient("r_gripper_sensor_controller/find_contact",true);
      slip_client_  = new SlipClient("r_gripper_sensor_controller/slip_servo",true);
      event_detector_client_  = new EventDetectorClient("r_gripper_sensor_controller/event_detector",true);
      grab_client_ = new GrabClient("r_gripper_sensor_controller/grab", true);
      release_client_ = new ReleaseClient("r_gripper_sensor_controller/release",true);
      force_client_ = new ForceClient("r_gripper_sensor_controller/force_servo",true);
    } else {
      gripper_client_ = new GripperClient("l_gripper_sensor_controller/gripper_action", true);
      contact_client_  = new ContactClient("l_gripper_sensor_controller/find_contact",true);
      slip_client_  = new SlipClient("l_gripper_sensor_controller/slip_servo",true);
      event_detector_client_  = new EventDetectorClient("l_gripper_sensor_controller/event_detector",true);
      grab_client_ = new GrabClient("l_gripper_sensor_controller/grab", true);
      release_client_ = new ReleaseClient("l_gripper_sensor_controller/release",true);
      force_client_ = new ForceClient("l_gripper_sensor_controller/force_servo",true);
    }
  }

  if((gripper_client_!=NULL)&&(contact_client_!=NULL)&&(slip_client_!=NULL)&&(event_detector_client_!=NULL)&&(grab_client_!=NULL)&&(release_client_!=NULL)&&(force_client_!=NULL)){
    if(!gripper_client_->isServerConnected()){
      //wait for the gripper action server to come up
      ROS_INFO("Waiting for the gripper_sensor_controller/gripper_action action server to come up");
      gripper_client_->waitForServer(ros::Duration(5.0));
      if(!gripper_client_->isServerConnected()){
	result=SERVER_NOT_CONNECTED;
      }
    }
    if(!contact_client_->isServerConnected()){
      ROS_INFO("Waiting for the gripper_sensor_controller/find_contact action server to come up");
      contact_client_->waitForServer(ros::Duration(5.0));
      if(!contact_client_->isServerConnected()){
	result = SERVER_NOT_CONNECTED;
      }
    }
    if(!slip_client_->isServerConnected()){
      ROS_INFO("Waiting for the gripper_sensor_controller/slip_servo action server to come up");
      slip_client_->waitForServer(ros::Duration(5.0));
      if(!slip_client_->isServerConnected()){
	result = SERVER_NOT_CONNECTED;
      }
    }
    if(!event_detector_client_->isServerConnected()){
      ROS_INFO("Waiting for the gripper_sensor_controller/event_detector action server to come up");
      event_detector_client_->waitForServer(ros::Duration(5.0));
      if(!event_detector_client_->isServerConnected()){
	result = SERVER_NOT_CONNECTED;
      }
    }
    if(!grab_client_->isServerConnected()){
      ROS_INFO("Waiting for the gripper_sensor_controller/grab action server to come up");
      grab_client_->waitForServer(ros::Duration(5.0));
      if(!grab_client_->isServerConnected()){
	result = SERVER_NOT_CONNECTED;
      }
    }
    if(!release_client_->isServerConnected()){
      ROS_INFO("Waiting for the gripper_sensor_controller/release action server to come up");
      release_client_->waitForServer(ros::Duration(5.0));
      if(!release_client_->isServerConnected()){
	result = SERVER_NOT_CONNECTED;
      }
    }
    if(!force_client_->isServerConnected()){
      ROS_INFO("Waiting for the gripper_sensor_controller/force_servo action server to come up");
      force_client_->waitForServer(ros::Duration(5.0));
      if(!force_client_->isServerConnected()){
	result = SERVER_NOT_CONNECTED;
      }
    }
  } else {
    ROS_INFO("Not able to create GripperSensorClient\n");
    result=INIT_FAILED;
  }
  return result;
}

Gripper::ERROR Gripper::isConnected(){
  ERROR result = OK;
  if((gripper_client_==NULL)||(contact_client_==NULL)||(slip_client_==NULL)||(event_detector_client_==NULL)){
    result=INIT_NOT_DONE;
  } else {
    if((!gripper_client_->isServerConnected())||(!contact_client_->isServerConnected())||(!slip_client_->isServerConnected())||(!event_detector_client_->isServerConnected()))
      result=SERVER_NOT_CONNECTED;
  }
  return result;
}

// CLOSE
bool Gripper::close_isDone() {
  return (gripper_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState Gripper::close_getState() {
  return gripper_client_->getState();
}

void Gripper::close_doneCb(const actionlib::SimpleClientGoalState& state,
			   const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void Gripper::close_activeCb()
{
  ROS_INFO("Goal just went active");
}

void Gripper::close_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback\n");
}

void Gripper::close(){
  // pr2_controllers_msgs::Pr2GripperCommandGoal open;
  pr2_controllers_msgs::Pr2GripperCommandGoal close_cmd;
  // open.command.position = 0.09;    // position open (9 cm)
  close_cmd.command.position = close_position_;
  // open.command.max_effort = -1.0;  // unlimited motor effort
  close_cmd.command.max_effort = close_max_effort_;
  ROS_INFO("Sending close goal");
  //gripper_client_->sendGoal(close, &close_doneCb, &close_activeCb,&close_feedbackCb);
  gripper_client_->sendGoal(close_cmd,
			    boost::bind(&Gripper::close_doneCb, this, _1, _2),
			    boost::bind(&Gripper::close_activeCb, this));
			    //boost::bind(&Gripper::close_feedbackCb, this, _1));   //This feedback will spam your terminal and log files
}

void Gripper::close_cancel(){
  gripper_client_->cancelAllGoals();
}

// OPEN
bool Gripper::open_isDone() {
  return (gripper_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState Gripper::open_getState() {
  return gripper_client_->getState();
}

void Gripper::open_doneCb(const actionlib::SimpleClientGoalState& state,
			  const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void Gripper::open_activeCb()
{
  ROS_INFO("Goal just went active");
}

void Gripper::open_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback\n");
}

void Gripper::open(){
  pr2_controllers_msgs::Pr2GripperCommandGoal open_cmd;
  // open_cmd.command.position = 0.09;    // position open (9 cm)
   open_cmd.command.position = open_position_;
  // open_cmd.command.max_effort = -1.0;  // unlimited motor effort
  open_cmd.command.max_effort = open_max_effort_;
  ROS_INFO("Sending open goal");
  gripper_client_->sendGoal(open_cmd,
			    boost::bind(&Gripper::open_doneCb, this, _1, _2),
			    boost::bind(&Gripper::open_activeCb, this));
			    //boost::bind(&Gripper::open_feedbackCb, this, _1));  //This feedback will spam your terminal and log files
}

void Gripper::open_cancel(){
  gripper_client_->cancelAllGoals();
}

//
// FINDTWO
// Find two contacts on the robot gripper
//
bool Gripper::findTwo_isDone() {
  return (contact_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState Gripper::findTwo_getState(){
  return contact_client_->getState();
}

void Gripper::findTwo_doneCb(const actionlib::SimpleClientGoalState& state,
			     const pr2_gripper_sensor_msgs::PR2GripperFindContactResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void Gripper::findTwo_activeCb()
{
  ROS_INFO("Goal just went active");
}

void Gripper::findTwo_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperFindContactFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback\n");
}

void Gripper::findTwoContacts(){
  ROS_INFO("findTwoContacts\n");
  pr2_gripper_sensor_msgs::PR2GripperFindContactGoal findTwo_cmd;
  findTwo_cmd.command.contact_conditions = findtwo_contact_conditions_;
  findTwo_cmd.command.zero_fingertip_sensors = findtwo_zero_finger_tip_sensors_;
  ROS_INFO("Sending find 2 contact goal");
  contact_client_->sendGoal(findTwo_cmd,
			    boost::bind(&Gripper::findTwo_doneCb, this, _1, _2),
			    boost::bind(&Gripper::findTwo_activeCb, this));
			    //boost::bind(&Gripper::findTwo_feedbackCb, this, _1));  //This feedback will spam your terminal and log files
}

void Gripper::findTwo_cancel(){
  contact_client_->cancelAllGoals();
}

//
// SLIPSERVO
// Slip servo the robot
//
bool Gripper::slipServo_isDone() {
  return (slip_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState Gripper::slipServo_getState(){
  return slip_client_->getState();
}

void Gripper::slipServo_doneCb(const actionlib::SimpleClientGoalState& state,
			       const pr2_gripper_sensor_msgs::PR2GripperSlipServoResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void Gripper::slipServo_activeCb()
{
  ROS_INFO("Goal just went active");
}

void Gripper::slipServo_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperSlipServoFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback\n");
}

void Gripper::slipServo(){
  pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal slip_goal;
  ROS_INFO("Slip Servoing");
  //    slip_client_->sendGoal(slip_goal, &slipServo_doneCb, &slipServo_activeCb, &slipServo_feedbackCb);
  //    slip_client_->sendGoal(slip_goal);
  slip_client_->sendGoal(slip_goal,
			 boost::bind(&Gripper::slipServo_doneCb, this, _1, _2),
			 boost::bind(&Gripper::slipServo_activeCb, this));
			 //boost::bind(&Gripper::slipServo_feedbackCb, this, _1));   //This feedback will spam your terminal and log files
}

void Gripper::slipServo_cancel(){
  slip_client_->cancelAllGoals();
}

//
// PLACE
// move into event_detector mode to detect object contact
//

Gripper::ERROR Gripper::setPlaceTriggerConditions(int8_t place_trigger_conditions){
  ERROR result=OK;
  if((place_trigger_conditions==pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::FINGER_SIDE_IMPACT_OR_ACC)||(place_trigger_conditions==pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::SLIP_AND_ACC)||(place_trigger_conditions==pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC)||(place_trigger_conditions==pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::SLIP)||(place_trigger_conditions==pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::ACC)){
    place_trigger_conditions_=place_trigger_conditions;
  } else {
    ROS_INFO("Gripper::setPlaceTriggerConditions : you propose trigger_conditions = %d which is not a possible value, keep trigger_conditions %d", place_trigger_conditions, place_trigger_conditions_);
    result = INVALID_PARAM;
  }
  return result;
}

int8_t Gripper::getPlaceTriggerConditions(){
  return place_trigger_conditions_;
}

Gripper::ERROR Gripper::setPlaceAccelerationTriggerMagnitude(double place_acceleration_trigger_magnitude){
  ERROR result=OK;
  place_acceleration_trigger_magnitude_=place_acceleration_trigger_magnitude;
  return result;
}

double Gripper::getPlaceAccelerationTriggerMagnitude(){
  return place_acceleration_trigger_magnitude_;
}

Gripper::ERROR Gripper::setPlaceSlipTriggerMagnitude(double place_slip_trigger_magnitude){
  ERROR result=OK;
  place_slip_trigger_magnitude_=place_slip_trigger_magnitude;
  return result;
}

double Gripper::getPlaceSlipTriggerMagnitude(){
  return place_slip_trigger_magnitude_;
}

bool Gripper::place_isDone() {
  return (event_detector_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState Gripper::place_getState(){
  return event_detector_client_->getState();
}

void Gripper::place_doneCb(const actionlib::SimpleClientGoalState& state,
			   const pr2_gripper_sensor_msgs::PR2GripperEventDetectorResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void Gripper::place_activeCb()
{
  ROS_INFO("Goal just went active");
}

void Gripper::place_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback\n");
}

void Gripper::place(){
  ROS_INFO("TRYING TO PLACE\n");
  pr2_gripper_sensor_msgs::PR2GripperEventDetectorGoal place_cmd;
  // place_goal.command.trigger_conditions = place_goal.command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;
  place_cmd.command.trigger_conditions = place_trigger_conditions_;
  // place_goal.command.acceleration_trigger_magnitude = 4.0;  // set the contact acceleration to n m/s^2
  place_cmd.command.acceleration_trigger_magnitude = place_acceleration_trigger_magnitude_;
  // place_goal.command.slip_trigger_magnitude = .005;
  place_cmd.command.slip_trigger_magnitude = place_slip_trigger_magnitude_;

  ROS_INFO("Waiting for object placement contact...");
  event_detector_client_->sendGoal(place_cmd,
				   boost::bind(&Gripper::place_doneCb,  this, _1, _2),
				   boost::bind(&Gripper::place_activeCb, this));
				   //boost::bind(&Gripper::place_feedbackCb, this, _1));  //This feedback will spam your terminal and log files
}

void Gripper::place_cancel(){
  event_detector_client_->cancelAllGoals();
}

Gripper::ERROR Gripper::setOpenPosition(double open_position){
  ERROR result=OK;
  if((open_position<open_position_max_)&&(open_position>open_position_min_)){
    open_position_=open_position;
  } else {
    ROS_INFO("Gripper::setOpenPosition : open_position_max = %f, open_position_min = %f, where you propose open_position = %f, keep open_position %f", open_position_max_, open_position_min_, open_position, open_position_);
    result = INVALID_PARAM;
  }
  return result;
}

double Gripper::getOpenPosition(){
  return open_position_;
}

Gripper::ERROR Gripper::setClosePosition(double close_position){
  ERROR result=OK;
  if((close_position<close_position_max_)&&(close_position>close_position_min_)){
    close_position_=close_position;
  } else {
    ROS_INFO("Gripper::setClosePosition : close_position_max = %f, close_position_min = %f, where you propose close_position = %f, keep close_position %f", close_position_max_, close_position_min_, close_position, close_position_);
    result = INVALID_PARAM;
  }
  return result;
}

double Gripper::getClosePosition(){
  return close_position_;
}

Gripper::ERROR Gripper::setOpenMaxEffort(double open_max_effort){
  ERROR result=OK;
  if(((open_max_effort<open_max_effort_max_)&&(open_max_effort>0))||(open_max_effort == -1.0)){
    open_max_effort_=open_max_effort;
  } else {
    ROS_INFO("Gripper::setOpenMaxEffort : open_max_effort_max = %f, where you propose open_max_effort = %f, keep open_max_effort %f", open_max_effort_max_, open_max_effort, open_max_effort_);
    result = INVALID_PARAM;
  }
  return result;
}

double Gripper::getOpenMaxEffort(){
  return open_max_effort_;
}

Gripper::ERROR Gripper::setCloseMaxEffort(double close_max_effort){
  ERROR result=OK;
  if(((close_max_effort<close_max_effort_max_)&&(close_max_effort>0))||(close_max_effort == -1.0)){
    close_max_effort_=close_max_effort;
  } else {
    ROS_INFO("Gripper::setCloseMaxEffort : close_max_effort_max = %f, where you propose close_max_effort = %f, keep close_max_effort %f", close_max_effort_max_, close_max_effort, close_max_effort_);
    result = INVALID_PARAM;
  }
  return result;
}

double Gripper::getCloseMaxEffort(){
  return close_max_effort_;
}

Gripper::ERROR Gripper::setFindTwoContactConditions(int8_t findtwo_contact_conditions){
  ERROR result=OK;
  if((findtwo_contact_conditions==pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::BOTH) || (findtwo_contact_conditions==pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::LEFT)||(findtwo_contact_conditions==pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::RIGHT) || (findtwo_contact_conditions==pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::EITHER)){
    findtwo_contact_conditions_=findtwo_contact_conditions;
  } else {
    ROS_INFO("Gripper::setFindTwoContactConditions : you propose findtwo_contact_conditions = %d which is not a possible value, keep findtwo_contact_conditions %d", findtwo_contact_conditions, findtwo_contact_conditions_);
    result = INVALID_PARAM;
  }
  return result;
}

int8_t Gripper::getFindTwoContactConditions(){
  return findtwo_contact_conditions_;
}

Gripper::ERROR Gripper::setFindTwoZeroFingerTipSensors(bool findtwo_zero_finger_tip_sensors){
  ERROR result =OK;
  findtwo_zero_finger_tip_sensors_=findtwo_zero_finger_tip_sensors;
  return result;
}

bool Gripper::getFindTwoZeroFingerTipSensors(){
  return findtwo_zero_finger_tip_sensors_;
}


// GRAB

bool Gripper::grab_isDone() {
  return (grab_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState Gripper::grab_getState() {
  return grab_client_->getState();
}

void Gripper::grab_doneCb(const actionlib::SimpleClientGoalState& state,
			   const pr2_gripper_sensor_msgs::PR2GripperGrabResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void Gripper::grab_activeCb()
{
  ROS_INFO("Goal just went active");
}

void Gripper::grab_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperGrabFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback\n");
}

void Gripper::grab(){
  pr2_gripper_sensor_msgs::PR2GripperGrabGoal grip_cmd;
  grip_cmd.command.hardness_gain = grab_hardness_gain_;
  ROS_INFO("Sending grab goal");
  //gripper_client_->sendGoal(grip_cmd, &grab_doneCb, &grab_activeCb,&grab_feedbackCb);
  grab_client_->sendGoal(grip_cmd,
			    boost::bind(&Gripper::grab_doneCb, this, _1, _2),
			    boost::bind(&Gripper::grab_activeCb, this));
			    //boost::bind(&Gripper::grab_feedbackCb, this, _1));  //This feedback will spam your terminal and log files
}

void Gripper::grab_cancel(){
  grab_client_->cancelAllGoals();
}

// RELEASE

bool Gripper::release_isDone() {
  return (release_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState Gripper::release_getState() {
  return release_client_->getState();
}

void Gripper::release_doneCb(const actionlib::SimpleClientGoalState& state,
			   const pr2_gripper_sensor_msgs::PR2GripperReleaseResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void Gripper::release_activeCb()
{
  ROS_INFO("Goal just went active");
}

void Gripper::release_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperReleaseFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback\n");
}

void Gripper::release(){
  pr2_gripper_sensor_msgs::PR2GripperReleaseGoal release_cmd;
  release_cmd.command.event.trigger_conditions = place_trigger_conditions_;
  release_cmd.command.event.acceleration_trigger_magnitude = place_acceleration_trigger_magnitude_;
  release_cmd.command.event.slip_trigger_magnitude = place_slip_trigger_magnitude_;
  ROS_INFO("Sending release goal");
  //gripper_client_->sendGoal(grip_cmd, &grab_doneCb, &grab_activeCb,&grab_feedbackCb);
  release_client_->sendGoal(release_cmd,
			    boost::bind(&Gripper::release_doneCb, this, _1, _2),
			    boost::bind(&Gripper::release_activeCb, this));
			    //boost::bind(&Gripper::release_feedbackCb, this, _1));  //This feedback will spam your terminal and log files
}

void Gripper::release_cancel(){
  release_client_->cancelAllGoals();
}

// FORCE_SERVO
int8_t Gripper::getForceServoFingertipForce(){
  return fingertip_force_;
}

Gripper::ERROR Gripper::setForceServoFingertipForce(double fingertip_force){
  ERROR result=OK;
  if((fingertip_force>0) && (fingertip_force<=15))
    fingertip_force_=fingertip_force;
  else
    result = INVALID_PARAM;
  return result;
}

bool Gripper::forceServo_isDone() {
  return (force_client_->getState()).isDone();
}

actionlib::SimpleClientGoalState Gripper::forceServo_getState() {
  return force_client_->getState();
}

void Gripper::forceServo_doneCb(const actionlib::SimpleClientGoalState& state,
			   const pr2_gripper_sensor_msgs::PR2GripperForceServoResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void Gripper::forceServo_activeCb()
{
  ROS_INFO("Goal just went active");
}

void Gripper::forceServo_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperForceServoFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback\n");
}

void Gripper::forceServo(){
  pr2_gripper_sensor_msgs::PR2GripperForceServoGoal force_cmd;
  force_cmd.command.fingertip_force = fingertip_force_;
  ROS_INFO("Sending hold goal");
  force_client_->sendGoal(force_cmd,
			    boost::bind(&Gripper::forceServo_doneCb, this, _1, _2),
			    boost::bind(&Gripper::forceServo_activeCb, this));
			    //boost::bind(&Gripper::forceServo_feedbackCb, this, _1));   //This feedback will spam your terminal and log files
}

void Gripper::forceServo_cancel(){
  force_client_->cancelAllGoals();
}
