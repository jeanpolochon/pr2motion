#include <pr2_gripper_sensor_client.hh>

// Gripper::Gripper(const std::string &name){
Gripper::Gripper(){
}
Gripper::~Gripper(){
    delete gripper_client_;
    delete contact_client_;
    delete slip_client_;
    delete event_detector_client_;

  }

Gripper::ERROR Gripper::init(Gripper::SIDE side){
  ERROR result = OK;
  
  if((gripper_client_==NULL)||(contact_client_==NULL)||(slip_client_==NULL)||(event_detector_client_==NULL)){
    if (side==RIGHT) {
      //Initialize the client for the Action interface to the gripper controller
      //and tell the action client that we want to spin a thread by default
      gripper_client_ = new GripperClient("r_gripper_sensor_controller/gripper_action", true);
      contact_client_  = new ContactClient("r_gripper_sensor_controller/find_contact",true);
      slip_client_  = new SlipClient("r_gripper_sensor_controller/slip_servo",true);
      event_detector_client_  = new EventDetectorClient("r_gripper_sensor_controller/event_detector",true);
    } else {
      gripper_client_ = new GripperClient("l_gripper_sensor_controller/gripper_action", true);
      contact_client_  = new ContactClient("l_gripper_sensor_controller/find_contact",true);
      slip_client_  = new SlipClient("l_gripper_sensor_controller/slip_servo",true);
      event_detector_client_  = new EventDetectorClient("l_gripper_sensor_controller/event_detector",true);
    }
  }

  if((gripper_client_!=NULL)&&(contact_client_!=NULL)&&(slip_client_!=NULL)&&(event_detector_client_!=NULL)){
    if(!gripper_client_->isServerConnected()){
      //wait for the gripper action server to come up 
      while(!gripper_client_->waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the r_gripper_sensor_controller/gripper_action action server to come up");
      }
      if(!gripper_client_->isServerConnected()){
	result=SERVER_NOT_CONNECTED;
      }
    }
    if(!contact_client_->isServerConnected()){
      while(!contact_client_->waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the r_gripper_sensor_controller/find_contact action server to come up");
      }
      if(!contact_client_->isServerConnected()){
	result = SERVER_NOT_CONNECTED;
      }
    }
    if(!slip_client_->isServerConnected()){
      while(!slip_client_->waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the r_gripper_sensor_controller/slip_servo action server to come up");
      }    
      if(!slip_client_->isServerConnected()){
	result = SERVER_NOT_CONNECTED;
      }
    }
    if(!event_detector_client_->isServerConnected()){
      while(!event_detector_client_->waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the r_gripper_sensor_controller/event_detector action server to come up");
      }    
      if(!event_detector_client_->isServerConnected()){
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
    // pr2_controllers_msgs/Pr2GripperCommandResult result
    //  float64 position
    //  float64 effort
    //  bool stalled
    //  bool reached_goal
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  }
void Gripper::close_activeCb()
{
  ROS_INFO("Goal just went active");
}
void Gripper::close_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr& feedback)
{
  // pr2_controllers_msgs/Pr2GripperCommandFeedback feedback
  //   float64 position
  //   float64 effort
  //   bool stalled
  //   bool reached_goal
  ROS_INFO("Got Feedback\n");
}
void Gripper::close(pr2_controllers_msgs::Pr2GripperCommandGoal close_cmd){
    // pr2_controllers_msgs::Pr2GripperCommandGoal open;
    // open.command.position = 0.09;    // position open (9 cm)
    // open.command.max_effort = -1.0;  // unlimited motor effort
    ROS_INFO("Sending close goal");
    //gripper_client_->sendGoal(close, &close_doneCb, &close_activeCb,&close_feedbackCb);
    gripper_client_->sendGoal(close_cmd, 
			      boost::bind(&Gripper::close_doneCb, this, _1, _2), 
			      boost::bind(&Gripper::close_activeCb, this),
			      boost::bind(&Gripper::close_feedbackCb, this, _1));
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
    // pr2_controllers_msgs/Pr2GripperCommandResult result
    //  float64 position
    //  float64 effort
    //  bool stalled
    //  bool reached_goal
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  }
void Gripper::open_activeCb()
  {
    ROS_INFO("Goal just went active");
  }
void Gripper::open_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr& feedback)
  {
    // pr2_controllers_msgs/Pr2GripperCommandFeedback feedback
    //   float64 position
    //   float64 effort
    //   bool stalled
    //   bool reached_goal
    ROS_INFO("Got Feedback\n");
  }
void Gripper::open(pr2_controllers_msgs::Pr2GripperCommandGoal open_cmd){
    // pr2_controllers_msgs::Pr2GripperCommandGoal open;
    // open.command.position = 0.09;    // position open (9 cm)
    // open.command.max_effort = -1.0;  // unlimited motor effort
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open_cmd, 
			      boost::bind(&Gripper::open_doneCb, this, _1, _2), 
			      boost::bind(&Gripper::open_activeCb, this),
			      boost::bind(&Gripper::open_feedbackCb, this, _1));
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

void Gripper::findTwoContacts(pr2_gripper_sensor_msgs::PR2GripperFindContactGoal findTwo_cmd){
    // findTwo.command.contact_conditions = findTwo.command.BOTH;  // close until both fingers contact
    // findTwo.command.zero_fingertip_sensors = true;   // zero fingertip sensor values before moving
 
    ROS_INFO("Sending find 2 contact goal");
    // contact_client_->sendGoal(findTwo, &findTwo_doneCb, &findTwo_activeCb, &findTwo_feedbackCb);
    //    contact_client_->sendGoal(findTwo);
    contact_client_->sendGoal(findTwo_cmd, 
			      boost::bind(&Gripper::findTwo_doneCb, this, _1, _2), 
			      boost::bind(&Gripper::findTwo_activeCb, this),
			      boost::bind(&Gripper::findTwo_feedbackCb, this, _1));
    // contact_client_->waitForResult(ros::Duration(5.0));
    // if(contact_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //   ROS_INFO("Contact found. Left: %d, Right: %d", contact_client_->getResult()->data.left_fingertip_pad_contact, 
    //            contact_client_->getResult()->data.right_fingertip_pad_contact);
    //   ROS_INFO("Contact force. Left: %f, Right: %f", contact_client_->getResult()->data.left_fingertip_pad_force, 
    //        contact_client_->getResult()->data.right_fingertip_pad_force);
    // }
    // else
    //   ROS_INFO("The gripper did not find a contact.");
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
			      boost::bind(&Gripper::slipServo_activeCb, this),
			      boost::bind(&Gripper::slipServo_feedbackCb, this, _1));
  }  
void Gripper::slipServo_cancel(){
  slip_client_->cancelAllGoals();
}
 
  //
  // PLACE
  // move into event_detector mode to detect object contact
  //
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

void Gripper::place(pr2_gripper_sensor_msgs::PR2GripperEventDetectorGoal place_cmd){
    // place_goal.command.trigger_conditions = place_goal.command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;  
    // place_goal.command.acceleration_trigger_magnitude = 4.0;  // set the contact acceleration to n m/s^2
    // place_goal.command.slip_trigger_magnitude = .005;
    ROS_INFO("Waiting for object placement contact...");
    event_detector_client_->sendGoal(place_cmd, 
				     boost::bind(&Gripper::place_doneCb,  this, _1, _2), 
				     boost::bind(&Gripper::place_activeCb, this),
				     boost::bind(&Gripper::place_feedbackCb, this, _1));
    //event_detector_client_->sendGoal(place_goal);


  //   event_detector_client_->waitForResult();
  //   if(event_detector_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //   {
  //     ROS_INFO("Place Success");
  //     ROS_INFO("cond met: %d, acc_met: %d, slip_met: %d", event_detector_client_->getResult()->data.trigger_conditions_met, event_detector_client_->getResult()->data.acceleration_event, event_detector_client_->getResult()->data.slip_event);
  //   }
  //   else
  //     ROS_INFO("Place Failure");
} 

void Gripper::place_cancel(){
  event_detector_client_->cancelAllGoals();
}

// int main(int argc, char** argv){
//   ros::init(argc, argv, "simple_gripper");

//   Gripper gripper;

//     // open the hand to prepare for a grasp
//     gripper.open();
  
//     // wait (you don't have to do this in your code)
//     // in this demo here is a good time to put an object inside the gripper
//     // in your code the robot would move its arm around an object
//     sleep(7.0);

//     // close the gripper until we have a contact on each finger and move into force control mode
//     gripper.findTwoContacts();

//     // now hold the object but don't drop it
//     gripper.slipServo();

//     // now we wait some amount of time until we want to put it down
//     // in your code you probably wouldn't do this but you would move the arm around instead
//     sleep(8.0);

//     // now we decided we are ready to put the  object down, so tell the gripper that
//     gripper.place();
    
//     // open the gripper once placement has been detected
//     gripper.open();

//   return 0;
// }