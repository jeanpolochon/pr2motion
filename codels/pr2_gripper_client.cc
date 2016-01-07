#include <pr2_gripper_client.hh>

// GripperSimple::GripperSimple(const std::string &name){
GripperSimple::GripperSimple(){
}
GripperSimple::~GripperSimple(){
    delete gripper_client_;
}

void  GripperSimple::init(GripperSimple::SIDE side){
  if (side==RIGHT) {
    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperSimpleClient("r_gripper_controller/gripper_action", true);
  } else {
    gripper_client_ = new GripperSimpleClient("l_gripper_controller/gripper_action", true);
  }

    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the gripper controller to come up\n");
    }
  }


  // CLOSE
actionlib::SimpleClientGoalState GripperSimple::close_getState() {
   return gripper_client_->getState();
 }
void GripperSimple::close_doneCb(const actionlib::SimpleClientGoalState& state,
	      const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr& result)
  {
    // pr2_controllers_msgs/Pr2GripperCommandResult result
    //  float64 position
    //  float64 effort
    //  bool stalled
    //  bool reached_goal
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  }
void GripperSimple::close_activeCb()
{
  ROS_INFO("Goal just went active");
}
void GripperSimple::close_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr& feedback)
{
  // pr2_controllers_msgs/Pr2GripperCommandFeedback feedback
  //   float64 position
  //   float64 effort
  //   bool stalled
  //   bool reached_goal
  ROS_INFO("Got Feedback\n");
}
void GripperSimple::close(pr2_controllers_msgs::Pr2GripperCommandGoal close_cmd){
    // pr2_controllers_msgs::Pr2GripperCommandGoal open;
    // open.command.position = 0.09;    // position open (9 cm)
    // open.command.max_effort = -1.0;  // unlimited motor effort
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
actionlib::SimpleClientGoalState GripperSimple::open_getState() {
  return gripper_client_->getState();
}
void GripperSimple::open_doneCb(const actionlib::SimpleClientGoalState& state,
	      const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr& result)
  {
    // pr2_controllers_msgs/Pr2GripperCommandResult result
    //  float64 position
    //  float64 effort
    //  bool stalled
    //  bool reached_goal
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  }
void GripperSimple::open_activeCb()
  {
    ROS_INFO("Goal just went active");
  }
void GripperSimple::open_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr& feedback)
  {
    // pr2_controllers_msgs/Pr2GripperCommandFeedback feedback
    //   float64 position
    //   float64 effort
    //   bool stalled
    //   bool reached_goal
    ROS_INFO("Got Feedback\n");
  }
void GripperSimple::open(pr2_controllers_msgs::Pr2GripperCommandGoal open_cmd){
    // pr2_controllers_msgs::Pr2GripperCommandGoal open;
    // open.command.position = 0.09;    // position open (9 cm)
    // open.command.max_effort = -1.0;  // unlimited motor effort
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open_cmd, 
			      boost::bind(&GripperSimple::open_doneCb, this, _1, _2), 
			      boost::bind(&GripperSimple::open_activeCb, this),
			      boost::bind(&GripperSimple::open_feedbackCb, this, _1));
  }
void GripperSimple::open_cancel(){
  gripper_client_->cancelAllGoals();
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



//   return 0;
// }
