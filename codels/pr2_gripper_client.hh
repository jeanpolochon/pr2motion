#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandFeedback.h>
#include <pr2_controllers_msgs/Pr2GripperCommandResult.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperSimpleClient;
// Our Action interface type, provided as a typedef for convenience                   

class GripperSimple{
private:
  GripperSimpleClient* gripper_client_;  
  actionlib::SimpleClientGoalState::StateEnum close_state_;
  actionlib::SimpleClientGoalState::StateEnum open_state_;

public:
  enum SIDE { LEFT, RIGHT, NB_SIDE };

  //Action client initialization
  GripperSimple();
  ~GripperSimple();

  void init(SIDE);
  // Get client state
  //Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST. 
  //The goal's state. Returns LOST if this SimpleActionClient isn't tracking a goal. 
  
  // CLOSE
  actionlib::SimpleClientGoalState close_getState();
  void close_doneCb(const actionlib::SimpleClientGoalState&, const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr&);
  void close_activeCb();
  void close_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr&);
  void close(pr2_controllers_msgs::Pr2GripperCommandGoal);
  void close_cancel();

  // OPEN
  actionlib::SimpleClientGoalState open_getState();
  void open_doneCb(const actionlib::SimpleClientGoalState&, const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr&);
  void open_activeCb();
  void open_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr&);
  void open(pr2_controllers_msgs::Pr2GripperCommandGoal);
  void open_cancel();

};
