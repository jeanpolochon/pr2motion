#ifndef _PR2_GRIPPERSIMPLE_CLIENT_H
#define _PR2_GRIPPERSIMPLE_CLIENT_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandFeedback.h>
#include <pr2_controllers_msgs/Pr2GripperCommandResult.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_model.hh>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperSimpleClient;
// Our Action interface type, provided as a typedef for convenience

class GripperSimple{
private:
  GripperSimpleClient* gripper_client_;
  actionlib::SimpleClientGoalState::StateEnum close_state_;
  actionlib::SimpleClientGoalState::StateEnum open_state_;
  double open_position_;
  double open_position_max_;
  double open_position_min_;
  double open_max_effort_;
  double open_max_effort_max_;
  double close_position_;
  double close_position_max_;
  double close_position_min_;
  double close_max_effort_;
  double close_max_effort_max_;

public:
  enum SIDE { LEFT, RIGHT, NB_SIDE };
  enum ERROR { OK, INIT_FAILED, INIT_NOT_DONE, CANNOT_READ_LIMITS, UNKNOWN_JOINT, SERVER_NOT_CONNECTED, INVALID_PARAM, NB_ERROR };

  //Action client initialization
  GripperSimple();
  ~GripperSimple();

  ERROR init(SIDE);

  // Get client state
  //Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.
  //The goal's state. Returns LOST if this SimpleActionClient isn't tracking a goal.
    ERROR isConnected();

  // CLOSE
  ERROR setClosePosition(double);
  double getClosePosition();
  ERROR setCloseMaxEffort(double);
  double getCloseMaxEffort();
  bool close_isDone();
  actionlib::SimpleClientGoalState close_getState();
  void close_doneCb(const actionlib::SimpleClientGoalState&, const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr&);
  void close_activeCb();
  void close_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr&);
  void close();
  void close_cancel();

  // OPEN
  ERROR setOpenPosition(double);
  double getOpenPosition();
  ERROR setOpenMaxEffort(double);
  double getOpenMaxEffort();
  bool open_isDone();
  actionlib::SimpleClientGoalState open_getState();
  void open_doneCb(const actionlib::SimpleClientGoalState&, const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr&);
  void open_activeCb();
  void open_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr&);
  void open();
  void open_cancel();

};
#endif
