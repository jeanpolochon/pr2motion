// inspired from http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20Head

#ifndef _PR2_HEAD_CLIENT_H
#define _PR2_HEAD_CLIENT_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_model.hh>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class RobotHead
{
private:
  PointHeadClient* point_head_client_;
  double pan_min_;
  double pan_max_;
  double tilt_min_;
  double tilt_max_;
  ros::Duration min_duration_;
  ros::Duration min_duration_max_;  
  double max_velocity_;
  double max_velocity_max_;

public:
  enum ERROR { OK, INIT_FAILED, INIT_NOT_DONE, CANNOT_READ_LIMITS, UNKNOWN_JOINT, SERVER_NOT_CONNECTED, INVALID_PARAM, NB_ERROR };
  RobotHead();
  ~RobotHead();

  ERROR isConnected();

  ERROR checkCmdLimits(double, double);

  // listenerCallback
  // This callback aims to read the head desired position send by the controller and check if it is in the joint limits (seems there is no check at the controller level (sic !)
  // The problem with that solution is that it does not react enough quickly and the move begins...
  //
  // Design
  // Get the pan and tilt desired position through the /head_traj_controller/point_head_action topic
  // then check the limits
  // If one value is over limits, send a cancelAllGoals to the head_traj_controller
  void listenerCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr&);
  
  // init
  // Create the client
  // Initialize the connection with point_head_action server
  // It is a choice to separate this initialisation from the 
  //
  // Design
  // Create the client and initialize the connection to the server
  // Wait five seconds
  // Test if the server is connected, if not return ERROR
  ERROR init();

  // lookAt_isDone();
  bool lookAt_isDone();

  // lookAt_getState
  // Get the head_controller_client state
  actionlib::SimpleClientGoalState lookAt_getState();

  // i did not achieve to make these callbacks work
  void lookAt_doneCb(const actionlib::SimpleClientGoalState&,
		     const pr2_controllers_msgs::PointHeadActionResultConstPtr&);
  void lookAt_activeCb();
  void lookAt_feedbackCb(const pr2_controllers_msgs::PointHeadActionFeedbackConstPtr&);

  // lookAt
  // lookAt a given position
  // arg1 is the frame then x, y , z
  //
  // Design
  // Create a nodeHandle and a subscriber to the /head_traj_controller/state
  // Fill the goal with the inputs
  // Send the goal
  // Wait until it is finished or 2.0 seconds
  void lookAt(std::string, double, double, double);

  // cancelCmd
  // cancel cmd send to the head_controller_client
  void cancelCmd();

  ERROR setMaxVelocity(double);
};

#endif
