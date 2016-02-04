#ifndef _PR2_TORSO_CLIENT_H
#define _PR2_TORSO_CLIENT_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_model.hh>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

class Torso{
private:
  TorsoClient *torso_client_;
  double position_min_;
  double position_max_;
  ros::Duration min_duration_min_;
  ros::Duration min_duration_max_;
  ros::Duration min_duration_;
  double max_velocity_min_;
  double max_velocity_max_;
  double max_velocity_;
  // true if connected, false otherwise
  bool is_connected_;

public:
  enum ERROR { OK, INIT_FAILED, INIT_NOT_DONE, CANNOT_READ_LIMITS, UNKNOWN_JOINT, SERVER_NOT_CONNECTED, INVALID_PARAM, NB_ERROR };

  // Torso()
  // initialise limits values
  Torso();

  // ~Torso()
  // destroy the client
  ~Torso();

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


  // isConnected
  // Check if the server is connected, return OK if yes
  ERROR isConnected();

  // getMaxVelocityDefault
  double getMaxVelocity();
  
  // getMinDurationDefault
  ros::Duration getMinDuration();

  // checkParamLimits
  // check min_duration and max_velocity parameters limits
  ERROR checkParamLimits(ros::Duration, double);
  // checlCmdLimits
  // check torso position limits
  ERROR checkCmdLimits(double);


  // move_isDone
  bool move_isDone();

  // move_getState
  // get torso_client state
  actionlib::SimpleClientGoalState move_getState();
  
  // move_doneCb
  // callback launched when the movement is done
  void move_doneCb(const actionlib::SimpleClientGoalState&, const pr2_controllers_msgs::SingleJointPositionResultConstPtr&);

  // move_activeCb
  // callback launched when the client becomes active
  void move_activeCb();


  // move_feedbackCb
  // callback launched regularly by the server
  // it gives feedback information through 
  // pr2_controllers_msgs/SingleJointPositionFeedback feedback
  //  Header header
  //  float64 position
  //  float64 velocity
  //  float64 error
  void move_feedbackCb(const pr2_controllers_msgs::SingleJointPositionFeedbackConstPtr&);

  // move
  // move the torso to a given position
  // pr2_controllers_msgs::SingleJointPositionGoal 
  // float64 position
  // duration min_duration
  // float64 max_velocity
  //
  // Design
  // Check input parameters, return ERROR in case of problem
  // SendGoal to the controller
  ERROR move(pr2_controllers_msgs::SingleJointPositionGoal);

  // cancelCmd
  // cancel all commands send to the controller
  void cancelCmd();  

  // setTorsoMaxVelocity
  //
  ERROR setTorsoMaxVelocity(double);
};

#endif
