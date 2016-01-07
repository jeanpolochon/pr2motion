#include <pr2_head_client.hh>

RobotHead::RobotHead()
  : pan_min_(-2.8),
    pan_max_(2.8),
    tilt_min_(-0.37),
    tilt_max_(1.29)
{
}

RobotHead::~RobotHead(){
  delete point_head_client_;
}

void RobotHead::listenerCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
  int count=0;
   // pr2_controllers_msgs::JointTrajectoryControllerStat
   //     std_msgs/Header header
   //     string[] joint_names
   //     trajectory_msgs/JointTrajectoryPoint desired
   //     trajectory_msgs/JointTrajectoryPoint actual
   //     trajectory_msgs/JointTrajectoryPoint error
   // trajectory_msgs/JointTrajectoryPoint
   //        float64[] positions
   //        float64[] velocities
   //        float64[] accelerations
   //        float64[] effort
   //        duration time_from_start

  // check pan boundaries
   if(msg->joint_names[0].compare("head_pan_joint")==0) {
     count = count+1;
     if(msg->desired.positions[0]>pan_max_ || msg->desired.positions[0]<pan_min_) {
       ROS_INFO("head desired position out of bound with pan = %f (limits [-2.8 , 2.8] ) \n", msg->desired.positions[0]);
       point_head_client_->cancelAllGoals();
     }
   }

   if(msg->joint_names[1].compare("head_pan_joint")==0) {
     count = count + 1;
     if(msg->desired.positions[1]>pan_max_ || msg->desired.positions[1]<pan_min_) {
       ROS_INFO("head desired position out of bound with pan = %f (limits [-2.8 , 2.8] ) \n", msg->desired.positions[1]);
       point_head_client_->cancelAllGoals();
     }
   }

   // check tilt boundaries
   if(msg->joint_names[0].compare("head_tilt_joint")==0) {
     count = count +1;
     if(msg->desired.positions[0]>tilt_max_ || msg->desired.positions[0]<tilt_min_) {
       ROS_INFO("head desired position out of bound with tilt = %f (limits [-0.37, 1.29])\n", msg->desired.positions[0]);
       point_head_client_->cancelAllGoals();
     }
   }

   if(msg->joint_names[1].compare("head_tilt_joint")==0) {
     count = count +1;
     if(msg->desired.positions[1]>tilt_max_ || msg->desired.positions[1]<tilt_min_) {
       ROS_INFO("head desired position out of bound with tilt = %f (limits [-0.37, 1.29])\n", msg->desired.positions[1]);
       point_head_client_->cancelAllGoals();
     }
   }

   // check if we have read pan and tilt data
   if(count !=2)
     ROS_INFO("ERROR: we do not get all head bounds !!!");

}

RobotHead::ERROR RobotHead::init(){
  ERROR result=OK;
  //Initialize the client for the Action interface to the head controller
  point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
  
  //wait for head controller action server to come up 
  while(!point_head_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the point_head_action server to come up");
  }

  if(!point_head_client_->isServerConnected())
    result=INIT_FAILED;

  return result;
}

actionlib::SimpleClientGoalState RobotHead::lookAt_getState() {
  return point_head_client_->getState();
}
void RobotHead::lookAt_doneCb(const actionlib::SimpleClientGoalState& state,
	      const pr2_controllers_msgs::PointHeadActionResultConstPtr& result)
  {
    // pr2_controllers_msgs/PointHeadActionResult result
    // Header header
    // actionlib_msgs/GoalStatus status
    // PointHeadResult result    
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  }

void RobotHead::lookAt_activeCb()
  {
    ROS_INFO("Goal just went active");
  }

void RobotHead::lookAt_feedbackCb(const pr2_controllers_msgs::PointHeadActionFeedbackConstPtr& feedback)
  {
    // pr2_controllers_msgs/SingleJointPositionFeedback feedback
    ROS_INFO("Got Feedback\n");
  }

void RobotHead::lookAt(std::string frame_id, double x, double y, double z){
  // create a nodehandle and a subscriber to read the desired position
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/head_traj_controller/state", 1000, &RobotHead::listenerCallback, this);
  //the goal message
  //pr2_controllers_msgs::PointHeadGoal
  // geometry_msgs/PointStamped target
  // geometry_msgs/Vector3 pointing_axis
  // string pointing_frame
  // duration min_duration
  // float64 max_velocity
  pr2_controllers_msgs::PointHeadGoal goal_cmd;

  //the target point, expressed in the requested frame
  geometry_msgs::PointStamped point;
  point.header.frame_id = frame_id;
  point.point.x = x; point.point.y = y; point.point.z = z;
  goal_cmd.target = point;
  
  //we are pointing the high-def camera frame 
  //(pointing_axis defaults to X-axis)
  goal_cmd.pointing_frame = "high_def_frame";
  goal_cmd.pointing_axis.x = 1.0;
  goal_cmd.pointing_axis.y = 0.0;
  goal_cmd.pointing_axis.z = 0.0;
  //take at least 0.5 seconds to get there
  goal_cmd.min_duration = ros::Duration(0.5);
  
  //and go no faster than 1 rad/s
  goal_cmd.max_velocity = 1.0;
  
  //send the goal
  point_head_client_->sendGoal(goal_cmd);
  point_head_client_->waitForResult(ros::Duration(2.0));


  //send the goal with callback (never achieved to make it work with that client)
  // point_head_client_->sendGoal(goal_cmd,
  //   boost::bind(&RobotHead::lookAt_doneCb, this, _1, _2), 
  //   boost::bind(&RobotHead::lookAt_activeCb, this),
  //   boost::bind(&RobotHead::lookAt_feedbackCb, this, _1));
    
}

void RobotHead::cancelCmd(){
  point_head_client_->cancelAllGoals();
}
