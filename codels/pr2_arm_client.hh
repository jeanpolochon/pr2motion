#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

// to be able to use gatech trajectory computation
#include <Eigen/Core>
#include "trajectories/TrajectoryG.h"
#include "trajectories/PathG.h"
using namespace std;
using namespace Eigen;

// to be able to use softmotion trajectory computation
#include "softMotion/Sm_Traj.h"
#include "softMotion/softMotion.h"

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

// this class deals with PR2 arm trajectory computation and execution

class RobotArm{
public:
  enum ERROR { OK, INIT_FAILED, INVALID_PARAM, INVALID_TRAJ, TRAJECTORY_COMPUTATION_FAILED, NB_ERROR };
  enum SIDE { LEFT, RIGHT, NB_SIDE };
  enum TRAJ_MODE { SOFT_MOTION, GATECH, PATH, TEST, NB_MODE };
  RobotArm();
  ~RobotArm();

  // init
  // this function initializes the action with the correct controller (ie right/left)
  ERROR init(SIDE);

  // clearTrajectory
  // this function clear arm_traj_
  void clearTrajectory();

  // setTraj
  // this function copies the trajectory given in parameter in arm_traj_
  ERROR setTraj(pr2_controllers_msgs::JointTrajectoryGoal *);

  // setTrajMode
  // this function sets the trajectory mode
  ERROR setTrajMode(TRAJ_MODE);

  // setMax
  // this function helps to set max_vel_, max_acc_, max_jerk_
  ERROR setMax(double, double, double);
  // setT
  // this function sets trajectory time slot
  ERROR setT(double);

  // validateTraj
  // this function helps to check the trajectory regarding the joint constraints
  // returns:
  // OK if the traj is considered as valid
  // INVALID_TRAJ otherwise
  ERROR validateTrajectory();
  ERROR validateTrajectory(pr2_controllers_msgs::JointTrajectoryGoal *);

  // computeTrajectory
  // this function computes a trajectory from a path
  // if no parameters are given it takes as input arm_traj_ as path and write the result also in arm_traj_
  // the way the trajectory is generated is choosen given the traj_mode_ value
  ERROR computeTrajectory();
  // computeTrajectoryG
  // this function computes a trajectory using gatech software
  ERROR computeTrajectoryG();
  ERROR computeTrajectoryG(pr2_controllers_msgs::JointTrajectoryGoal *, pr2_controllers_msgs::JointTrajectoryGoal *);
  // computeTrajectorySoftMotion
  // this function computes a trajectory using soft motion
  ERROR computeTrajectorySoftMotion();
  ERROR computeTrajectorySoftMotion(pr2_controllers_msgs::JointTrajectoryGoal *, pr2_controllers_msgs::JointTrajectoryGoal *);

  // move_getState
  // this function helps to get the state of the action
  actionlib::SimpleClientGoalState move_getState();
  // move
  // this function launch the action by sending the goal
  void move();
  void move(pr2_controllers_msgs::JointTrajectoryGoal *);
  //pr2_controllers_msgs::JointTrajectoryGoal getgenom2Traj(SIDE);
  void gettestPath();
  void gettestPath(pr2_controllers_msgs::JointTrajectoryGoal *);
 
private:
  TrajClient* traj_client_;
  SIDE arm_side_;
  TRAJ_MODE traj_mode_;
  pr2_controllers_msgs::JointTrajectoryGoal arm_traj_;
  double max_acc_;
  double max_vel_;
  double max_jerk_;
  double time_slot_;
};
