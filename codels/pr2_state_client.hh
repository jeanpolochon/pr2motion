#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class RobotState{
public:
  enum ERROR { OK, INVALID_STATE, NB_ERROR};
  RobotState();
  ~RobotState();
  ERROR getRobotQ();
  ERROR setRobotQ(sensor_msgs::JointState);

private:
  sensor_msgs::JointState state_;
};
  
