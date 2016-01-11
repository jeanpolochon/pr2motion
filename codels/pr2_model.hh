#ifndef _PR2_MODEL_H
#define _PR2_MODEL_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <iostream>
#include <string>

class Pr2Model{
 public:
  enum ERROR {OK, INIT_NOT_DONE, UNABLE_TO_READ_DESCRIPTION, UNKNOWN_JOINT, NB_ERROR};
  Pr2Model();
  ~Pr2Model();
  ERROR getRobotModel();
  ERROR checkModel();
  ERROR checkJointName(std::string);
  double getJointLimitUpper(std::string);
  double getJointLimitLower(std::string);
  double getJointLimitVelocity(std::string);
  double getJointLimitEffort(std::string);
  
 private:
   // The object for urdf model
   boost::shared_ptr<urdf::Model> robot_model_;
};

#endif
