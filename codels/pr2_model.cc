#include "pr2_model.hh"

Pr2Model::Pr2Model()
  : robot_model_(boost::shared_ptr<urdf::Model>(new urdf::Model()))
{}

Pr2Model::~Pr2Model(){}

Pr2Model::ERROR Pr2Model::getRobotModel(){
  ERROR result = OK;
  ros::NodeHandle node;
  std::string robot_desc_string;
  node.param("robot_description", robot_desc_string, std::string());

  if(!robot_model_->initString(robot_desc_string)){
    printf("Pr2Model::getRobotModel() : unable to read description");
    result = UNABLE_TO_READ_DESCRIPTION;
  }
  return result;
}

Pr2Model::ERROR Pr2Model::checkModel(){
  ERROR result = OK;
  if(!robot_model_)
    result = INIT_NOT_DONE;
  return result;
}

Pr2Model::ERROR Pr2Model::checkJointName(std::string joint_name){
  ERROR result = OK;
  const urdf::Joint* joint = robot_model_->getJoint(joint_name).get();
  if(!joint)
    result = UNKNOWN_JOINT;
  return result;
}
 
double Pr2Model::getJointLimitUpper(std::string joint_name){
  const urdf::Joint* joint = robot_model_->getJoint(joint_name).get();
  const boost::shared_ptr<urdf::JointSafety> safety = joint->safety;
  const boost::shared_ptr<urdf::JointLimits> limits = joint->limits;  
  //return limits->upper;  
  return safety->soft_upper_limit;
}

double Pr2Model::getJointLimitLower(std::string joint_name){
  const urdf::Joint* joint = robot_model_->getJoint(joint_name).get();
  const boost::shared_ptr<urdf::JointSafety> safety = joint->safety;
  const boost::shared_ptr<urdf::JointLimits> limits = joint->limits;  
  //return limits->lower;  
  return safety->soft_lower_limit;
}

double Pr2Model::getJointLimitVelocity(std::string joint_name){
  const urdf::Joint* joint = robot_model_->getJoint(joint_name).get();
  const boost::shared_ptr<urdf::JointSafety> safety = joint->safety;
  const boost::shared_ptr<urdf::JointLimits> limits = joint->limits;  
  return limits->velocity;  
}

double Pr2Model::getJointLimitEffort(std::string joint_name){
  const urdf::Joint* joint = robot_model_->getJoint(joint_name).get();
  const boost::shared_ptr<urdf::JointLimits> limits = joint->limits;  
  return limits->effort;  
}




//   const urdf::Joint* joint = robot_model_->getJoint("torso_lift_joint").get();
//   ERROR result = OK;
//   const urdf::Joint* joint = robot_model_->getJoint(joint_name).get();
  
//   if(joint!=NULL) {
//     urdf::Joint::type joint_type = joint->type;
// //     //int joint_type = joint->type;
    
// //     // there is no lower and upper bound for CONTINUOUS joint
//     if(joint_type != urdf::Joint::CONTINUOUS) {
//       limits->lower = joint->limits.lower;
//       limits->upper = joint->limits.upper;
//     }
    
//      limits->velocity = joint->limits.effort;
//           limits->effort = joint->limits.velocity;
//   } else {
//      printf("unknown joint \n");
//      result = UNKNOWN_JOINT;
//    }
//    return result;
//  } 

// Pr2Model::ERROR Pr2Model::getJointType(std::string joint_name, urdf::Joint::type){
//   //const urdf::Joint* joint = robot_model_->getJoint("torso_lift_joint").get();
//   ERROR result = OK;
//   const urdf::Joint* joint = robot_model_->getJoint(joint_name).get();  
//   if(joint!=NULL) {
//     joint_type = joint->type;
//   } else {
//     printf("unknown joint \n");
//     result = UNKNOWN_JOINT;
//   }
//   return result;
// } 
