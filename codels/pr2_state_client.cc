#include "pr2_state_client.hh"

RobotState::RobotState(){}

RobotState::~RobotState(){
}

RobotState::ERROR RobotState::setRobotQ(sensor_msgs::JointState input_state) {
  ERROR result=OK;
  int name_size=0;
  int position_size=0;
  int velocity_size=0;
  int effort_size=0;

  name_size = input_state.name.size();
  position_size = input_state.position.size();
  velocity_size = input_state.velocity.size();
  effort_size = input_state.effort.size();
  
  if(name_size==0){
    printf("the state is empty !");
    return INVALID_STATE;
  }
      
  if( (name_size!=position_size) ||
      (name_size!=velocity_size) ||
      (name_size!=effort_size)){
    printf("issue concerning the size of the state vector");
    return INVALID_STATE;
  }

  state_.name.resize(name_size);
  state_.position.resize(position_size);
  state_.velocity.resize(velocity_size);
  state_.effort.resize(effort_size);

  for (size_t ind=0; ind<name_size; ind++) {
    state_.name[ind]=input_state.name[ind];
    state_.position[ind]=input_state.position[ind];
    state_.velocity[ind]=input_state.velocity[ind];
    state_.effort[ind]=input_state.effort[ind];
  }
  
  return result;

}
