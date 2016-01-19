/*
 * Copyright (c) 2015 CNRS/LAAS
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include "acpr2motion.h"

#include "pr2motion_c_types.h"

// to be able to use str compare
#include <iostream>
#include <string>
// to be able to use vector
#include <vector>

#define PR2_SIMU

#ifndef PR2_SIMU
#include "pr2_gripper_sensor_client.hh"
#else
#include "pr2_gripper_client.hh"
#endif
#include "pr2_torso_client.hh"
#include "pr2_head_client.hh"
#include "pr2_arm_client.hh"
#include "pr2_state_client.hh"

//#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

/* --- Task main -------------------------------------------------------- */

#ifndef PR2_SIMU
// we are on the real robot
Gripper left_gripper;
Gripper right_gripper;
#else
// we are in simulation
GripperSimple left_gripper;
GripperSimple right_gripper;
#endif

Torso torso;
RobotHead head;
RobotArm left_arm;
RobotArm right_arm;
//RobotState robot_state; 

Pr2Model pr2_model;

/* 
StateEnum {
  PENDING, ACTIVE, RECALLED, REJECTED,
  PREEMPTED, ABORTED, SUCCEEDED, LOST
}
*/


/* --- Task main -------------------------------------------------------- */


/** Codel initMain of task main.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_routine.
 */
genom_event
initMain(double *open_position, double *open_max_effort,
         double *close_position, double *close_max_effort,
         pr2motion_GRIPPER_CONTACT_CONDITIONS *findtwo_contact_conditions,
         bool *findtwo_zero_fingertip_sensors,
         pr2motion_GRIPPER_EVENT_DETECTOR *event_trigger_conditions,
         double *event_acceleration_trigger_magnitude,
         double *event_slip_trigger_magnitude,
         bool *joint_state_availability, pr2motion_SIDE *left_side,
         pr2motion_SIDE *right_side, genom_context self)
{
  char* argv[] =  { "pr2motion",
                    "",
                    NULL};
  int argc = 2;
  ros::init(argc, argv, "pr2motion");
  // at the beginning the joint_state is not yet available
  *joint_state_availability = false;
  // gripper param initialisation
  *open_position = 0.09;
  // unlimited
  *open_max_effort=-1.0;
  *close_position = 0.0;
  // close gently
  *close_max_effort = -1.0;
  // close until both fingers contact
  *findtwo_contact_conditions = pr2motion_GRIPPER_CONTACT_BOTH;
  // zero fingertip sensor values before moving
  *findtwo_zero_fingertip_sensors = true;
  *event_trigger_conditions = pr2motion_GRIPPER_FINGER_SIDE_IMPACT_OR_ACC;
  *event_acceleration_trigger_magnitude = 4.0;
  *event_slip_trigger_magnitude = 0.005;
  *left_side=pr2motion_LEFT;
  *right_side=pr2motion_RIGHT;
  return pr2motion_routine;
}


/** Codel routineMain of task main.
 *
 * Triggered by pr2motion_routine.
 * Yields to pr2motion_routine, pr2motion_stop.
 */
genom_event
routineMain(const pr2motion_joint_state *joint_state,
            pr2motion_sensor_msgs_jointstate *joint_state_msg,
            bool *joint_state_availability, genom_context self)
{
  int name_size=0;
  int position_size=0;
  int velocity_size=0;
  int effort_size=0;

  //sensor_msgs::JointState joint_state_msg;
  // read the trajectory from the port
  joint_state->read(self);
  if(joint_state->data(self)!=NULL){
    // nb elements in joint_state
    name_size = joint_state->data(self)->name._length;
    position_size = joint_state->data(self)->position._length;
    velocity_size = joint_state->data(self)->velocity._length;
    effort_size = joint_state->data(self)->effort._length;

    if(name_size==0){
      printf("pr2motion The joint_state is empty\n");
      *joint_state_availability=false;
      return pr2motion_routine;
    }
    
    if( (name_size!=position_size) ||
	(name_size!=velocity_size) ||
	(name_size!=effort_size)){
      printf("pr2motion There is an issue concerning the size of the joint_state vector\n");
      *joint_state_availability = false;
      return pr2motion_routine;
    }
    if(joint_state_msg->name._maximum<name_size) {
      printf("pr2motion Need to add joint(s) to the sequence\n");
      genom_sequence_reserve(&(joint_state_msg->name), name_size);
      joint_state_msg->name._length=name_size;
      genom_sequence_reserve(&(joint_state_msg->position), position_size);
      joint_state_msg->position._length=position_size;
      genom_sequence_reserve(&(joint_state_msg->velocity), velocity_size);
      joint_state_msg->velocity._length=velocity_size;
      genom_sequence_reserve(&(joint_state_msg->effort), effort_size);
      joint_state_msg->effort._length=effort_size;
    }
    for (size_t ind=0; ind<name_size; ind++) {
      if(joint_state_msg->name._buffer[ind])
	free(joint_state_msg->name._buffer[ind]);
      joint_state_msg->name._buffer[ind]=strdup(joint_state->data(self)->name._buffer[ind]);
      joint_state_msg->position._buffer[ind]=joint_state->data(self)->position._buffer[ind];
      joint_state_msg->velocity._buffer[ind]=joint_state->data(self)->velocity._buffer[ind];
      joint_state_msg->effort._buffer[ind]=joint_state->data(self)->effort._buffer[ind];
      }
    *joint_state_availability = true;
    //robot_state.setRobotQ(joint_state_msg);
  } else {
    if(*joint_state_availability == true)
      printf("pr2motion nothing to read on the port...\n");
    *joint_state_availability = false;
  }
  return pr2motion_routine;
}


/** Codel endMain of task main.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 */
genom_event
endMain(genom_context self)
{
 return pr2motion_ether;
}


/* --- Activity Init ---------------------------------------------------- */

/** Codel initConnect of activity Init.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_ether.
 * Throws pr2motion_unable_to_read_description.
 */
genom_event
initConnect(genom_context self)
{
  ros::Duration torso_duration;

  // Pr2 Model
  Pr2Model::ERROR pr2model_init_result;
  pr2model_init_result=pr2_model.getRobotModel();
  if(pr2model_init_result!=Pr2Model::OK) {
    return pr2motion_unable_to_read_description(self);
  }

  // Torso Initialisation
  Torso::ERROR torso_init_result;
  torso_init_result=torso.init();
  if(torso_init_result==Torso::OK) {
    // torso_duration = torso.getMinDurationDefault();
    // torso_min_duration = torso_duration.toSec();
    // printf("initialisation of the torso with : sec = %d and nsec = %d so torso_min_duration = %f \n",torso_duration.sec, torso_duration.nsec, torso_min_duration);
    // torso_max_velocity = torso.getMaxVelocityDefault();  
  } else {
    printf("pr2motion::initConnect: WARNING: Torso initialisation failed, you won't be able to use it! \n");
  }

  // Head Initialisation
  RobotHead::ERROR head_init_result;
  head_init_result=head.init();  
  if(head_init_result!=RobotHead::OK) {
    printf("pr2motion::initConnect: WARNING: Head Initialisation failed, you won't be able to use it! \n");
  }

  RobotArm::ERROR right_arm_init_result;
  right_arm_init_result = right_arm.init(RobotArm::RIGHT);
  if(right_arm_init_result!=RobotArm::OK) {
    printf("pr2motion::initConnect: WARNING: right arm Initialisation failed, you won't be able to use it! \n");
  }

  RobotArm::ERROR left_arm_init_result;
  left_arm_init_result = left_arm.init(RobotArm::LEFT);
  if(left_arm_init_result!=RobotArm::OK) {
    printf("pr2motion::initConnect: WARNING: left arm Initialisation failed, you won't be able to use it! \n");
  } 

#ifndef PR2_SIMU
  Gripper::ERROR left_gripper_init_result;
  left_gripper_init_result=left_gripper.init(Gripper::LEFT);
  if(left_gripper_init_result!=Gripper::OK) {
    printf("pr2motion::initConnect: WARNING: left gripper Initialisation failed, you won't be able to use it! \n");
  }
  Gripper::ERROR right_gripper_init_result;
  right_gripper_init_result=right_gripper.init(Gripper::RIGHT);
  if(right_gripper_init_result!=Gripper::OK) {
    printf("pr2motion::initConnect: WARNING: left gripper Initialisation failed, you won't be able to use it! \n");
  }  
#else
  GripperSimple::ERROR left_gripper_init_result;
  left_gripper_init_result=left_gripper.init(GripperSimple::LEFT);
  if(left_gripper_init_result!=GripperSimple::OK) {
    printf("pr2motion::initConnect: WARNING: left gripper Initialisation failed, you won't be able to use it! \n");
  }  
  GripperSimple::ERROR right_gripper_init_result;
  right_gripper_init_result=right_gripper.init(GripperSimple::RIGHT);
  if(right_gripper_init_result!=GripperSimple::OK) {
    printf("pr2motion::initConnect: WARNING: left gripper Initialisation failed, you won't be able to use it! \n");
  } 
#endif
  
  return pr2motion_ether;
}


/* --- Activity Gripper_Operate ----------------------------------------- */

/** Codel startOperateGripper of activity Gripper_Operate.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_exec, pr2motion_stop, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
startOperateGripper(pr2motion_SIDE side,
                    pr2motion_GRIPPER_MODE goal_mode,
                    genom_context self)
{
  // check parameters and whether the corresponding gripper client is connected

#ifndef PR2_SIMU 
  Gripper::ERROR result_connect;
  switch(side){
  case pr2motion_LEFT :
    result_connect = left_gripper.isConnected();
    break;
  case pr2motion_RIGHT :
    result_connect = right_gripper.isConnected();
    break;    
  default:
    return pr2motion_invalid_param(self);
  }
  switch(result_connect){
  case Gripper::OK:
    return pr2motion_exec;
  case Gripper::INIT_NOT_DONE:
    return pr2motion_init_not_done(self);
  case Gripper::SERVER_NOT_CONNECTED:
    return pr2motion_not_connected(self);
  default:
    return pr2motion_unknown_error(self);
  }
#else
  GripperSimple::ERROR result_connect;
  switch(side){
  case pr2motion_LEFT :
    result_connect = left_gripper.isConnected();
    break;
  case pr2motion_RIGHT :
    result_connect = right_gripper.isConnected();
    break;    
  default:
    return pr2motion_invalid_param(self);
  }
  switch(result_connect){
  case GripperSimple::OK:
    return pr2motion_exec;
  case GripperSimple::INIT_NOT_DONE:
    return pr2motion_init_not_done(self);
  case GripperSimple::SERVER_NOT_CONNECTED:
    return pr2motion_not_connected(self);
  default:
    return pr2motion_unknown_error(self);
  }
#endif
}

/** Codel execOperateGripper of activity Gripper_Operate.
 *
 * Triggered by pr2motion_exec.
 * Yields to pr2motion_exec, pr2motion_wait, pr2motion_waitcontact,
 *           pr2motion_waitopen, pr2motion_waitclose,
 *           pr2motion_waitrelease, pr2motion_stop, pr2motion_end,
 *           pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */

genom_event
execOperateGripper(pr2motion_SIDE side,
                   pr2motion_GRIPPER_MODE goal_mode,
                   double open_position, double open_max_effort,
                   double close_position, double close_max_effort,
                   pr2motion_GRIPPER_CONTACT_CONDITIONS findtwo_contact_conditions,
                   bool findtwo_zero_fingertip_sensors,
                   pr2motion_GRIPPER_EVENT_DETECTOR event_trigger_conditions,
                   double event_acceleration_trigger_magnitude,
                   double event_slip_trigger_magnitude,
                   genom_context self)
{
  // we are on the real robot
#ifndef PR2_SIMU
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = open_position;    // position open (9 cm)
  open.command.max_effort = open_max_effort;  // unlimited motor effort
  pr2_controllers_msgs::Pr2GripperCommandGoal close;
  close.command.position = close_position;    // position open (9 cm)
  close.command.max_effort = close_max_effort;  // unlimited motor effort
  pr2_gripper_sensor_msgs::PR2GripperFindContactGoal findTwo;
  findTwo.command.contact_conditions = findtwo_contact_conditions;
  findTwo.command.zero_fingertip_sensors = findtwo_zero_fingertip_sensors;  
  pr2_gripper_sensor_msgs::PR2GripperEventDetectorGoal place;
  place.command.trigger_conditions = event_trigger_conditions;  
  place.command.acceleration_trigger_magnitude = event_acceleration_trigger_magnitude;  // set the contact acceleration to n m/s^2
  place.command.slip_trigger_magnitude = event_slip_trigger_magnitude;

  switch(side){
  case pr2motion_LEFT :
    switch(goal_mode){
    case pr2motion_GRIPPER_GRAB :
      left_gripper.findTwoContacts(findTwo);
      return pr2motion_waitcontact;
    case pr2motion_GRIPPER_RELEASE :
      left_gripper.place(place);
      return pr2motion_waitrelease;
    case pr2motion_GRIPPER_OPEN :
      left_gripper.open(open);
      return pr2motion_waitopen;
    case pr2motion_GRIPPER_CLOSE :
      left_gripper.close(close);
      return pr2motion_waitclose;
    }
  case pr2motion_RIGHT:
    switch(goal_mode){
    case pr2motion_GRIPPER_GRAB :
      right_gripper.findTwoContacts(findTwo);
      return pr2motion_waitcontact;
    case pr2motion_GRIPPER_RELEASE :
      return pr2motion_waitrelease;
    case pr2motion_GRIPPER_OPEN :
      right_gripper.open(open);
      return pr2motion_waitopen;
    case pr2motion_GRIPPER_CLOSE :
      right_gripper.close(close);
      return pr2motion_waitclose;
    }
  }

  /* skeleton sample */ return pr2motion_wait;
  // we are in simulation
#else
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = open_position;    // position open (9 cm)
  open.command.max_effort = open_max_effort;  // unlimited motor effort
  pr2_controllers_msgs::Pr2GripperCommandGoal close;
  close.command.position = close_position;    // position open (9 cm)
  close.command.max_effort = close_max_effort;  // unlimited motor effort

  switch(side){
  case pr2motion_LEFT :
    switch(goal_mode){
    case pr2motion_GRIPPER_OPEN :
      left_gripper.open(open);
      return pr2motion_waitopen;
    case pr2motion_GRIPPER_CLOSE :
      left_gripper.close(close);
      return pr2motion_waitclose;
    }
  case pr2motion_RIGHT:
    switch(goal_mode){
    case pr2motion_GRIPPER_OPEN :
      right_gripper.open(open);
      return pr2motion_waitopen;
    case pr2motion_GRIPPER_CLOSE :
      right_gripper.close(close);
      return pr2motion_waitclose;
    }
  }
#endif
}

/** Codel waitOperateGripper of activity Gripper_Operate.
 *
 * Triggered by pr2motion_wait.
 * Yields to pr2motion_wait, pr2motion_exec, pr2motion_stop,
 *           pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
waitOperateGripper(pr2motion_SIDE side,
                   pr2motion_GRIPPER_MODE goal_mode,
                   genom_context self)
{

  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_end;
}

/** Codel waitcontactOperateGripper of activity Gripper_Operate.
 *
 * Triggered by pr2motion_waitcontact.
 * Yields to pr2motion_slipservo, pr2motion_wait, pr2motion_stop,
 *           pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
waitcontactOperateGripper(pr2motion_SIDE side,
                          pr2motion_GRIPPER_MODE goal_mode,
                          genom_context self)
{
#ifndef PR2_SIMU
  if (side >= pr2motion_NB_SIDE)
    return pr2motion_end;    

 switch(side){
  case pr2motion_LEFT :
    if(left_gripper.findTwo_isDone())
      return pr2motion_slipservo;
    else
      return pr2motion_waitcontact;
  case pr2motion_RIGHT :
    if(right_gripper.findTwo_isDone())
      return pr2motion_slipservo;
    else
      return pr2motion_waitcontact;
  default:
    return pr2motion_unknown_error(self);
 }
 
#else
  return pr2motion_unknown_error(self); 
#endif
}

/** Codel waitopenOperateGripper of activity Gripper_Operate.
 *
 * Triggered by pr2motion_waitopen.
 * Yields to pr2motion_waitopen, pr2motion_wait, pr2motion_stop,
 *           pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
waitopenOperateGripper(pr2motion_SIDE side,
                       pr2motion_GRIPPER_MODE goal_mode,
                       genom_context self)
{
 if (side >= pr2motion_NB_SIDE)
    return pr2motion_invalid_param(self);    

#ifndef PR2_SIMU  
  switch(side){
  case pr2motion_LEFT :
    if(left_gripper.open_isDone())
      return pr2motion_waitcontact;
    else
      return pr2motion_waitopen;
  case pr2motion_RIGHT :
    if(right_gripper.open_isDone())
      return pr2motion_waitcontact;
    else
      return pr2motion_waitopen;
  default:
    return pr2motion_unknown_error(self);
  }
#else
  switch(side){
  case pr2motion_LEFT :
    if(left_gripper.open_isDone())
      return pr2motion_end;
    else
      return pr2motion_waitopen;
  case pr2motion_RIGHT :
    if(right_gripper.open_isDone())
      return pr2motion_end;
    else
      return pr2motion_waitopen;
  default:
    return pr2motion_unknown_error(self);
  }
#endif

}

/** Codel waitcloseOperateGripper of activity Gripper_Operate.
 *
 * Triggered by pr2motion_waitclose.
 * Yields to pr2motion_waitclose, pr2motion_wait, pr2motion_stop,
 *           pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
waitcloseOperateGripper(pr2motion_SIDE side,
                        pr2motion_GRIPPER_MODE goal_mode,
                        genom_context self)
{

  if (side >= pr2motion_NB_SIDE)
    return pr2motion_invalid_param(self);    
  
  switch(side){
  case pr2motion_LEFT :
    if(left_gripper.close_isDone()){
      printf("close done\n");
      return pr2motion_end;
    } else {
      return pr2motion_waitclose;
    }
  case pr2motion_RIGHT :
    if(right_gripper.close_isDone()){
      printf("close done\n");
      return pr2motion_end;
    } else {
      return pr2motion_waitclose;
    }
  default:
    return pr2motion_unknown_error(self);
  }

}

/** Codel waitreleaseOperateGripper of activity Gripper_Operate.
 *
 * Triggered by pr2motion_waitrelease.
 * Yields to pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
waitreleaseOperateGripper(pr2motion_SIDE side,
                          pr2motion_GRIPPER_MODE goal_mode,
                          genom_context self)
{
#ifndef PR2_SIMU
  if (side >= pr2motion_NB_SIDE)
    return pr2motion_invalid_param(self);    

  switch(side){
  case pr2motion_LEFT :
    if(left_gripper.place_isDone())
      return pr2motion_end;
    else
      return pr2motion_waitcontact;
  case pr2motion_RIGHT :
    if(right_gripper.place_isDone())
      return pr2motion_end;
    else
      return pr2motion_waitcontact;
  default:
    return pr2motion_unknown_error(self);
  }
#else
  return pr2motion_unknown_error(self);
#endif
}

/** Codel slipservoOperateGripper of activity Gripper_Operate.
 *
 * Triggered by pr2motion_slipservo.
 * Yields to pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
slipservoOperateGripper(pr2motion_SIDE side,
                        pr2motion_GRIPPER_MODE goal_mode,
                        genom_context self)
{
  if (side >= pr2motion_NB_SIDE)
    return pr2motion_invalid_param(self);    

#ifndef PR2_SIMU
  switch(side){
  case pr2motion_LEFT :
    left_gripper.slipServo();
    return pr2motion_end;
  case pr2motion_RIGHT :
    right_gripper.slipServo();
    return pr2motion_end;
  default:
    return pr2motion_unknown_error(self);
  }
#else
  return pr2motion_unknown_error(self);
#endif
}

/** Codel stopOperateGripper of activity Gripper_Operate.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
stopOperateGripper(pr2motion_SIDE side,
                   pr2motion_GRIPPER_MODE goal_mode,
                   genom_context self)
{
  if (side >= pr2motion_NB_SIDE)
    return pr2motion_end;    
  
  if (goal_mode >= pr2motion_GRIPPER_NB_MODE)
    return pr2motion_end;

  switch(side){
  case pr2motion_LEFT :
    switch(goal_mode){
#ifndef PR2_SIMU
    case pr2motion_GRIPPER_GRAB :
      left_gripper.findTwo_cancel();
    case pr2motion_GRIPPER_RELEASE :
      left_gripper.place_cancel();
      return pr2motion_waitrelease;
#endif
    case pr2motion_GRIPPER_OPEN :
      left_gripper.open_cancel();
    case pr2motion_GRIPPER_CLOSE :
      left_gripper.close_cancel();
    }
  case pr2motion_RIGHT :
    switch(goal_mode){
#ifndef PR2_SIMU
    case pr2motion_GRIPPER_GRAB :
      right_gripper.findTwo_cancel();
    case pr2motion_GRIPPER_RELEASE :
      right_gripper.place_cancel();
      return pr2motion_waitrelease;
#endif
    case pr2motion_GRIPPER_OPEN :
      right_gripper.open_cancel();
    case pr2motion_GRIPPER_CLOSE :
      right_gripper.close_cancel();
    }   
  }
  return pr2motion_end;
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}

/** Codel endOperateGripper of activity Gripper_Operate.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
endOperateGripper(pr2motion_SIDE side,
                  pr2motion_GRIPPER_MODE goal_mode,
                  genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}


/* --- Activity Gripper_Right_Operate ----------------------------------- */

/** Codel startOperateGripper of activity Gripper_Right_Operate.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_exec, pr2motion_stop, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel execOperateGripper of activity Gripper_Right_Operate.
 *
 * Triggered by pr2motion_exec.
 * Yields to pr2motion_exec, pr2motion_wait, pr2motion_waitcontact, pr2motion_waitopen, pr2motion_waitclose, pr2motion_waitrelease, pr2motion_stop, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel waitOperateGripper of activity Gripper_Right_Operate.
 *
 * Triggered by pr2motion_wait.
 * Yields to pr2motion_wait, pr2motion_exec, pr2motion_stop, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel waitcontactOperateGripper of activity Gripper_Right_Operate.
 *
 * Triggered by pr2motion_waitcontact.
 * Yields to pr2motion_slipservo, pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel waitopenOperateGripper of activity Gripper_Right_Operate.
 *
 * Triggered by pr2motion_waitopen.
 * Yields to pr2motion_waitopen, pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel waitcloseOperateGripper of activity Gripper_Right_Operate.
 *
 * Triggered by pr2motion_waitclose.
 * Yields to pr2motion_waitclose, pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel waitreleaseOperateGripper of activity Gripper_Right_Operate.
 *
 * Triggered by pr2motion_waitrelease.
 * Yields to pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel slipservoOperateGripper of activity Gripper_Right_Operate.
 *
 * Triggered by pr2motion_slipservo.
 * Yields to pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel stopOperateGripper of activity Gripper_Right_Operate.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel endOperateGripper of activity Gripper_Right_Operate.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */



/* --- Activity Gripper_Left_Operate ------------------------------------ */

/** Codel startOperateGripper of activity Gripper_Left_Operate.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_exec, pr2motion_stop, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel execOperateGripper of activity Gripper_Left_Operate.
 *
 * Triggered by pr2motion_exec.
 * Yields to pr2motion_exec, pr2motion_wait, pr2motion_waitcontact, pr2motion_waitopen, pr2motion_waitclose, pr2motion_waitrelease, pr2motion_stop, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel waitOperateGripper of activity Gripper_Left_Operate.
 *
 * Triggered by pr2motion_wait.
 * Yields to pr2motion_wait, pr2motion_exec, pr2motion_stop, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel waitcontactOperateGripper of activity Gripper_Left_Operate.
 *
 * Triggered by pr2motion_waitcontact.
 * Yields to pr2motion_slipservo, pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel waitopenOperateGripper of activity Gripper_Left_Operate.
 *
 * Triggered by pr2motion_waitopen.
 * Yields to pr2motion_waitopen, pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel waitcloseOperateGripper of activity Gripper_Left_Operate.
 *
 * Triggered by pr2motion_waitclose.
 * Yields to pr2motion_waitclose, pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel waitreleaseOperateGripper of activity Gripper_Left_Operate.
 *
 * Triggered by pr2motion_waitrelease.
 * Yields to pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel slipservoOperateGripper of activity Gripper_Left_Operate.
 *
 * Triggered by pr2motion_slipservo.
 * Yields to pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel stopOperateGripper of activity Gripper_Left_Operate.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */


/** Codel endOperateGripper of activity Gripper_Left_Operate.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Gripper_Operate */



/* --- Activity Torso_Move ---------------------------------------------- */

/** Codel startMoveTorso of activity Torso_Move.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_wait.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
startMoveTorso(float torso_position, genom_context self)
{
  Torso::ERROR result_move;
  Torso::ERROR result_connect = torso.isConnected();
  pr2_controllers_msgs::SingleJointPositionGoal torso_cmd;

  switch(result_connect){
  case Torso::OK:
    torso_cmd.position=torso_position;
    //    torso_cmd.min_duration=ros::Duration(torso_min_duration);
    //torso_cmd.max_velocity=torso_max_velocity;
    result_move = torso.move(torso_cmd);
    printf("result_move=%d\n",result_move);
    if(result_move==Torso::OK)
      return pr2motion_wait;
    else
      torso.cancelCmd();
      return pr2motion_invalid_param(self);
  case Torso::INIT_NOT_DONE:
    return pr2motion_init_not_done(self);
  case Torso::SERVER_NOT_CONNECTED:
    return pr2motion_not_connected(self);
  default:
    return pr2motion_unknown_error(self);
  }

}

/** Codel waitMoveTorso of activity Torso_Move.
 *
 * Triggered by pr2motion_wait.
 * Yields to pr2motion_wait, pr2motion_end.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */

genom_event
waitMoveTorso(genom_context self)
{
  // Check if move is in a terminal state, 
  // if yes, goto end
  // otherwise, goto wait

  if(torso.move_isDone())
    return pr2motion_end;
  else
    return pr2motion_wait;
}

/** Codel endMoveTorso of activity Torso_Move.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
endMoveTorso(genom_context self)
{
  return pr2motion_ether;
}

/** Codel stopMoveTorso of activity Torso_Move.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
stopMoveTorso(genom_context self)
{
  torso.cancelCmd();
  return pr2motion_ether;
}


/* --- Activity Head_Move ----------------------------------------------- */

/** Codel startMoveHead of activity Head_Move.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_wait.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
startMoveHead(pr2motion_HEAD_MODE head_mode,
              const char head_target_frame[128], double head_target_x,
              double head_target_y, double head_target_z,
              genom_context self)
{
  std::string frame_id(head_target_frame);
  if(head_mode>=pr2motion_HEAD_NB_MODE)
    return pr2motion_invalid_param(self);
 
  RobotHead::ERROR result_connect = head.isConnected();
  switch(result_connect){
  case RobotHead::OK:
    head.lookAt(frame_id, head_target_x, head_target_y, head_target_z);
    return pr2motion_wait;  
  case RobotHead::INIT_NOT_DONE:
    return pr2motion_init_not_done(self);
  case RobotHead::SERVER_NOT_CONNECTED:
    return pr2motion_not_connected(self);
  default:
    return pr2motion_unknown_error(self);
  }
}

/** Codel waitMoveHead of activity Head_Move.
 *
 * Triggered by pr2motion_wait.
 * Yields to pr2motion_start, pr2motion_wait, pr2motion_end,
 *           pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
waitMoveHead(pr2motion_HEAD_MODE head_mode,
             const pr2motion_head_controller_state *head_controller_state,
             pr2motion_pr2_controllers_msgs_jointtrajectorycontrollerstate *head_controller_state_msg,
             genom_context self)
{
  int name_size=0;
  double pan_desired_position;
  double tilt_desired_position;
  int head_pan_joint_indice = -1;
  int head_tilt_joint_indice = -1;
  RobotHead::ERROR result_check=RobotHead::OK;

  head_controller_state->read(self);
  if(head_controller_state->data(self)!=NULL){
    name_size = head_controller_state->data(self)->joint_names._length;
    if(name_size!=2){
      printf("Head_Move the number of joint in the head_controller_state is not correct\n");
      head.cancelCmd();
      return pr2motion_invalid_param(self);
    } else {
      genom_sequence_reserve(&(head_controller_state_msg->desired.positions), name_size);
      for (size_t ind=0; ind<name_size; ind++) {
	head_controller_state_msg->desired.positions._buffer[ind]=head_controller_state->data(self)->desired.positions._buffer[ind];
	if(strcmp(head_controller_state->data(self)->joint_names._buffer[ind],"head_pan_joint")==0){
	  head_pan_joint_indice = ind;
	}
	if(strcmp(head_controller_state->data(self)->joint_names._buffer[ind],"head_tilt_joint")==0){
	  head_tilt_joint_indice = ind;
	}
      }
      if ((head_pan_joint_indice==-1) || (head_tilt_joint_indice==-1)){
	head.cancelCmd();
	return pr2motion_invalid_param(self);
      }
      result_check = head.checkCmdLimits(head_controller_state_msg->desired.positions._buffer[head_pan_joint_indice],head_controller_state_msg->desired.positions._buffer[head_tilt_joint_indice]);
      if(result_check!=RobotHead::OK){
	head.cancelCmd();
	return pr2motion_invalid_param(self);
      }
    } 
  } else {
    printf("Head_Move cannot read head_controller_state, cannot check limits\n");
    head.cancelCmd();
    return pr2motion_invalid_param(self);    
  }

  // Check if move is in a terminal state, 
  // if yes, goto end
  // otherwise, goto wait
  if(head.lookAt_isDone())
    if(head_mode==pr2motion_HEAD_LOOKAT) {
      return pr2motion_end;
    } else {
      return pr2motion_start;
    }
  else
    return pr2motion_wait;   

}

/** Codel endMoveHead of activity Head_Move.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
endMoveHead(genom_context self)
{
  return pr2motion_ether;
}

/** Codel stopMoveHead of activity Head_Move.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
stopMoveHead(genom_context self)
{
  head.cancelCmd();
  return pr2motion_ether;
}


/* --- Activity Arm_Move ------------------------------------------------ */

/** Codel getPathArm of activity Arm_Move.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_computetraj,
 *           pr2motion_checktraj.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
getPathArm(pr2motion_SIDE side, pr2motion_PATH_MODE path_mode,
           const pr2motion_traj *traj, genom_context self)
{
  printf("getpath arm \n");
  // goal (path) to be send to the controller
  pr2_controllers_msgs::JointTrajectoryGoal path_cmd;
  //pr2_controllers_msgs::JointTrajectoryGoal traj_cmd;

  // path nb_points
  int points_vector_size;
  // joint vector size
  int joint_names_vector_size = 7;

  // right arm joint indice
  int r_shoulder_pan_joint_indice = 0;
  int r_shoulder_lift_joint_indice = 0;
  int r_upper_arm_roll_joint_indice = 0;
  int r_elbow_flex_joint_indice = 0;
  int r_forearm_roll_joint_indice = 0;
  int r_wrist_flex_joint_indice = 0;
  int r_wrist_roll_joint_indice = 0;

  // left arm joint indice
  int l_shoulder_pan_joint_indice = 0;  
  int l_shoulder_lift_joint_indice = 0;
  int l_upper_arm_roll_joint_indice = 0;
  int l_elbow_flex_joint_indice = 0;
  int l_forearm_roll_joint_indice = 0;
  int l_wrist_flex_joint_indice = 0;
  int l_wrist_roll_joint_indice = 0;

  RobotArm::ERROR result;

  if (side >= pr2motion_NB_SIDE)
    return pr2motion_invalid_param(self);    

  if (path_mode >= pr2motion_PATH_NB_MODE)
    return pr2motion_invalid_param(self);

  // test if the server is connected
  RobotArm::ERROR result_connect;
  if (side == pr2motion_RIGHT) {
    result_connect = right_arm.isConnected();
  } else {
    result_connect = left_arm.isConnected();
  }

  switch(result_connect){
  case RobotArm::OK:
    break;
  case RobotArm::INIT_NOT_DONE:
    return pr2motion_init_not_done(self);
  case RobotArm::SERVER_NOT_CONNECTED:
    return pr2motion_not_connected(self);
  default:
    return pr2motion_unknown_error(self);
  }

  // read trajectory from the test function
  if(path_mode==pr2motion_PATH_TEST) {
    if (side == pr2motion_RIGHT) {
      right_arm.clearTrajectory();
      right_arm.gettestPath();
      return pr2motion_computetraj;
    } else {
      left_arm.clearTrajectory();
      left_arm.gettestPath();
      return pr2motion_computetraj;
    }
  }

  // read the trajectory from the port
  traj->read(self);
  if(traj->data(self)!=NULL){
    // nb_points in the path
    points_vector_size = traj->data(self)->traj.points._length;
    if(points_vector_size==0){
      printf("the trajectory is empty !");
      return pr2motion_end;
    }
    path_cmd.trajectory.points.resize(points_vector_size);

    // get the correct index for all needed joints in the path
    for (size_t ind=0; ind<traj->data(self)->traj.joint_names._length; ind++) {
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind],"r_shoulder_pan_joint")==0){
	r_shoulder_pan_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind],"r_shoulder_lift_joint")==0){
	r_shoulder_lift_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind],"r_upper_arm_roll_joint")==0){
	r_upper_arm_roll_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind],"r_elbow_flex_joint")==0){
	r_elbow_flex_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind],"r_forearm_roll_joint")==0){
	r_forearm_roll_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind],"r_wrist_flex_joint")==0){
	r_wrist_flex_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind],"r_wrist_roll_joint")==0){
	r_wrist_roll_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind], "l_shoulder_pan_joint")==0){
	l_shoulder_pan_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind], "l_shoulder_lift_joint")==0){
	l_shoulder_lift_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind], "l_upper_arm_roll_joint")==0){
	l_upper_arm_roll_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind],"l_elbow_flex_joint")==0){
	l_elbow_flex_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind],"l_forearm_roll_joint")==0){
	l_forearm_roll_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind],"l_wrist_flex_joint")==0){
	l_wrist_flex_joint_indice = ind;
      }
      if(strcmp(traj->data(self)->traj.joint_names._buffer[ind],"l_wrist_roll_joint")==0){
	l_wrist_roll_joint_indice = ind;
      }
    }
    
    if(side == pr2motion_RIGHT){
      path_cmd.trajectory.joint_names.resize(joint_names_vector_size);
      path_cmd.trajectory.joint_names[0]="r_shoulder_pan_joint";
      path_cmd.trajectory.joint_names[1]="r_shoulder_lift_joint";
      path_cmd.trajectory.joint_names[2]="r_upper_arm_roll_joint";
      path_cmd.trajectory.joint_names[3]="r_elbow_flex_joint";
      path_cmd.trajectory.joint_names[4]="r_forearm_roll_joint";
      path_cmd.trajectory.joint_names[5]="r_wrist_flex_joint";
      path_cmd.trajectory.joint_names[6]="r_wrist_roll_joint";
      for ( size_t ind=0; ind < points_vector_size ; ind++) {
	path_cmd.trajectory.points[ind].positions.resize(joint_names_vector_size);
	path_cmd.trajectory.points[ind].positions[0] = traj->data(self)->traj.points._buffer[ind].positions._buffer[r_shoulder_pan_joint_indice];
	path_cmd.trajectory.points[ind].positions[1] = traj->data(self)->traj.points._buffer[ind].positions._buffer[r_shoulder_lift_joint_indice];
	path_cmd.trajectory.points[ind].positions[2] = traj->data(self)->traj.points._buffer[ind].positions._buffer[r_upper_arm_roll_joint_indice];
	path_cmd.trajectory.points[ind].positions[3] = traj->data(self)->traj.points._buffer[ind].positions._buffer[r_elbow_flex_joint_indice];
	path_cmd.trajectory.points[ind].positions[4] = traj->data(self)->traj.points._buffer[ind].positions._buffer[r_forearm_roll_joint_indice];
	path_cmd.trajectory.points[ind].positions[5] = traj->data(self)->traj.points._buffer[ind].positions._buffer[r_wrist_flex_joint_indice];
	path_cmd.trajectory.points[ind].positions[6] = traj->data(self)->traj.points._buffer[ind].positions._buffer[r_wrist_roll_joint_indice];
      }
      right_arm.clearTrajectory();
      if(right_arm.setTraj(&path_cmd) == RobotArm::OK){
	return pr2motion_computetraj;
      } else {
	return pr2motion_end;	 
      }
    } else {
      path_cmd.trajectory.joint_names.resize(joint_names_vector_size);
      path_cmd.trajectory.joint_names[0]="l_shoulder_pan_joint";
      path_cmd.trajectory.joint_names[1]="l_shoulder_lift_joint";
      path_cmd.trajectory.joint_names[2]="l_upper_arm_roll_joint";
      path_cmd.trajectory.joint_names[3]="l_elbow_flex_joint";
      path_cmd.trajectory.joint_names[4]="l_forearm_roll_joint";
      path_cmd.trajectory.joint_names[5]="l_wrist_flex_joint";
      path_cmd.trajectory.joint_names[6]="l_wrist_roll_joint";
      for ( size_t ind=0; ind < points_vector_size ; ind++) {
	path_cmd.trajectory.points[ind].positions.resize(joint_names_vector_size);
	path_cmd.trajectory.points[ind].positions[0] = traj->data(self)->traj.points._buffer[ind].positions._buffer[l_shoulder_pan_joint_indice];
	path_cmd.trajectory.points[ind].positions[1] = traj->data(self)->traj.points._buffer[ind].positions._buffer[l_shoulder_lift_joint_indice];
	path_cmd.trajectory.points[ind].positions[2] = traj->data(self)->traj.points._buffer[ind].positions._buffer[l_upper_arm_roll_joint_indice];
	path_cmd.trajectory.points[ind].positions[3] = traj->data(self)->traj.points._buffer[ind].positions._buffer[l_elbow_flex_joint_indice];
	path_cmd.trajectory.points[ind].positions[4] = traj->data(self)->traj.points._buffer[ind].positions._buffer[l_forearm_roll_joint_indice];
	path_cmd.trajectory.points[ind].positions[5] = traj->data(self)->traj.points._buffer[ind].positions._buffer[l_wrist_flex_joint_indice];
	path_cmd.trajectory.points[ind].positions[6] = traj->data(self)->traj.points._buffer[ind].positions._buffer[l_wrist_roll_joint_indice];
      }
      left_arm.clearTrajectory();
      if(left_arm.setTraj(&path_cmd) == RobotArm::OK)
	return pr2motion_computetraj;
      else
	return pr2motion_end;     
    }
  } else {
    // not able to read the trajectory on the port
    printf("no trajectory found on the port.. \n");
    return pr2motion_end;
  }
  printf("aurevoir.. \n");
  return pr2motion_end;
}

/** Codel computeTrajArm of activity Arm_Move.
 *
 * Triggered by pr2motion_computetraj.
 * Yields to pr2motion_checktraj, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
computeTrajArm(pr2motion_SIDE side, pr2motion_TRAJ_MODE traj_mode,
               double time_slot, genom_context self)
{
  printf("computetrajarm\n");
  if(side == pr2motion_RIGHT){
    //    right_arm.setMax(max_vel, max_acc, max_jerk);
    //    right_arm.setT(time_slot);
    switch (traj_mode){
    case pr2motion_TRAJ_SOFTMOTION:
      right_arm.setTrajMode(RobotArm::SOFT_MOTION);
      break;
    case pr2motion_TRAJ_GATECH:
      right_arm.setTrajMode(RobotArm::GATECH);
      break;
    case pr2motion_TRAJ_PATH:
      right_arm.setTrajMode(RobotArm::PATH);
    default:
      return pr2motion_end;
    }
    right_arm.computeTrajectory();
    return pr2motion_checktraj;
  } else {
    //    left_arm.setMax(max_vel, max_acc, max_jerk);
    //    left_arm.setT(time_slot);
    switch (traj_mode){
    case pr2motion_TRAJ_SOFTMOTION:
      left_arm.setTrajMode(RobotArm::SOFT_MOTION);
      break;
    case pr2motion_TRAJ_GATECH:
      left_arm.setTrajMode(RobotArm::GATECH);
      break;
    case pr2motion_TRAJ_PATH:
      left_arm.setTrajMode(RobotArm::PATH);
    default:
      return pr2motion_end;
    }
    left_arm.computeTrajectory();
    return pr2motion_checktraj;    
  }
  return pr2motion_end;
}

/** Codel checkTrajArm of activity Arm_Move.
 *
 * Triggered by pr2motion_checktraj.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_launchmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
checkTrajArm(pr2motion_SIDE side, genom_context self)
{
  printf("checktrajarm\n");
  RobotArm::ERROR result;
  if(side == pr2motion_RIGHT){
    result=right_arm.validateTrajectory();
  } else {
    result=left_arm.validateTrajectory();
  }
  if(result == RobotArm::OK)
    return pr2motion_launchmove;
  else
    return pr2motion_end;
}

/** Codel launchMoveArm of activity Arm_Move.
 *
 * Triggered by pr2motion_launchmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
launchMoveArm(pr2motion_SIDE side, genom_context self)
{
  printf("launchmovearm\n");
  if(side == pr2motion_RIGHT){
    right_arm.move();
  } else {
    left_arm.move();
  }
  return pr2motion_waitmove;
}

/** Codel waitMoveArm of activity Arm_Move.
 *
 * Triggered by pr2motion_waitmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
waitMoveArm(pr2motion_SIDE side, genom_context self)
{
  printf("waitmovearm\n");
  if(side == pr2motion_RIGHT){
    if(right_arm.move_isDone())
      return pr2motion_end;
    else
      return pr2motion_waitmove;
  } else {
    if(left_arm.move_isDone())
      return pr2motion_end;
    else
      return pr2motion_waitmove;    
  }
}

/** Codel endMoveArm of activity Arm_Move.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
endMoveArm(pr2motion_SIDE side, genom_context self)
{
  printf("endmovearm\n");
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}

/** Codel stopMoveArm of activity Arm_Move.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
stopMoveArm(pr2motion_SIDE side, genom_context self)
{
  if(side == pr2motion_RIGHT){
    right_arm.cancelCmd();
  } else {
    left_arm.cancelCmd();
  }
  return pr2motion_ether;
}


/* --- Activity Arm_MoveToQGoal ----------------------------------------- */

/** Codel getQGoal of activity Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_computetraj, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error,
 *        pr2motion_joint_state_unavailable.
 */
genom_event
getQGoal(pr2motion_SIDE side, bool joint_state_availability,
         const pr2motion_sensor_msgs_jointstate *joint_state_msg,
         double shoulder_pan_joint, double shoulder_lift_joint,
         double upper_arm_roll_joint, double elbow_flex_joint,
         double forearm_roll_joint, double wrist_flex_joint,
         double wrist_roll_joint, genom_context self)
{
  // goal (path) to be send to the controller
  pr2_controllers_msgs::JointTrajectoryGoal path_cmd;
  int joint_names_vector_size = 7;
  int points_vector_size = 2;
  path_cmd.trajectory.joint_names.resize(joint_names_vector_size);
  path_cmd.trajectory.points.resize(points_vector_size);
  path_cmd.trajectory.points[0].positions.resize(joint_names_vector_size);
  path_cmd.trajectory.points[1].positions.resize(joint_names_vector_size);

  // test if the server is connected
  RobotArm::ERROR result_connect;
  if (side == pr2motion_RIGHT) {
    result_connect = right_arm.isConnected();
  } else {
    result_connect = left_arm.isConnected();
  }  

  switch(result_connect){
  case RobotArm::OK:
    break;
  case RobotArm::INIT_NOT_DONE:
    return pr2motion_init_not_done(self);
  case RobotArm::SERVER_NOT_CONNECTED:
    return pr2motion_not_connected(self);
  default:
    return pr2motion_unknown_error(self);
  }
  
  printf("GetQGoal: Arm is connected with vector length %d \n",joint_state_msg->name._length);
  
  // check if we can get the actual arm position
  if(!joint_state_availability){
    printf("GetQGoal cannot get robot actual position\n");
    return pr2motion_joint_state_unavailable(self);
  }
    
  printf("avant\n");
  if(side == pr2motion_RIGHT){
    printf("1\n");
  // get the correct index for all needed joints in the path
  for (size_t ind=0; ind<joint_state_msg->name._length; ind++) {
    printf("ind= %d 2\n", ind);
    if(strcmp(joint_state_msg->name._buffer[ind],"r_shoulder_pan_joint")==0){
       printf("r_shoulder_pan_joint\n");
      path_cmd.trajectory.joint_names[0]="r_shoulder_pan_joint";
      path_cmd.trajectory.points[0].positions[0] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[0] = shoulder_pan_joint;
    }
    if(strcmp(joint_state_msg->name._buffer[ind],"r_shoulder_lift_joint")==0){
      printf("r_shoulder_lift_joint\n");
      path_cmd.trajectory.joint_names[1]="r_shoulder_lift_joint";
      path_cmd.trajectory.points[0].positions[1] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[1] = shoulder_lift_joint;
    }
    if(strcmp(joint_state_msg->name._buffer[ind],"r_upper_arm_roll_joint")==0){
      printf("r_upper_arm_roll_joint\n");      
      path_cmd.trajectory.joint_names[2]="r_upper_arm_roll_joint";
      path_cmd.trajectory.points[0].positions[2] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[2] = upper_arm_roll_joint;
    }
    if(strcmp(joint_state_msg->name._buffer[ind],"r_elbow_flex_joint")==0){
      printf("r_elbow_flex_joint\n");           
      path_cmd.trajectory.joint_names[3]="r_elbow_flex_joint";
      path_cmd.trajectory.points[0].positions[3] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[3] = elbow_flex_joint;
    }
    if(strcmp(joint_state_msg->name._buffer[ind],"r_forearm_roll_joint")==0){
      printf("r_forearm_roll_joint\n");
      path_cmd.trajectory.joint_names[4]="r_forearm_roll_joint";
      path_cmd.trajectory.points[0].positions[4] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[4] = forearm_roll_joint;
    }
    if(strcmp(joint_state_msg->name._buffer[ind],"r_wrist_flex_joint")==0){
      printf("r_wrist_flex_joint\n");      
      path_cmd.trajectory.joint_names[5]="r_wrist_flex_joint";
      path_cmd.trajectory.points[0].positions[5] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[5] = wrist_flex_joint;
    }
    if(strcmp(joint_state_msg->name._buffer[ind],"r_wrist_roll_joint")==0){
      printf("r_wrist_roll_joint\n"); 
      path_cmd.trajectory.joint_names[6]="r_wrist_roll_joint";
      path_cmd.trajectory.points[0].positions[6] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[6] = wrist_roll_joint;
    }
     printf("4\n");
  }
   printf("5\n");
  right_arm.clearTrajectory();
   printf("6\n");
  if(right_arm.setTraj(&path_cmd) == RobotArm::OK){
    printf("GetQGoal setTraj ok\n");
    return pr2motion_computetraj;
  } else {
    printf("GetQGoal setTraj ko\n");
    return pr2motion_end;	 
  }
  } else {
    printf("1, length of the joint_state %d\n",joint_state_msg->name._length);
  for (size_t ind=0; ind<joint_state_msg->name._length; ind++) {
    if(strcmp(joint_state_msg->name._buffer[ind],"l_shoulder_pan_joint")==0){
      printf("fill l_shoulder_pan_joint with 0 %f and 1 %f \n", joint_state_msg->position._buffer[ind],shoulder_pan_joint);
      path_cmd.trajectory.joint_names[0]="l_shoulder_pan_joint";
      path_cmd.trajectory.points[0].positions[0] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[0] = shoulder_pan_joint;
    }
    if(strcmp(joint_state_msg->name._buffer[ind],"l_shoulder_lift_joint")==0){
      printf("fill l_shoulder_lift_joint\n");
      path_cmd.trajectory.joint_names[1]="l_shoulder_lift_joint";
      path_cmd.trajectory.points[0].positions[1] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[1] = shoulder_lift_joint;
    }
    if(strcmp(joint_state_msg->name._buffer[ind],"l_upper_arm_roll_joint")==0){
      printf("fill l_upper_arm_roll_joint\n");
      path_cmd.trajectory.joint_names[2]="l_upper_arm_roll_joint";
      path_cmd.trajectory.points[0].positions[2] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[2] = upper_arm_roll_joint;
    }
    if(strcmp(joint_state_msg->name._buffer[ind],"l_elbow_flex_joint")==0){
      printf("fill l_elbow_flex_joint\n");
      path_cmd.trajectory.joint_names[3]="l_elbow_flex_joint";
      path_cmd.trajectory.points[0].positions[3] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[3] = elbow_flex_joint;
    }
    if(strcmp(joint_state_msg->name._buffer[ind],"l_forearm_roll_joint")==0){
      printf("fill l_forearm_roll_joint\n");
      path_cmd.trajectory.joint_names[4]="l_forearm_roll_joint";
      path_cmd.trajectory.points[0].positions[4] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[4] = forearm_roll_joint;
    }
    if(strcmp(joint_state_msg->name._buffer[ind],"l_wrist_flex_joint")==0){
      printf("fill l_wrist_flex_joint with 0 %f and 1 %f\n", joint_state_msg->position._buffer[ind], wrist_flex_joint);
      path_cmd.trajectory.joint_names[5]="l_wrist_flex_joint";
      path_cmd.trajectory.points[0].positions[5] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[5] = wrist_flex_joint;
    }
    if(strcmp(joint_state_msg->name._buffer[ind],"l_wrist_roll_joint")==0){
      printf("fill l_wrist_roll_joint\n");
      path_cmd.trajectory.joint_names[6]="l_wrist_flex_joint";
      path_cmd.trajectory.points[0].positions[6] = joint_state_msg->position._buffer[ind];
      path_cmd.trajectory.points[1].positions[6] = wrist_roll_joint;
    } 
  }
  left_arm.clearTrajectory();
  if(left_arm.setTraj(&path_cmd) == RobotArm::OK){
    printf("GetQGoal setTraj ok\n");      
    return pr2motion_computetraj;
  } else {
    printf("GetQGoal setTraj ko\n");
    return pr2motion_end;     
  }
  }
  return pr2motion_end;
}

/** Codel computeTrajQGoal of activity Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_computetraj.
 * Yields to pr2motion_checktraj, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error,
 *        pr2motion_joint_state_unavailable.
 */
genom_event
computeTrajQGoal(pr2motion_SIDE side, pr2motion_TRAJ_MODE traj_mode,
                 genom_context self)
{
  printf("ComputeTrajQGoal\n");
  if(side == pr2motion_RIGHT){
    //    right_arm.setMax(max_vel, max_acc, max_jerk);
    //    right_arm.setT(time_slot);
    switch (traj_mode){
    case pr2motion_TRAJ_SOFTMOTION:
      right_arm.setTrajMode(RobotArm::SOFT_MOTION);
      break;
    case pr2motion_TRAJ_GATECH:
      right_arm.setTrajMode(RobotArm::GATECH);
      break;
    case pr2motion_TRAJ_PATH:
      right_arm.setTrajMode(RobotArm::PATH);
    default:
      return pr2motion_end;
    }
    right_arm.computeTrajectory();
    return pr2motion_checktraj;
  } else {
    //    left_arm.setMax(max_vel, max_acc, max_jerk);
    //    left_arm.setT(time_slot);
    switch (traj_mode){
    case pr2motion_TRAJ_SOFTMOTION:
      left_arm.setTrajMode(RobotArm::SOFT_MOTION);
      break;
    case pr2motion_TRAJ_GATECH:
      left_arm.setTrajMode(RobotArm::GATECH);
      break;
    case pr2motion_TRAJ_PATH:
      left_arm.setTrajMode(RobotArm::PATH);
    default:
      return pr2motion_end;
    }
    left_arm.computeTrajectory();
    return pr2motion_checktraj;    
  }
  return pr2motion_end;
}

/** Codel checkTrajQGoal of activity Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_checktraj.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_launchmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error,
 *        pr2motion_joint_state_unavailable.
 */
genom_event
checkTrajQGoal(pr2motion_SIDE side, genom_context self)
{
  printf("CheckTrajQGoal\n");
  RobotArm::ERROR result;
  if(side == pr2motion_RIGHT){
    result=right_arm.validateTrajectory();
  } else {
    result=left_arm.validateTrajectory();
  }
  if(result == RobotArm::OK)
    return pr2motion_launchmove;
  else
    return pr2motion_end;
}

/** Codel launchMoveQ of activity Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_launchmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error,
 *        pr2motion_joint_state_unavailable.
 */
genom_event
launchMoveQ(pr2motion_SIDE side, genom_context self)
{
  printf("launchMoveQ\n");
  if(side == pr2motion_RIGHT){
    right_arm.move();
  } else {
    left_arm.move();
  }
  return pr2motion_waitmove;
}

/** Codel waitMoveQ of activity Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_waitmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error,
 *        pr2motion_joint_state_unavailable.
 */
genom_event
waitMoveQ(pr2motion_SIDE side, genom_context self)
{
  // printf("waitmovearmq\n");
  // if(side == pr2motion_RIGHT){
  //   if(right_arm.move_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
  //     return pr2motion_end;
  //   else if(right_arm.move_getState()==actionlib::SimpleClientGoalState::ACTIVE)
  //     return pr2motion_waitmove;
  //   else
  //     return pr2motion_end;
  // } else {
  //   if(left_arm.move_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
  //     return pr2motion_end;
  //   else if(left_arm.move_getState()==actionlib::SimpleClientGoalState::ACTIVE)
  //     return pr2motion_waitmove;
  //   else
  //     return pr2motion_end;
  // }
  // return pr2motion_waitmove;

  if(side == pr2motion_RIGHT){
    if(right_arm.move_isDone())
      return pr2motion_end;
    else
      return pr2motion_waitmove;
  } else {
    if(left_arm.move_isDone())
      return pr2motion_end;
    else
      return pr2motion_waitmove;    
  }
}

/** Codel endMoveQ of activity Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error,
 *        pr2motion_joint_state_unavailable.
 */
genom_event
endMoveQ(pr2motion_SIDE side, genom_context self)
{
  printf("endmovearmq\n");
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}

/** Codel stopMoveQ of activity Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 *        pr2motion_invalid_param, pr2motion_unknown_error,
 *        pr2motion_joint_state_unavailable.
 */
genom_event
stopMoveQ(pr2motion_SIDE side, genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}


/* --- Activity Arm_Right_Move ------------------------------------------ */

/** Codel getPathArm of activity Arm_Right_Move.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_computetraj, pr2motion_checktraj.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */


/** Codel computeTrajArm of activity Arm_Right_Move.
 *
 * Triggered by pr2motion_computetraj.
 * Yields to pr2motion_checktraj, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */


/** Codel checkTrajArm of activity Arm_Right_Move.
 *
 * Triggered by pr2motion_checktraj.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_launchmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */


/** Codel launchMoveArm of activity Arm_Right_Move.
 *
 * Triggered by pr2motion_launchmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */


/** Codel waitMoveArm of activity Arm_Right_Move.
 *
 * Triggered by pr2motion_waitmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */


/** Codel endMoveArm of activity Arm_Right_Move.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */


/** Codel stopMoveArm of activity Arm_Right_Move.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */



/* --- Activity Arm_Right_MoveToQGoal ----------------------------------- */

/** Codel getQGoal of activity Arm_Right_MoveToQGoal.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_computetraj, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */


/** Codel computeTrajQGoal of activity Arm_Right_MoveToQGoal.
 *
 * Triggered by pr2motion_computetraj.
 * Yields to pr2motion_checktraj, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */


/** Codel checkTrajQGoal of activity Arm_Right_MoveToQGoal.
 *
 * Triggered by pr2motion_checktraj.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_launchmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */


/** Codel launchMoveQ of activity Arm_Right_MoveToQGoal.
 *
 * Triggered by pr2motion_launchmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */


/** Codel waitMoveQ of activity Arm_Right_MoveToQGoal.
 *
 * Triggered by pr2motion_waitmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */


/** Codel endMoveQ of activity Arm_Right_MoveToQGoal.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */


/** Codel stopMoveQ of activity Arm_Right_MoveToQGoal.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */



/* --- Activity Arm_Left_Move ------------------------------------------- */

/** Codel getPathArm of activity Arm_Left_Move.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_computetraj, pr2motion_checktraj.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */


/** Codel computeTrajArm of activity Arm_Left_Move.
 *
 * Triggered by pr2motion_computetraj.
 * Yields to pr2motion_checktraj, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */


/** Codel checkTrajArm of activity Arm_Left_Move.
 *
 * Triggered by pr2motion_checktraj.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_launchmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */


/** Codel launchMoveArm of activity Arm_Left_Move.
 *
 * Triggered by pr2motion_launchmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */


/** Codel waitMoveArm of activity Arm_Left_Move.
 *
 * Triggered by pr2motion_waitmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */


/** Codel endMoveArm of activity Arm_Left_Move.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */


/** Codel stopMoveArm of activity Arm_Left_Move.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Arm_Move */



/* --- Activity Arm_Left_MoveToQGoal ------------------------------------ */

/** Codel getQGoal of activity Arm_Left_MoveToQGoal.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_computetraj, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */


/** Codel computeTrajQGoal of activity Arm_Left_MoveToQGoal.
 *
 * Triggered by pr2motion_computetraj.
 * Yields to pr2motion_checktraj, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */


/** Codel checkTrajQGoal of activity Arm_Left_MoveToQGoal.
 *
 * Triggered by pr2motion_checktraj.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_launchmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */


/** Codel launchMoveQ of activity Arm_Left_MoveToQGoal.
 *
 * Triggered by pr2motion_launchmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */


/** Codel waitMoveQ of activity Arm_Left_MoveToQGoal.
 *
 * Triggered by pr2motion_waitmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */


/** Codel endMoveQ of activity Arm_Left_MoveToQGoal.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */


/** Codel stopMoveQ of activity Arm_Left_MoveToQGoal.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 * Throws pr2motion_not_connected, pr2motion_init_not_done,
 * pr2motion_invalid_param, pr2motion_unknown_error,
 * pr2motion_joint_state_unavailable.
 */
/* already defined in service Arm_MoveToQGoal */



/* --- Activity GetQ ---------------------------------------------------- */

/** Codel getQ of activity GetQ.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_end, pr2motion_ether.
 * Throws pr2motion_joint_state_unavailable,
 *        pr2motion_joint_name_unknown.
 */
genom_event
getQ(const char *joint_name, bool joint_state_availability,
     const pr2motion_sensor_msgs_jointstate *joint_state_msg,
     double *position, double *velocity, double *effort,
     genom_context self)
{
  bool find=false;
  // check if we can get the actual arm position
  if(!joint_state_availability){
    printf("GetQGoal cannot get robot actual position\n");
    return pr2motion_joint_state_unavailable(self);
  }

  for (size_t ind=0; ind<joint_state_msg->name._length; ind++) {
    if(strcmp(joint_state_msg->name._buffer[ind],joint_name)==0){
       printf("r_shoulder_pan_joint\n");
       *position=joint_state_msg->position._buffer[ind];
       *velocity=joint_state_msg->velocity._buffer[ind];
       *effort=joint_state_msg->effort._buffer[ind];
       find=true;
    }
  }
  
  if(!find)
    return pr2motion_joint_name_unknown(self);

  return pr2motion_end;
}

/** Codel endGetQ of activity GetQ.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_joint_state_unavailable,
 *        pr2motion_joint_name_unknown.
 */
genom_event
endGetQ(genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}
