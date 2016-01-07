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


/* --- Task main -------------------------------------------------------- */

#ifndef PR2_SIMU
// we are on the real robot
extern Gripper left_gripper;
extern Gripper right_gripper;
#else
// we are in simulation
extern GripperSimple left_gripper;
extern GripperSimple right_gripper;
#endif

extern Torso torso;
extern RobotHead head;
extern RobotArm left_arm;
extern RobotArm right_arm;
extern RobotState robot_state; 

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
 * Yields to pr2motion_sleep.
 */
genom_event
initMain(double torso_min_duration, float torso_max_velocity,
         genom_context self)
{
  char* argv[] =  { "pr2motion",
                    "",
                    NULL};
  int argc = 2;
  ros::init(argc, argv, "pr2motion");
#ifndef PR2_SIMU
  left_gripper.init(Gripper::LEFT);
  right_gripper.init(Gripper::RIGHT);
#else
  left_gripper.init(GripperSimple::LEFT);
  right_gripper.init(GripperSimple::RIGHT);
#endif
  Torso::ERROR torso_init_result;
  torso_init_result=torso.init();
  RobotHead::ERROR head_init_result;
  head_init_result=head.init();
  RobotArm::ERROR right_arm_init_result;
  right_arm_init_result = right_arm.init(RobotArm::RIGHT);
  RobotArm::ERROR left_arm_init_result;
  left_arm_init_result = left_arm.init(RobotArm::LEFT);
  
  if((torso_init_result!=Torso::OK) || (head_init_result!=RobotHead::OK) || (right_arm_init_result!=RobotArm::OK) || (left_arm_init_result!=RobotArm::OK)){
    printf("there is an error concerning almost one controller initialisation !");
    return pr2motion_end;
  }

  torso_min_duration = 2.0;
  torso_max_velocity = 0.5;

    
  //  Gripper_SetOpenGoal(0.09,-1.0,self);
  //Gripper_SetCloseGoal(0.0,-1.0,self);
  //Gripper_SetFindContactGoal(pr2motion_PR2MOTION_GRIPPER_CONTACT_BOTH, true);
  //Gripper_SetEventDetectorGoal(pr2motion_PR2MOTION_GRIPPER_FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC, 4.0, 0.005);
  //Gripper_SetForceServoGoal(10);
  //Torso_SetOpenGoal(10.0, 2.0, 1.0,self);
  
 // // SDI Init
 //  //Pr2GripperCommandGoal
 //  open_position=0.09;
 //  open_max_effort=-1.0;
 //  //Pr2GripperCommandGoal
 //  close_position=0.08;
 //  close_max_effort=-1;
 //  //PR2GripperFindContactGoal
 //  findtwo_contact_conditions=pr2motion_PR2MOTION_GRIPPER_CONTACT_BOTH;
 //  findtwo_zero_fingertip_sensors=TRUE;
 //  //PR2GripperEventDetectorGoal
 //  event_trigger_conditions=pr2motion_PR2MOTION_GRIPPER_FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;
 //  event_acceleration_trigger_magnitude=4.0;
 //  event_slip_trigger_magnitude=.005;
 //  //PR2GripperForceServoCommand
 //  force_fingertip_force=10;  

  //  Gripper gripper(pr2motion_PR2MOTION_RIGHT);
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_sleep;
}


/** Codel endMain of task main.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 */
genom_event
endMain(genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}


/* --- Activity PR2MOTION_Init ------------------------------------------ */

/** Codel startInit of activity PR2MOTION_Init.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_ether.
 * Throws pr2motion_serverconnection_error.
 */
genom_event
startInit(genom_context self)
{
 
 
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}


/* --- Activity PR2MOTION_Gripper_OperateGripper ------------------------ */

/** Codel startOperateGripper of activity PR2MOTION_Gripper_OperateGripper.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_exec, pr2motion_stop, pr2motion_ether.
 * Throws pr2motion_serverconnection_error.
 */
genom_event
startOperateGripper(pr2motion_PR2MOTION_SIDE side,
                    pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                    genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_exec;
}

/** Codel execOperateGripper of activity PR2MOTION_Gripper_OperateGripper.
 *
 * Triggered by pr2motion_exec.
 * Yields to pr2motion_exec, pr2motion_wait, pr2motion_waitcontact, pr2motion_waitopen, pr2motion_waitclose, pr2motion_waitrelease, pr2motion_stop, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_serverconnection_error.
 */
#ifndef PR2_SIMU
// we are on the real robot
genom_event
execOperateGripper(pr2motion_PR2MOTION_SIDE side,
                   pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                   double open_position, double open_max_effort,
                   double close_position, double close_max_effort,
                   pr2motion_PR2MOTION_GRIPPER_CONTACT_CONDITIONS findtwo_contact_conditions,
                   bool findtwo_zero_fingertip_sensors,
                   pr2motion_PR2MOTION_GRIPPER_EVENT_DETECTOR event_trigger_conditions,
                   double event_acceleration_trigger_magnitude,
                   double event_slip_trigger_magnitude,
                   genom_context self)
{
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
  case pr2motion_PR2MOTION_LEFT :
    switch(goal_mode){
    case pr2motion_PR2MOTION_GRIPPER_GRAB :
      left_gripper.findTwoContacts(findTwo);
      return pr2motion_waitcontact;
    case pr2motion_PR2MOTION_GRIPPER_RELEASE :
      left_gripper.place(place);
      return pr2motion_waitrelease;
    case pr2motion_PR2MOTION_GRIPPER_OPEN :
      left_gripper.open(open);
      return pr2motion_waitopen;
    case pr2motion_PR2MOTION_GRIPPER_CLOSE :
      left_gripper.close(close);
      return pr2motion_waitclose;
    }
  case pr2motion_PR2MOTION_RIGHT:
    switch(goal_mode){
    case pr2motion_PR2MOTION_GRIPPER_GRAB :
      right_gripper.findTwoContacts(findTwo);
      return pr2motion_waitcontact;
    case pr2motion_PR2MOTION_GRIPPER_RELEASE :
      return pr2motion_waitrelease;
    case pr2motion_PR2MOTION_GRIPPER_OPEN :
      right_gripper.open(open);
      return pr2motion_waitopen;
    case pr2motion_PR2MOTION_GRIPPER_CLOSE :
      right_gripper.close(close);
      return pr2motion_waitclose;
    }
  }

  /* skeleton sample */ return pr2motion_wait;
}
#else
// we are in simulation
genom_event
execOperateGripper(pr2motion_PR2MOTION_SIDE side,
                   pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                   double open_position, double open_max_effort,
                   double close_position, double close_max_effort,
                   genom_context self)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = open_position;    // position open (9 cm)
  open.command.max_effort = open_max_effort;  // unlimited motor effort
  pr2_controllers_msgs::Pr2GripperCommandGoal close;
  close.command.position = close_position;    // position open (9 cm)
  close.command.max_effort = close_max_effort;  // unlimited motor effort

  switch(side){
  case pr2motion_PR2MOTION_LEFT :
    switch(goal_mode){
    case pr2motion_PR2MOTION_GRIPPER_OPEN :
      left_gripper.open(open);
      return pr2motion_waitopen;
    case pr2motion_PR2MOTION_GRIPPER_CLOSE :
      left_gripper.close(close);
      return pr2motion_waitclose;
    }
  case pr2motion_PR2MOTION_RIGHT:
    switch(goal_mode){
    case pr2motion_PR2MOTION_GRIPPER_OPEN :
      right_gripper.open(open);
      return pr2motion_waitopen;
    case pr2motion_PR2MOTION_GRIPPER_CLOSE :
      right_gripper.close(close);
      return pr2motion_waitclose;
    }
  }

  /* skeleton sample */ return pr2motion_wait;
}
#endif

/** Codel waitOperateGripper of activity PR2MOTION_Gripper_OperateGripper.
 *
 * Triggered by pr2motion_wait.
 * Yields to pr2motion_wait, pr2motion_exec, pr2motion_stop, pr2motion_end, pr2motion_ether.
 * Throws pr2motion_serverconnection_error.
 */
genom_event
waitOperateGripper(pr2motion_PR2MOTION_SIDE side,
                   pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                   genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_end;
}

#ifndef PR2_SIMU
/** Codel waitcontactOperateGripper of activity PR2MOTION_Gripper_OperateGripper.
 *
 * Triggered by pr2motion_waitcontact.
 * Yields to pr2motion_slipservo, pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_serverconnection_error.
 */
genom_event
waitcontactOperateGripper(pr2motion_PR2MOTION_SIDE side,
                          pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                          genom_context self)
{
  if (side >= pr2motion_PR2MOTION_NB_SIDE)
    return pr2motion_end;    

  switch(side){
  case pr2motion_PR2MOTION_LEFT :
    if(left_gripper.findTwo_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_slipservo;
    else if(left_gripper.findTwo_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitcontact;
    else
      return pr2motion_end;
  case pr2motion_PR2MOTION_RIGHT :
    if(left_gripper.findTwo_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_slipservo;
    else if(left_gripper.findTwo_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitcontact;
    else
      return pr2motion_end;    
  }
}
#endif
  
#ifndef PR2_SIMU
/** Codel waitopenOperateGripper of activity PR2MOTION_Gripper_OperateGripper.
 *
 * Triggered by pr2motion_waitopen.
 * Yields to pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_serverconnection_error.
 */
genom_event
waitopenOperateGripper(pr2motion_PR2MOTION_SIDE side,
                       pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                       genom_context self)
{
  if (side >= pr2motion_PR2MOTION_NB_SIDE)
    return pr2motion_end;    

  switch(side){
  case pr2motion_PR2MOTION_LEFT :
    if(left_gripper.open_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_wait;
    else if(left_gripper.open_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitcontact;
    else
      return pr2motion_end;
  case pr2motion_PR2MOTION_RIGHT :
    if(left_gripper.open_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_wait;
    else if(left_gripper.open_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitcontact;
    else
      return pr2motion_end;    
  }
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_wait;
}
#else
genom_event
waitopenOperateGripper(pr2motion_PR2MOTION_SIDE side,
                       pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                       genom_context self)
{
  if (side >= pr2motion_PR2MOTION_NB_SIDE)
    return pr2motion_end;    

  switch(side){
  case pr2motion_PR2MOTION_LEFT :
    if(left_gripper.open_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_end;
   else if(left_gripper.open_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitopen;
    else
      return pr2motion_end;
  case pr2motion_PR2MOTION_RIGHT :
    if(right_gripper.open_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_end;
    else if(right_gripper.open_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitopen;
    else
      return pr2motion_end;    
  }
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_wait;
}
#endif

#ifndef PR2_SIMU
/** Codel waitcloseOperateGripper of activity PR2MOTION_Gripper_OperateGripper.
 *
 * Triggered by pr2motion_waitclose.
 * Yields to pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_serverconnection_error.
 */
genom_event
waitcloseOperateGripper(pr2motion_PR2MOTION_SIDE side,
                        pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                        genom_context self)
{
  if (side >= pr2motion_PR2MOTION_NB_SIDE)
    return pr2motion_end;    

  switch(side){
  case pr2motion_PR2MOTION_LEFT :
    if(left_gripper.close_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_end;
    else if(left_gripper.close_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitclose;
    else
      return pr2motion_end;
  case pr2motion_PR2MOTION_RIGHT :
    if(left_gripper.close_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_end;
    else if(left_gripper.close_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitcontact;
    else
      return pr2motion_end;    
  }  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_wait;
}
#else
genom_event
waitcloseOperateGripper(pr2motion_PR2MOTION_SIDE side,
                        pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                        genom_context self)
{
  if (side >= pr2motion_PR2MOTION_NB_SIDE)
    return pr2motion_end;    

  switch(side){
  case pr2motion_PR2MOTION_LEFT :
    if(left_gripper.close_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_end;
    else if(left_gripper.close_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitclose;
    else
      return pr2motion_end;
  case pr2motion_PR2MOTION_RIGHT :
    if(right_gripper.close_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_end;
    else if(right_gripper.close_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitclose;
    else
      return pr2motion_end;    
  }  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_wait;
}
#endif

#ifndef PR2_SIMU
/** Codel waitreleaseOperateGripper of activity PR2MOTION_Gripper_OperateGripper.
 *
 * Triggered by pr2motion_waitrelease.
 * Yields to pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_serverconnection_error.
 */
genom_event
waitreleaseOperateGripper(pr2motion_PR2MOTION_SIDE side,
                          pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                          genom_context self)
{
  if (side >= pr2motion_PR2MOTION_NB_SIDE)
    return pr2motion_end;    

  switch(side){
  case pr2motion_PR2MOTION_LEFT :
    if(left_gripper.place_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_wait;
    else if(left_gripper.place_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitcontact;
    else
      return pr2motion_end;
  case pr2motion_PR2MOTION_RIGHT :
    if(left_gripper.place_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_wait;
    else if(left_gripper.place_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitcontact;
    else
      return pr2motion_end;    
  }
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_wait;
}
#endif

#ifndef PR2_SIMU
/** Codel slipservoOperateGripper of activity PR2MOTION_Gripper_OperateGripper.
 *
 * Triggered by pr2motion_slipservo.
 * Yields to pr2motion_wait, pr2motion_stop, pr2motion_end.
 * Throws pr2motion_serverconnection_error.
 */
genom_event
slipservoOperateGripper(pr2motion_PR2MOTION_SIDE side,
                        pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                        genom_context self)
{
  if (side >= pr2motion_PR2MOTION_NB_SIDE)
    return pr2motion_end;    

  switch(side){
  case pr2motion_PR2MOTION_LEFT :
    left_gripper.slipServo();
    return pr2motion_wait;
  case pr2motion_PR2MOTION_RIGHT :
    right_gripper.slipServo();
    return pr2motion_wait;
  }
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_wait;
}
#endif

/** Codel stopOperateGripper of activity PR2MOTION_Gripper_OperateGripper.
 *
 * Triggered by pr2motion_stop.
 * Yields to pr2motion_ether.
 * Throws pr2motion_serverconnection_error.
 */
genom_event
stopOperateGripper(pr2motion_PR2MOTION_SIDE side,
                   pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                   genom_context self)
{
  if (side >= pr2motion_PR2MOTION_NB_SIDE)
    return pr2motion_end;    
  
  if (goal_mode >= pr2motion_PR2MOTION_GRIPPER_NB_MODE)
    return pr2motion_end;

  switch(side){
  case pr2motion_PR2MOTION_LEFT :
    switch(goal_mode){
#ifndef PR2_SIMU
    case pr2motion_PR2MOTION_GRIPPER_GRAB :
      left_gripper.findTwo_cancel();
    case pr2motion_PR2MOTION_GRIPPER_RELEASE :
      left_gripper.place_cancel();
      return pr2motion_waitrelease;
#endif
    case pr2motion_PR2MOTION_GRIPPER_OPEN :
      left_gripper.open_cancel();
    case pr2motion_PR2MOTION_GRIPPER_CLOSE :
      left_gripper.close_cancel();
    }
  case pr2motion_PR2MOTION_RIGHT :
    switch(goal_mode){
#ifndef PR2_SIMU
    case pr2motion_PR2MOTION_GRIPPER_GRAB :
      right_gripper.findTwo_cancel();
    case pr2motion_PR2MOTION_GRIPPER_RELEASE :
      right_gripper.place_cancel();
      return pr2motion_waitrelease;
#endif
    case pr2motion_PR2MOTION_GRIPPER_OPEN :
      right_gripper.open_cancel();
    case pr2motion_PR2MOTION_GRIPPER_CLOSE :
      right_gripper.close_cancel();
    }   
  }
  return pr2motion_end;
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}

/** Codel endOperateGripper of activity PR2MOTION_Gripper_OperateGripper.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 * Throws pr2motion_serverconnection_error.
 */
genom_event
endOperateGripper(pr2motion_PR2MOTION_SIDE side,
                  pr2motion_PR2MOTION_GRIPPER_MODE goal_mode,
                  genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}


/* --- Activity PR2MOTION_Torso_MoveTorso ------------------------------- */

/** Codel startMoveTorso of activity PR2MOTION_Torso_MoveTorso.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_wait.
 */
genom_event
startMoveTorso(float torso_position, double torso_min_duration,
               float torso_max_velocity, genom_context self)
{
  Torso::ERROR result;
  pr2_controllers_msgs::SingleJointPositionGoal torso_cmd;
  torso_cmd.position=torso_position;
  torso_cmd.min_duration=ros::Duration(torso_min_duration);
  torso_cmd.max_velocity=torso_max_velocity;
  result = torso.move(torso_cmd);
  if(result==Torso::OK)
    return pr2motion_wait;
  else
    return pr2motion_end;
}

/** Codel waitMoveTorso of activity PR2MOTION_Torso_MoveTorso.
 *
 * Triggered by pr2motion_wait.
 * Yields to pr2motion_wait, pr2motion_end, pr2motion_ether.
 */

genom_event
waitMoveTorso(genom_context self)
{
  // INTERMEDIATE STATES : PENDING ACTIVE
  // TERMINAL STATES : REJECTED SUCCEEDED ABORTED RECALLED PREEMPTED LOST 	
  
  // if the state is a terminal state, go to end
  // otherwise wait

  if(torso.move_getState()==actionlib::SimpleClientGoalState::SUCCEEDED) {
      return pr2motion_end;
  } else if(torso.move_getState()==actionlib::SimpleClientGoalState::REJECTED) {
    return pr2motion_end;    
  } else if(torso.move_getState()==actionlib::SimpleClientGoalState::ABORTED) {
   return pr2motion_end;        
  } else if(torso.move_getState()==actionlib::SimpleClientGoalState::RECALLED) {
   return pr2motion_end;        
  } else if(torso.move_getState()==actionlib::SimpleClientGoalState::PREEMPTED) {
   return pr2motion_end;     
  } else if(torso.move_getState()==actionlib::SimpleClientGoalState::LOST) {
   return pr2motion_end;     
  }

  if(torso.move_getState()==actionlib::SimpleClientGoalState::ACTIVE) {
    return pr2motion_wait;
  } else if(torso.move_getState()==actionlib::SimpleClientGoalState::PENDING) {
    return pr2motion_wait;    
  }

  printf("The actual state of the torso is unknown !\n");
  return pr2motion_end;
}

/** Codel endMoveTorso of activity PR2MOTION_Torso_MoveTorso.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 */
genom_event
endMoveTorso(genom_context self)
{
  return pr2motion_ether;
}


/* --- Activity PR2MOTION_Head_MoveHead --------------------------------- */

/** Codel startMoveHead of activity PR2MOTION_Head_MoveHead.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_wait.
 */
genom_event
startMoveHead(pr2motion_PR2MOTION_HEAD_MODE head_mode,
              const char head_target_frame[128], double head_target_x,
              double head_target_y, double head_target_z,
              genom_context self)
{
  std::string frame_id(head_target_frame);
  if(head_mode>=pr2motion_PR2MOTION_HEAD_NB_MODE)
    return pr2motion_end;
  
  head.lookAt(frame_id, head_target_x, head_target_y, head_target_z);

  return pr2motion_wait;
}

/** Codel waitMoveHead of activity PR2MOTION_Head_MoveHead.
 *
 * Triggered by pr2motion_wait.
 * Yields to pr2motion_start, pr2motion_wait, pr2motion_end, pr2motion_ether.
 */
genom_event
waitMoveHead(pr2motion_PR2MOTION_HEAD_MODE head_mode,
             genom_context self)
{
  // INTERMEDIATE STATES : PENDING ACTIVE
  // TERMINAL STATES : REJECTED SUCCEEDED ABORTED RECALLED PREEMPTED LOST 	
  
  // if the state is a terminal state, go to end
  // otherwise wait

  if(head.lookAt_getState()==actionlib::SimpleClientGoalState::SUCCEEDED) {
    if(head_mode==pr2motion_PR2MOTION_HEAD_LOOKAT) {
      return pr2motion_end;
    } else {
      return pr2motion_start;
    }
  } else if(head.lookAt_getState()==actionlib::SimpleClientGoalState::REJECTED) {
    return pr2motion_end;    
  } else if(head.lookAt_getState()==actionlib::SimpleClientGoalState::ABORTED) {
   return pr2motion_end;        
  } else if(head.lookAt_getState()==actionlib::SimpleClientGoalState::RECALLED) {
   return pr2motion_end;        
  } else if(head.lookAt_getState()==actionlib::SimpleClientGoalState::PREEMPTED) {
   return pr2motion_end;     
  } else if(head.lookAt_getState()==actionlib::SimpleClientGoalState::LOST) {
   return pr2motion_end;     
  }

  if(head.lookAt_getState()==actionlib::SimpleClientGoalState::ACTIVE) {
    return pr2motion_wait;
  } else if(head.lookAt_getState()==actionlib::SimpleClientGoalState::PENDING) {
    return pr2motion_wait;    
  }

  printf("The actual state of the head is unknown !\n");
  return pr2motion_end;
}

/** Codel endMoveHead of activity PR2MOTION_Head_MoveHead.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 */
genom_event
endMoveHead(genom_context self)
{
  return pr2motion_ether;
}


/* --- Activity PR2MOTION_Head_StopHead --------------------------------- */

/** Codel stopHead of activity PR2MOTION_Head_StopHead.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_ether.
 */
genom_event
stopHead(genom_context self)
{
  head.cancelCmd();
  return pr2motion_ether;
}


/* --- Activity PR2MOTION_Arm_Move -------------------------------------- */

/** Codel getPathArm of activity PR2MOTION_Arm_Move.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_computetraj, pr2motion_checktraj.
 */
genom_event
getPathArm(pr2motion_PR2MOTION_SIDE side,
           pr2motion_PR2MOTION_PATH_MODE path_mode,
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

  if (side >= pr2motion_PR2MOTION_NB_SIDE)
    return pr2motion_end;    

  if (path_mode >= pr2motion_PR2MOTION_PATH_NB_MODE)
    return pr2motion_end;

  // read trajectory from the test function
  if(path_mode==pr2motion_PR2MOTION_PATH_TEST) {
    if (side == pr2motion_PR2MOTION_RIGHT) {
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
      if(traj->data(self)->traj.joint_names._buffer[ind] == "l_wrist_roll_joint"){
	l_wrist_roll_joint_indice = ind;
      }
    }
    
    if(side == pr2motion_PR2MOTION_RIGHT){
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

/** Codel computeTrajArm of activity PR2MOTION_Arm_Move.
 *
 * Triggered by pr2motion_computetraj.
 * Yields to pr2motion_checktraj, pr2motion_end, pr2motion_ether.
 */
genom_event
computeTrajArm(pr2motion_PR2MOTION_SIDE side,
               pr2motion_PR2MOTION_TRAJ_MODE traj_mode,
               double time_slot, double max_vel, double max_acc,
               double max_jerk, genom_context self)
{
  printf("computetrajarm\n");
  if(side == pr2motion_PR2MOTION_RIGHT){
    right_arm.setMax(max_vel, max_acc, max_jerk);
    right_arm.setT(time_slot);
    switch (traj_mode){
    case pr2motion_PR2MOTION_TRAJ_SOFTMOTION:
      right_arm.setTrajMode(RobotArm::SOFT_MOTION);
      break;
    case pr2motion_PR2MOTION_TRAJ_GATECH:
      right_arm.setTrajMode(RobotArm::GATECH);
      break;
    case pr2motion_PR2MOTION_TRAJ_PATH:
      right_arm.setTrajMode(RobotArm::PATH);
    default:
      return pr2motion_end;
    }
    right_arm.computeTrajectory();
    return pr2motion_checktraj;
  } else {
    left_arm.setMax(max_vel, max_acc, max_jerk);
    left_arm.setT(time_slot);
    switch (traj_mode){
    case pr2motion_PR2MOTION_TRAJ_SOFTMOTION:
      left_arm.setTrajMode(RobotArm::SOFT_MOTION);
      break;
    case pr2motion_PR2MOTION_TRAJ_GATECH:
      left_arm.setTrajMode(RobotArm::GATECH);
      break;
    case pr2motion_PR2MOTION_TRAJ_PATH:
      left_arm.setTrajMode(RobotArm::PATH);
    default:
      return pr2motion_end;
    }
    left_arm.computeTrajectory();
    return pr2motion_checktraj;    
  }
  return pr2motion_end;
}

/** Codel checkTrajArm of activity PR2MOTION_Arm_Move.
 *
 * Triggered by pr2motion_checktraj.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_launchmove.
 */
genom_event
checkTrajArm(pr2motion_PR2MOTION_SIDE side, genom_context self)
{
  printf("checktrajarm\n");
  RobotArm::ERROR result;
  if(side == pr2motion_PR2MOTION_RIGHT){
    result=right_arm.validateTrajectory();
  } else {
    result=left_arm.validateTrajectory();
  }
  if(result == RobotArm::OK)
    return pr2motion_launchmove;
  else
    return pr2motion_end;
}

/** Codel launchMoveArm of activity PR2MOTION_Arm_Move.
 *
 * Triggered by pr2motion_launchmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 */
genom_event
launchMoveArm(pr2motion_PR2MOTION_SIDE side, genom_context self)
{
  printf("launchmovearm\n");
  if(side == pr2motion_PR2MOTION_RIGHT){
    right_arm.move();
  } else {
    left_arm.move();
  }
  return pr2motion_waitmove;
}

/** Codel waitMoveArm of activity PR2MOTION_Arm_Move.
 *
 * Triggered by pr2motion_waitmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 */
genom_event
waitMoveArm(pr2motion_PR2MOTION_SIDE side, genom_context self)
{
  printf("waitmovearm\n");
  if(side == pr2motion_PR2MOTION_RIGHT){
    if(right_arm.move_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_end;
    else if(right_arm.move_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitmove;
    else
      return pr2motion_end;
  } else {
    if(left_arm.move_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_end;
    else if(left_arm.move_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitmove;
    else
      return pr2motion_end;
  }
  return pr2motion_waitmove;
}

/** Codel endMoveArm of activity PR2MOTION_Arm_Move.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 */
genom_event
endMoveArm(pr2motion_PR2MOTION_SIDE side, genom_context self)
{
  printf("endmovearm\n");
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}


/* --- Activity PR2MOTION_Arm_MoveToQGoal ------------------------------- */

/** Codel getQGoal of activity PR2MOTION_Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_computetraj, pr2motion_end, pr2motion_ether.
 */
genom_event
getQGoal(pr2motion_PR2MOTION_SIDE side, double shoulder_pan_joint,
         double shoulder_lift_joint, double upper_arm_roll_joint,
         double elbow_flex_joint, double forearm_roll_joint,
         double wrist_flex_joint, double wrist_roll_joint,
         genom_context self)
{
  // goal (path) to be send to the controller
  pr2_controllers_msgs::JointTrajectoryGoal path_cmd;
  int joint_names_vector_size = 7;
  if(side == pr2motion_PR2MOTION_RIGHT){
    path_cmd.trajectory.joint_names.resize(joint_names_vector_size);
    path_cmd.trajectory.joint_names[0]="r_shoulder_pan_joint";
    path_cmd.trajectory.joint_names[1]="r_shoulder_lift_joint";
    path_cmd.trajectory.joint_names[2]="r_upper_arm_roll_joint";
    path_cmd.trajectory.joint_names[3]="r_elbow_flex_joint";
    path_cmd.trajectory.joint_names[4]="r_forearm_roll_joint";
    path_cmd.trajectory.joint_names[5]="r_wrist_flex_joint";
    path_cmd.trajectory.joint_names[6]="r_wrist_roll_joint";
    path_cmd.trajectory.points[0].positions.resize(joint_names_vector_size);
    path_cmd.trajectory.points[0].positions[0] = shoulder_pan_joint;
    path_cmd.trajectory.points[0].positions[1] = shoulder_lift_joint;
    path_cmd.trajectory.points[0].positions[2] = upper_arm_roll_joint;
    path_cmd.trajectory.points[0].positions[3] = elbow_flex_joint;
    path_cmd.trajectory.points[0].positions[4] = forearm_roll_joint;
    path_cmd.trajectory.points[0].positions[5] = wrist_flex_joint;
    path_cmd.trajectory.points[0].positions[6] = wrist_roll_joint;
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
    path_cmd.trajectory.points[0].positions.resize(joint_names_vector_size);
    path_cmd.trajectory.points[0].positions[0] = shoulder_pan_joint;
    path_cmd.trajectory.points[0].positions[1] = shoulder_lift_joint;
    path_cmd.trajectory.points[0].positions[2] = upper_arm_roll_joint;
    path_cmd.trajectory.points[0].positions[3] = elbow_flex_joint;
    path_cmd.trajectory.points[0].positions[4] = forearm_roll_joint;
    path_cmd.trajectory.points[0].positions[5] = wrist_flex_joint;
    path_cmd.trajectory.points[0].positions[6] = wrist_roll_joint;   
    left_arm.clearTrajectory();
    if(left_arm.setTraj(&path_cmd) == RobotArm::OK)
      return pr2motion_computetraj;
    else
      return pr2motion_end;     
  }
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_computetraj;
}

/** Codel computeTrajQGoal of activity PR2MOTION_Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_computetraj.
 * Yields to pr2motion_checktraj, pr2motion_end, pr2motion_ether.
 */
genom_event
computeTrajQGoal(pr2motion_PR2MOTION_SIDE side,
                 pr2motion_PR2MOTION_TRAJ_MODE traj_mode,
                 double time_slot, double max_vel, double max_acc,
                 double max_jerk, genom_context self)
{
  printf("computetrajarmq\n");
  if(side == pr2motion_PR2MOTION_RIGHT){
    right_arm.setMax(max_vel, max_acc, max_jerk);
    right_arm.setT(time_slot);
    switch (traj_mode){
    case pr2motion_PR2MOTION_TRAJ_SOFTMOTION:
      right_arm.setTrajMode(RobotArm::SOFT_MOTION);
      break;
    case pr2motion_PR2MOTION_TRAJ_GATECH:
      right_arm.setTrajMode(RobotArm::GATECH);
      break;
    case pr2motion_PR2MOTION_TRAJ_PATH:
      right_arm.setTrajMode(RobotArm::PATH);
    default:
      return pr2motion_end;
    }
    right_arm.computeTrajectory();
    return pr2motion_checktraj;
  } else {
    left_arm.setMax(max_vel, max_acc, max_jerk);
    left_arm.setT(time_slot);
    switch (traj_mode){
    case pr2motion_PR2MOTION_TRAJ_SOFTMOTION:
      left_arm.setTrajMode(RobotArm::SOFT_MOTION);
      break;
    case pr2motion_PR2MOTION_TRAJ_GATECH:
      left_arm.setTrajMode(RobotArm::GATECH);
      break;
    case pr2motion_PR2MOTION_TRAJ_PATH:
      left_arm.setTrajMode(RobotArm::PATH);
    default:
      return pr2motion_end;
    }
    left_arm.computeTrajectory();
    return pr2motion_checktraj;    
  }
  return pr2motion_end;
}

/** Codel checkTrajQGoal of activity PR2MOTION_Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_checktraj.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_launchmove.
 */
genom_event
checkTrajQGoal(pr2motion_PR2MOTION_SIDE side, genom_context self)
{
  printf("checktrajarmq\n");
  RobotArm::ERROR result;
  if(side == pr2motion_PR2MOTION_RIGHT){
    result=right_arm.validateTrajectory();
  } else {
    result=left_arm.validateTrajectory();
  }
  if(result == RobotArm::OK)
    return pr2motion_launchmove;
  else
    return pr2motion_end;
}

/** Codel launchMoveQ of activity PR2MOTION_Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_launchmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 */
genom_event
launchMoveQ(pr2motion_PR2MOTION_SIDE side, genom_context self)
{
  printf("launchmovearmq\n");
  if(side == pr2motion_PR2MOTION_RIGHT){
    right_arm.move();
  } else {
    left_arm.move();
  }
  return pr2motion_waitmove;
}

/** Codel waitMoveQ of activity PR2MOTION_Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_waitmove.
 * Yields to pr2motion_end, pr2motion_ether, pr2motion_waitmove.
 */
genom_event
waitMoveQ(pr2motion_PR2MOTION_SIDE side, genom_context self)
{
  printf("waitmovearmq\n");
  if(side == pr2motion_PR2MOTION_RIGHT){
    if(right_arm.move_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_end;
    else if(right_arm.move_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitmove;
    else
      return pr2motion_end;
  } else {
    if(left_arm.move_getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      return pr2motion_end;
    else if(left_arm.move_getState()==actionlib::SimpleClientGoalState::ACTIVE)
      return pr2motion_waitmove;
    else
      return pr2motion_end;
  }
  return pr2motion_waitmove;
}

/** Codel endMoveQ of activity PR2MOTION_Arm_MoveToQGoal.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 */
genom_event
endMoveQ(pr2motion_PR2MOTION_SIDE side, genom_context self)
{
  printf("endmovearmq\n");
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}


/* --- Activity PR2MOTION_GetQ ------------------------------------------ */

/** Codel getQ of activity PR2MOTION_GetQ.
 *
 * Triggered by pr2motion_start.
 * Yields to pr2motion_end, pr2motion_ether.
 */
genom_event
getQ(const pr2motion_joint_state *joint_state, genom_context self)
{
  int name_size=0;
  int position_size=0;
  int velocity_size=0;
  int effort_size=0;

  printf("allo allo\n");
  sensor_msgs::JointState joint_state_msg;
  // read the trajectory from the port
  joint_state->read(self);
  if(joint_state->data(self)!=NULL){
    // nb_points in the path
    name_size = joint_state->data(self)->name._length;
    position_size = joint_state->data(self)->position._length;
    velocity_size = joint_state->data(self)->velocity._length;
    effort_size = joint_state->data(self)->effort._length;

    if(name_size==0){
      printf("the joint_state is empty !\n");
      return pr2motion_end;
    }
    
    if( (name_size!=position_size) ||
	(name_size!=velocity_size) ||
	(name_size!=effort_size)){
      printf("issue concerning the size of the joint_state vector\n");
      return pr2motion_end;
    }
    joint_state_msg.name.resize(name_size);
    joint_state_msg.position.resize(position_size);
    joint_state_msg.velocity.resize(velocity_size);
    joint_state_msg.effort.resize(effort_size);
    // get the correct index for all needed joints in the path
    for (size_t ind=0; ind<name_size; ind++) {
      printf("%s position %f velocity %f effort %f\n",joint_state->data(self)->name._buffer[ind],joint_state->data(self)->position._buffer[ind], joint_state->data(self)->velocity._buffer[ind], joint_state->data(self)->effort._buffer[ind]); 
      joint_state_msg.name[ind]=joint_state->data(self)->name._buffer[ind];
      joint_state_msg.position[ind]=joint_state->data(self)->position._buffer[ind];
      joint_state_msg.velocity[ind]=joint_state->data(self)->velocity._buffer[ind];
      joint_state_msg.effort[ind]=joint_state->data(self)->effort._buffer[ind];
      }
     robot_state.setRobotQ(joint_state_msg);
  } else {
    printf("nothing to read on the port...\n");
  }
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_end;
}

/** Codel endGetQ of activity PR2MOTION_GetQ.
 *
 * Triggered by pr2motion_end.
 * Yields to pr2motion_ether.
 */
genom_event
endGetQ(genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return pr2motion_ether;
}
