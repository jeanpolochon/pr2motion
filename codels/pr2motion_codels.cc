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
//#include "pr2motion.hh"
#include "acpr2motion.h"
#include "pr2motion_c_types.h"

#ifndef PR2_SIMU
#include "pr2_gripper_sensor_client.hh"
#else
#include "pr2_gripper_client.hh"
#endif
#include "pr2_torso_client.hh"
#include "pr2_head_client.hh"
#include "pr2_arm_client.hh"
#include "pr2_state_client.hh"

#ifndef PR2_SIMU
// we are on the real robot
extern Gripper left_gripper;
extern Gripper right_gripper;
#else
// we are in simulation
extern GripperSimple left_gripper;
extern GripperSimple right_gripper;
#endif

extern RobotHead head;
extern Torso torso;
extern RobotArm left_arm;
extern RobotArm right_arm;


/* --- Function Z_Gripper_SetOpenParam ---------------------------------- */

/** Codel setOpenParam of function Z_Gripper_SetOpenParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
setOpenParam(pr2motion_SIDE side, double open_position,
             double open_max_effort, genom_context self)
{
#ifndef PR2_SIMU
  Gripper::ERROR open_position_result=Gripper::OK;
  Gripper::ERROR open_max_effort_result=Gripper::OK;
#else
  GripperSimple::ERROR open_position_result=GripperSimple::OK;
  GripperSimple::ERROR open_max_effort_result=GripperSimple::OK;
#endif

  if (side >= pr2motion_NB_SIDE)
    return pr2motion_invalid_param(self);   

  switch(side){
  case pr2motion_LEFT :
    open_position_result = left_gripper.setOpenPosition(open_position);
    open_max_effort_result = left_gripper.setOpenMaxEffort(open_max_effort);
    break;
  case pr2motion_RIGHT :
    open_position_result = right_gripper.setOpenPosition(open_position);
    open_max_effort_result = right_gripper.setOpenMaxEffort(open_max_effort);
    break;
  default:
    return pr2motion_unknown_error(self);
  }

#ifndef PR2_SIMU
  if((open_position_result != Gripper::OK)||(open_max_effort_result != Gripper::OK)){
    return pr2motion_invalid_param(self);
  }
#else
  if((open_position_result != GripperSimple::OK)||(open_max_effort_result != GripperSimple::OK)){
    return pr2motion_invalid_param(self);
  }
#endif
  return genom_ok;
}


/* --- Function Z_Gripper_Right_SetOpenParam ---------------------------- */

/** Codel setOpenParam of function Z_Gripper_Right_SetOpenParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_SetOpenParam */



/* --- Function Z_Gripper_Left_SetOpenParam ----------------------------- */

/** Codel setOpenParam of function Z_Gripper_Left_SetOpenParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_SetOpenParam */



/* --- Function Z_Gripper_GetOpenParam ---------------------------------- */

/** Codel getOpenParam of function Z_Gripper_GetOpenParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
getOpenParam(pr2motion_SIDE side, double *open_position,
             double *open_max_effort, genom_context self)
{
  if (side >= pr2motion_NB_SIDE)
    return pr2motion_invalid_param(self);   

  switch(side){
  case pr2motion_LEFT :
    *open_position = left_gripper.getOpenPosition();
    *open_max_effort = left_gripper.getOpenMaxEffort();
    break;
  case pr2motion_RIGHT :
    *open_position = right_gripper.getOpenPosition();
    *open_max_effort = right_gripper.getOpenMaxEffort();
    break;
  default:
    ROS_INFO("what happens, side = %d and %d %d\n",side, pr2motion_LEFT, pr2motion_RIGHT); 
    return pr2motion_unknown_error(self);
  }

  return genom_ok;
}


/* --- Function Z_Gripper_Right_GetOpenParam ---------------------------- */

/** Codel getOpenParam of function Z_Gripper_Right_GetOpenParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_GetOpenParam */



/* --- Function Z_Gripper_Left_GetOpenParam ----------------------------- */

/** Codel getOpenParam of function Z_Gripper_Left_GetOpenParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_GetOpenParam */



/* --- Function Z_Gripper_SetCloseParam --------------------------------- */

/** Codel setCloseParam of function Z_Gripper_SetCloseParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
setCloseParam(pr2motion_SIDE side, double close_position,
              double close_max_effort, genom_context self)
{
#ifndef PR2_SIMU
  Gripper::ERROR close_position_result=Gripper::OK;
  Gripper::ERROR close_max_effort_result=Gripper::OK;
#else
  GripperSimple::ERROR close_position_result=GripperSimple::OK;
  GripperSimple::ERROR close_max_effort_result=GripperSimple::OK;
#endif

  if (side >= pr2motion_NB_SIDE)
    return pr2motion_invalid_param(self);   

  switch(side){
  case pr2motion_LEFT :
    close_position_result = left_gripper.setClosePosition(close_position);
    close_max_effort_result = left_gripper.setCloseMaxEffort(close_max_effort);
    break;
  case pr2motion_RIGHT :
    close_position_result = right_gripper.setClosePosition(close_position);
    close_max_effort_result = right_gripper.setCloseMaxEffort(close_max_effort);
    break;
  default:
    return pr2motion_unknown_error(self);
  }

#ifndef PR2_SIMU
  if((close_position_result != Gripper::OK)||(close_max_effort_result != Gripper::OK)){
    return pr2motion_invalid_param(self);
  }
#else
  if((close_position_result != GripperSimple::OK)||(close_max_effort_result != GripperSimple::OK)){
    return pr2motion_invalid_param(self);
  }
#endif
  return genom_ok;
}


/* --- Function Z_Gripper_Right_SetCloseParam --------------------------- */

/** Codel setCloseParam of function Z_Gripper_Right_SetCloseParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_SetCloseParam */



/* --- Function Z_Gripper_Left_SetCloseParam ---------------------------- */

/** Codel setCloseParam of function Z_Gripper_Left_SetCloseParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_SetCloseParam */



/* --- Function Z_Gripper_GetCloseParam --------------------------------- */

/** Codel getCloseParam of function Z_Gripper_GetCloseParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
getCloseParam(pr2motion_SIDE side, double *close_position,
              double *close_max_effort, genom_context self)
{
  if (side >= pr2motion_NB_SIDE)
    return pr2motion_invalid_param(self);   

  switch(side){
  case pr2motion_LEFT :
    *close_position = left_gripper.getClosePosition();
    *close_max_effort = left_gripper.getCloseMaxEffort();
    break;
  case pr2motion_RIGHT :
    *close_position = right_gripper.getClosePosition();
    *close_max_effort = right_gripper.getCloseMaxEffort();
    break;
  default:
    return pr2motion_unknown_error(self);
  }

  return genom_ok;
}


/* --- Function Z_Gripper_Right_GetCloseParam --------------------------- */

/** Codel getCloseParam of function Z_Gripper_Right_GetCloseParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_GetCloseParam */



/* --- Function Z_Gripper_Left_GetCloseParam ---------------------------- */

/** Codel getCloseParam of function Z_Gripper_Left_GetCloseParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_GetCloseParam */



/* --- Function Z_Gripper_SetFindContactParam --------------------------- */

/** Codel setFindContactParam of function Z_Gripper_SetFindContactParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
setFindContactParam(pr2motion_SIDE side,
                    pr2motion_GRIPPER_CONTACT_CONDITIONS findtwo_contact_conditions,
                    bool findtwo_zero_fingertip_sensors,
                    genom_context self)
{
#ifndef PR2_SIMU
  Gripper::ERROR result_conditions=Gripper::OK;
  Gripper::ERROR result_fingertip=Gripper::OK;
  int8_t findtwo_contact_conditions_int=Gripper::OK;

  switch(findtwo_contact_conditions){
  case pr2motion_GRIPPER_CONTACT_BOTH :
    findtwo_contact_conditions_int = pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::BOTH;
    break;
  case pr2motion_GRIPPER_CONTACT_LEFT :
    findtwo_contact_conditions_int = pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::LEFT;
    break;
  case pr2motion_GRIPPER_CONTACT_RIGHT :
    findtwo_contact_conditions_int = pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::RIGHT;
    break;
  case pr2motion_GRIPPER_CONTACT_EITHER :
    findtwo_contact_conditions_int = pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::EITHER;
    break;
  default :
    return pr2motion_unknown_error(self);
  }

  switch(side){
  case pr2motion_LEFT :
    result_conditions = left_gripper.setFindTwoContactConditions(findtwo_contact_conditions_int);
    result_fingertip = left_gripper.setFindTwoZeroFingerTipSensors(findtwo_zero_fingertip_sensors);
    break;
  case pr2motion_RIGHT :
    result_conditions = right_gripper.setFindTwoContactConditions(findtwo_contact_conditions_int);
    result_fingertip = right_gripper.setFindTwoZeroFingerTipSensors(findtwo_zero_fingertip_sensors);
    break;    
  default:
    return pr2motion_unknown_error(self);
  }

  if((result_conditions != Gripper::OK) || (result_fingertip != Gripper::OK))
    return pr2motion_invalid_param(self);
#else
  ROS_INFO("pr2motion::setFindContactParam: you are in simulation mode, this function is not available\n");
#endif
  return genom_ok;
}


/* --- Function Z_Gripper_Right_SetFindContactParam --------------------- */

/** Codel setFindContactParam of function Z_Gripper_Right_SetFindContactParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_SetFindContactParam */



/* --- Function Z_Gripper_Left_SetFindContactParam ---------------------- */

/** Codel setFindContactParam of function Z_Gripper_Left_SetFindContactParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_SetFindContactParam */



/* --- Function Z_Gripper_GetFindContactParam --------------------------- */

/** Codel getFindContactParam of function Z_Gripper_GetFindContactParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
getFindContactParam(pr2motion_SIDE side,
                    pr2motion_GRIPPER_CONTACT_CONDITIONS *findtwo_contact_conditions,
                    bool *findtwo_zero_fingertip_sensors,
                    genom_context self)
{
#ifndef PR2_SIMU
  int8_t findtwo_contact_conditions_int;
  switch(side){
  case pr2motion_LEFT :
    findtwo_contact_conditions_int = left_gripper.getFindTwoContactConditions();
    *findtwo_zero_fingertip_sensors = left_gripper.getFindTwoZeroFingerTipSensors();
    break;
  case pr2motion_RIGHT :
    findtwo_contact_conditions_int = right_gripper.getFindTwoContactConditions();
    *findtwo_zero_fingertip_sensors = right_gripper.getFindTwoZeroFingerTipSensors();
    break;    
  default:
    return pr2motion_unknown_error(self);
  }  

  switch(findtwo_contact_conditions_int){
  case pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::BOTH :
    *findtwo_contact_conditions = pr2motion_GRIPPER_CONTACT_BOTH;
    break;
  case pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::LEFT :
    *findtwo_contact_conditions = pr2motion_GRIPPER_CONTACT_LEFT;
    break;
  case pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::RIGHT :
    *findtwo_contact_conditions = pr2motion_GRIPPER_CONTACT_RIGHT;
    break;
  case pr2_gripper_sensor_msgs::PR2GripperFindContactCommand::EITHER :
    *findtwo_contact_conditions = pr2motion_GRIPPER_CONTACT_EITHER;
    break;
  default :
    return pr2motion_unknown_error(self);
  }
#else
  ROS_INFO("pr2motion::getFindContactParam: you are in simulation mode, this function is not available\n");
#endif  
  return genom_ok;
}


/* --- Function Z_Gripper_Right_GetFindContactParam --------------------- */

/** Codel getFindContactParam of function Z_Gripper_Right_GetFindContactParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_GetFindContactParam */



/* --- Function Z_Gripper_Left_GetFindContactParam ---------------------- */

/** Codel getFindContactParam of function Z_Gripper_Left_GetFindContactParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_GetFindContactParam */



/* --- Function Z_Gripper_SetEventDetectorParam ------------------------- */

/** Codel setEventDetectorParam of function Z_Gripper_SetEventDetectorParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
setEventDetectorParam(pr2motion_SIDE side,
                      pr2motion_GRIPPER_EVENT_DETECTOR event_trigger_conditions,
                      double event_acceleration_trigger_magnitude,
                      double event_slip_trigger_magnitude,
                      genom_context self)
{
#ifndef PR2_SIMU
  Gripper::ERROR result_conditions=Gripper::OK;
  Gripper::ERROR result_acceleration=Gripper::OK;
  Gripper::ERROR result_slip=Gripper::OK;
  int8_t event_trigger_conditions_int;
  
  switch(event_trigger_conditions_int){
  case pr2motion_GRIPPER_FINGER_SIDE_IMPACT_OR_ACC :
    event_trigger_conditions_int =pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::FINGER_SIDE_IMPACT_OR_ACC;
    break;
  case pr2motion_GRIPPER_FINGER_SLIP_AND_ACC :
    event_trigger_conditions_int = pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::SLIP_AND_ACC;
    break;
  case pr2motion_GRIPPER_FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC :
    event_trigger_conditions_int = pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;
    break;
  case pr2motion_GRIPPER_SLIP :
    event_trigger_conditions_int = pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::SLIP;
    break;
  case pr2motion_GRIPPER_ACC :
    event_trigger_conditions_int = pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::ACC;
    break;
  default :
    return pr2motion_unknown_error(self);
  }

  switch(side){
  case pr2motion_LEFT :
    result_conditions = left_gripper.setPlaceTriggerConditions(event_trigger_conditions_int);
    result_acceleration = left_gripper.setPlaceAccelerationTriggerMagnitude(event_acceleration_trigger_magnitude);
    result_slip = left_gripper.setPlaceSlipTriggerMagnitude(event_slip_trigger_magnitude);
    break;
  case pr2motion_RIGHT :
    result_conditions = right_gripper.setPlaceTriggerConditions(event_trigger_conditions_int);
    result_acceleration = right_gripper.setPlaceAccelerationTriggerMagnitude(event_acceleration_trigger_magnitude);
    result_slip = right_gripper.setPlaceSlipTriggerMagnitude(event_slip_trigger_magnitude);    
    break;    
  default:
    return pr2motion_unknown_error(self);
  }

  if((result_conditions != Gripper::OK) || (result_acceleration != Gripper::OK) || (result_slip != Gripper::OK))
    return pr2motion_invalid_param(self);
#else
  ROS_INFO("pr2motion::setEventDetectorParam: you are in simulation mode, this function is not available\n");
#endif 
  return genom_ok;
}


/* --- Function Z_Gripper_Right_SetEventDetectorParam ------------------- */

/** Codel setEventDetectorParam of function Z_Gripper_Right_SetEventDetectorParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_SetEventDetectorParam */



/* --- Function Z_Gripper_Left_SetEventDetectorParam -------------------- */

/** Codel setEventDetectorParam of function Z_Gripper_Left_SetEventDetectorParam.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_SetEventDetectorParam */



/* --- Function Z_Gripper_GetEventDetectorGoal -------------------------- */

/** Codel getEventDetectorParam of function Z_Gripper_GetEventDetectorGoal.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
getEventDetectorParam(pr2motion_SIDE side,
                      pr2motion_GRIPPER_EVENT_DETECTOR *event_trigger_conditions,
                      double *event_acceleration_trigger_magnitude,
                      double *event_slip_trigger_magnitude,
                      genom_context self)
{
#ifndef PR2_SIMU
  int8_t event_trigger_conditions_int;

  switch(side){
  case pr2motion_LEFT :
    event_trigger_conditions_int = left_gripper.getPlaceTriggerConditions();
    *event_acceleration_trigger_magnitude = left_gripper.getPlaceAccelerationTriggerMagnitude();
    *event_slip_trigger_magnitude = left_gripper.getPlaceSlipTriggerMagnitude();
    break;
  case pr2motion_RIGHT :
    event_trigger_conditions_int = right_gripper.getPlaceTriggerConditions();
    *event_acceleration_trigger_magnitude = right_gripper.getPlaceAccelerationTriggerMagnitude();
    *event_slip_trigger_magnitude = right_gripper.getPlaceSlipTriggerMagnitude();
    break;    
  default:
    return pr2motion_unknown_error(self);
  }

  switch(event_trigger_conditions_int){
  case pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::FINGER_SIDE_IMPACT_OR_ACC :
    *event_trigger_conditions = pr2motion_GRIPPER_FINGER_SIDE_IMPACT_OR_ACC;
    break;
  case pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::SLIP_AND_ACC :
    *event_trigger_conditions = pr2motion_GRIPPER_FINGER_SLIP_AND_ACC;
    break;
  case pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC :
    *event_trigger_conditions = pr2motion_GRIPPER_FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;
    break;
  case pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::SLIP :
    *event_trigger_conditions = pr2motion_GRIPPER_SLIP;
    break;
  case pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::ACC :
    *event_trigger_conditions = pr2motion_GRIPPER_ACC;
    break;
  default :
    return pr2motion_unknown_error(self);
  }

#else
  ROS_INFO("pr2motion::setEventDetectorParam: you are in simulation mode, this function is not available\n");
#endif 
  return genom_ok;
}


/* --- Function Z_Gripper_Right_GetEventDetectorGoal -------------------- */

/** Codel getEventDetectorParam of function Z_Gripper_Right_GetEventDetectorGoal.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_GetEventDetectorGoal */



/* --- Function Z_Gripper_Left_GetEventDetectorGoal --------------------- */

/** Codel getEventDetectorParam of function Z_Gripper_Left_GetEventDetectorGoal.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_GetEventDetectorGoal */



/* --- Function Z_Gripper_SetForceServoGoal ----------------------------- */

/** Codel setForceServoGoal of function Z_Gripper_SetForceServoGoal.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
setForceServoGoal(pr2motion_SIDE side, double fingertip_force,
                  genom_context self)
{
#ifndef PR2_SIMU
  Gripper::ERROR fingertip_force_result=Gripper::OK;

  if (side >= pr2motion_NB_SIDE)
    return pr2motion_invalid_param(self);   

  switch(side){
  case pr2motion_LEFT :
    fingertip_force_result = left_gripper.setForceServoFingertipForce(fingertip_force);
    break;
  case pr2motion_RIGHT :
    fingertip_force_result = right_gripper.setForceServoFingertipForce(fingertip_force);
    break;
  default:
    return pr2motion_unknown_error(self);
  }

  if(fingertip_force_result != Gripper::OK) {
    return pr2motion_invalid_param(self);
  }

#else
  return genom_ok;
#endif
}


/* --- Function Z_Gripper_Right_SetForceServoGoal ----------------------- */

/** Codel setForceServoGoal of function Z_Gripper_Right_SetForceServoGoal.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_SetForceServoGoal */



/* --- Function Z_Gripper_Left_SetForceServoGoal ------------------------ */

/** Codel setForceServoGoal of function Z_Gripper_Left_SetForceServoGoal.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_SetForceServoGoal */



/* --- Function Z_Gripper_GetForceServoGoal ----------------------------- */

/** Codel getForceServoGoal of function Z_Gripper_GetForceServoGoal.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
getForceServoGoal(pr2motion_SIDE side, double *fingertip_force,
                  genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Function Z_Gripper_Right_GetForceServoGoal ----------------------- */

/** Codel getForceServoGoal of function Z_Gripper_Right_GetForceServoGoal.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_GetForceServoGoal */



/* --- Function Z_Gripper_Left_GetForceServoGoal ------------------------ */

/** Codel getForceServoGoal of function Z_Gripper_Left_GetForceServoGoal.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Gripper_GetForceServoGoal */



/* --- Function Z_Torso_SetMaxVelocity ---------------------------------- */

/** Codel setTorsoMaxVelocity of function Z_Torso_SetMaxVelocity.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param.
 */
genom_event
setTorsoMaxVelocity(double torso_max_velocity, genom_context self)
{
  Torso::ERROR result= Torso::OK;
  result = torso.setTorsoMaxVelocity(torso_max_velocity);
  if(result != Torso::OK) {
    return pr2motion_invalid_param(self);
  } 
  return genom_ok;
}


/* --- Function Z_Head_SetMaxVelocity ----------------------------------- */

/** Codel setHeadMaxVelocity of function Z_Head_SetMaxVelocity.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param.
 */
genom_event
setHeadMaxVelocity(double head_max_velocity, genom_context self)
{
  RobotHead::ERROR result= RobotHead::OK;
  result = head.setMaxVelocity(head_max_velocity);
  if(result != RobotHead::OK) {
    return pr2motion_invalid_param(self);
  } 
  return genom_ok;
}


/* --- Function Z_Head_GetMaxVelocity ----------------------------------- */

/** Codel getHeadMaxVelocity of function Z_Head_GetMaxVelocity.
 *
 * Returns genom_ok.
 */
genom_event
getHeadMaxVelocity(double *head_max_velocity, genom_context self)
{
  *head_max_velocity = head.getMaxVelocity();
  return genom_ok;
}


/* --- Function Z_Head_SetMinDuration ----------------------------------- */

/** Codel setHeadMinDuration of function Z_Head_SetMinDuration.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param.
 */
genom_event
setHeadMinDuration(double head_min_duration, genom_context self)
{
  RobotHead::ERROR result= RobotHead::OK;
  result = head.setMinDuration(head_min_duration);
  if(result != RobotHead::OK) {
    return pr2motion_invalid_param(self);
  } 
  return genom_ok;
}


/* --- Function Z_Head_GetMinDuration ----------------------------------- */

/** Codel getHeadMinDuration of function Z_Head_GetMinDuration.
 *
 * Returns genom_ok.
 */
genom_event
getHeadMinDuration(double *head_min_duration, genom_context self)
{
  *head_min_duration = head.getMinDuration();
  return genom_ok;
}


/* --- Function Z_Arm_SetT ---------------------------------------------- */

/** Codel Arm_SetT of function Z_Arm_SetT.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
genom_event
Arm_SetT(pr2motion_SIDE side, double time_slot, genom_context self)
{
  RobotArm::ERROR result=RobotArm::OK;
  
  switch(side){
  case pr2motion_LEFT :
    result = left_arm.setT(time_slot);
    break;
  case pr2motion_RIGHT :
    result = right_arm.setT(time_slot);
    break;
  default:
    return pr2motion_unknown_error(self);
  }

  if(result!=RobotArm::OK)
    return pr2motion_invalid_param(self);

  return genom_ok;
}


/* --- Function Z_Arm_Right_SetT ---------------------------------------- */

/** Codel Arm_SetT of function Z_Arm_Right_SetT.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Arm_SetT */



/* --- Function Z_Arm_Left_SetT ----------------------------------------- */

/** Codel Arm_SetT of function Z_Arm_Left_SetT.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param, pr2motion_unknown_error.
 */
/* already defined in service Z_Arm_SetT */

