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

/* --- Attribute Arm_SetT ----------------------------------------------- */

/** Validation codel Arm_SetT of attribute Arm_SetT.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param.
 */
genom_event
Arm_SetT(double time_slot, genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Attribute Arm_Right_SetT ----------------------------------------- */

/** Validation codel Arm_SetT of attribute Arm_Right_SetT.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param.
 */
/* already defined in service Arm_SetT validation */



/* --- Attribute Arm_Left_SetT ------------------------------------------ */

/** Validation codel Arm_SetT of attribute Arm_Left_SetT.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param.
 */
/* already defined in service Arm_SetT validation */



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
  Gripper::ERROR open_position_result;
  Gripper::ERROR open_max_effort_result;
#else
  GripperSimple::ERROR open_position_result;
  GripperSimple::ERROR open_max_effort_result;
#endif

  if (side >= pr2motion_NB_SIDE)
    return pr2motion_invalid_param(self);   

  switch(side){
  case pr2motion_LEFT :
    open_position_result = left_gripper.setOpenPosition(open_position);
    open_max_effort_result = left_gripper.setOpenMaxEffort(open_max_effort);
  case pr2motion_RIGHT :
    open_position_result = right_gripper.setOpenPosition(open_position);
    open_max_effort_result = right_gripper.setOpenMaxEffort(open_max_effort);
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
  case pr2motion_RIGHT :
    *open_position = right_gripper.getOpenPosition();
    *open_max_effort = right_gripper.getOpenMaxEffort();
  default:
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
  Gripper::ERROR close_position_result;
  Gripper::ERROR close_max_effort_result;
#else
  GripperSimple::ERROR close_position_result;
  GripperSimple::ERROR close_max_effort_result;
#endif

  if (side >= pr2motion_NB_SIDE)
    return pr2motion_invalid_param(self);   

  switch(side){
  case pr2motion_LEFT :
    close_position_result = left_gripper.setClosePosition(close_position);
    close_max_effort_result = left_gripper.setCloseMaxEffort(close_max_effort);
  case pr2motion_RIGHT :
    close_position_result = right_gripper.setClosePosition(close_position);
    close_max_effort_result = right_gripper.setCloseMaxEffort(close_max_effort);
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
  case pr2motion_RIGHT :
    *close_position = right_gripper.getClosePosition();
    *close_max_effort = right_gripper.getCloseMaxEffort();
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
