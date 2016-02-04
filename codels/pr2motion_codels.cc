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

#include "pr2_torso_client.hh"
#include "pr2_head_client.hh"

extern RobotHead head;
extern Torso torso;

/* --- Attribute Gripper_SetOpenGoal ------------------------------------ */

/** Validation codel Gripper_SetOpenGoal of attribute Gripper_SetOpenGoal.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
Gripper_SetOpenGoal(double open_position, double open_max_effort,
                    genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Attribute Gripper_SetCloseGoal ----------------------------------- */

/** Validation codel Gripper_SetCloseGoal of attribute Gripper_SetCloseGoal.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
Gripper_SetCloseGoal(double close_position, double close_max_effort,
                     genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Attribute Gripper_Right_SetOpenGoal ------------------------------ */

/** Validation codel Gripper_SetOpenGoal of attribute Gripper_Right_SetOpenGoal.
 *
 * Returns genom_ok.
 * Throws .
 */
/* already defined in service Gripper_SetOpenGoal validation */



/* --- Attribute Gripper_Right_SetCloseGoal ----------------------------- */

/** Validation codel Gripper_SetCloseGoal of attribute Gripper_Right_SetCloseGoal.
 *
 * Returns genom_ok.
 * Throws .
 */
/* already defined in service Gripper_SetCloseGoal validation */



/* --- Attribute Gripper_Left_SetOpenGoal ------------------------------- */

/** Validation codel Gripper_SetOpenGoal of attribute Gripper_Left_SetOpenGoal.
 *
 * Returns genom_ok.
 * Throws .
 */
/* already defined in service Gripper_SetOpenGoal validation */



/* --- Attribute Gripper_Left_SetCloseGoal ------------------------------ */

/** Validation codel Gripper_SetCloseGoal of attribute Gripper_Left_SetCloseGoal.
 *
 * Returns genom_ok.
 * Throws .
 */
/* already defined in service Gripper_SetCloseGoal validation */



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
