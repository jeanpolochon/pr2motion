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


/* --- Attribute Torso_SetMoveParam ------------------------------------- */

/** Validation codel Torso_SetMoveParam of attribute Torso_SetMoveParam.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
Torso_SetMoveParam(double torso_min_duration, float torso_max_velocity,
                   genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Attribute Arm_SetMax --------------------------------------------- */

/** Validation codel Arm_SetMax of attribute Arm_SetMax.
 *
 * Returns genom_ok.
 * Throws pr2motion_invalid_param.
 */
genom_event
Arm_SetMax(double max_vel, double max_acc, double max_jerk,
           genom_context self)
{
    // switch(side){
    // case pr2motion_PR2MOTION_LEFT :
    //   right_arm.setMax(max_vel, max_acc, max_jerk);
    // case pr2motion_PR2MOTION_RIGHT:
    //   left_arm.setMax(max_vel, max_acc, max_jerk);
    // default:
    //   right_arm.setMax(max_vel, max_acc, max_jerk);
    //   left_arm.setMax(max_vel, max_acc, max_jerk);
    // }
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


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



