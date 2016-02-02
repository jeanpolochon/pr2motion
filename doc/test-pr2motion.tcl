# this file intend to explain how to use pr2motion module


#
#
# LOAD
#
#
proc pr2motionload { } {
    package require genomix  
    genomix::connect 
    genomix1 load pr2motion
}

#
#
# INIT
#
#
proc pr2motioninit { } {
    # initialize the module
    # ie connect to the pr2_controllers
    pr2motion::Init {}
    # connect to joint_states topic (that will be copied into joint_state)
    pr2motion::connect_port joint_state joint_states
    # connect to /head_traj_controller_state (that will be copied into head_controller_state)
    pr2motion::connect_port  head_controller_state /head_traj_controller/state
}


#
#
# GRIPPER RELATED FUNCTIONS
#
#
#ex ::pr2motion::Gripper_Right_Operate {goal_mode ::pr2motion::GRIPPER_OPEN}
# available mode are: 
#                     ::pr2motion::GRIPPER_OPEN 
#                     ::pr2motion::GRIPPER_CLOSE 
#                     ::pr2motion::GRIPPER_GRAB 
#                     ::pr2motion::GRIPPER_RELEASE
proc pr2motiongripperright { p1 } {
    pr2motion::Gripper_Right_Operate [subst {goal_mode $p1}]
}
proc pr2motiongripperrightstop { } {
    pr2motion::Gripper_Right_Stop
}

proc pr2motiongripperleft { p1 } {
    pr2motion::Gripper_Left_Operate [subst {goal_mode $p1}]
}

proc pr2motiongripperleftstop { } {
    pr2motion::Gripper_Left_Stop
}


#
#
# HEAD RELATED FUNCTIONS
#
#

# ex: ::pr2motion::Head_Move {head_mode ::pr2motion::HEAD_LOOKAT head_target_frame base_link head_target_x 3.0 head_target_y 0 head_target_z 0}
proc pr2motionlookat { p2 p3 p4 p5 } {
    pr2motion::Head_Move [subst {head_mode ::pr2motion::HEAD_LOOKAT head_target_frame $p2 head_target_x $p3 head_target_y $p4 head_target_z $p5}]
}

# ex ::pr2motion::Head_Move -ack {head_mode ::pr2motion::HEAD_FOLLOWING head_target_frame {r_gripper_tool_frame } head_target_x 0 head_target_y 0 head_target_z 0}
proc pr2motionheadfollow { p2 p3 p4 p5 } {
    pr2motion::Head_Move -ack [subst {head_mode ::pr2motion::HEAD_FOLLOWING head_target_frame $p2 head_target_x $p3 head_target_y $p4 head_target_z $p5}]
}

proc pr2motionheadstop { } {
    ::pr2motion::Head_Stop
}

#
#
# TORSO RELATED FUNCTIONS
#
#

#ex ::pr2motion::Torso_Move {torso_position 0.1}
proc pr2motiontorsomoveto { p1 } {
    pr2motion::Torso_Move [subst {torso_position 0.1}]
}

proc pr2motiontorsostop { } {
    ::pr2motion::Torso_Stop
}

#
#
# ARM RELATED FUNCTIONS
#
#

proc pr2motionrightarmstop { } {
     ::pr2motion::Arm_Right_Stop
}

proc pr2motionleftarmstop { } {
     ::pr2motion::Arm_Right_Stop
}


#
# without GTP
#
# ex: ::pr2motion::Arm_Right_MoveToQGoal {traj_mode ::pr2motion::TRAJ_SOFTMOTION shoulder_pan_joint 0 shoulder_lift_joint 0 upper_arm_roll_joint 0 elbow_flex_joint -0.5 forearm_roll_joint 0 wrist_flex_joint -0.2 wrist_roll_joint 0}
# available traj_mode are :
#        ::pr2motion::TRAJ_GATECH (compute a trajectory using Georgia Tech alg)
#        ::pr2motion::TRAJ_SOFTMOTION (compute a trajectory using softmotion alg)
#        ::pr2motion::TRAJ_PATH (do not compute a trajectory, use the path)
proc pr2motionrightarmtoQ { p1 p2 p3  p4 p5 p6 p7 p8} {
    ::pr2motion::Arm_Right_MoveToQGoal [subst {traj_mode $p1 shoulder_pan_joint $p2 shoulder_lift_joint $p3 upper_arm_roll_joint $p4 elbow_flex_joint $p5 forearm_roll_joint $p6 wrist_flex_joint $p7 wrist_roll_joint $p8}]
}
proc pr2motionleftarmtoQ { p1 p2 p3  p4 p5 p6 p7 p8} {
    ::pr2motion::Arm_Left_MoveToQGoal [subst {traj_mode $p1 shoulder_pan_joint $p2 shoulder_lift_joint $p3 upper_arm_roll_joint $p4 elbow_flex_joint $p5 forearm_roll_joint $p6 wrist_flex_joint $p7 wrist_roll_joint $p8}]
}

#
# with GTP
#
# you need first to connect the two topics (ports)
proc pr2motiongtpinit { } {
    # connect to gtp_trajectory (that will be copied into traj)
    pr2motion::connect_port traj gtp_trajectory    
}
# then if a trajectory is exported in the topic by gtp,
# you can use
#ex : ::pr2motion::Arm_Right_Move {traj_mode ::pr2motion::TRAJ_GATECH path_mode ::pr2motion::PATH_PORT}
# available traj_mode are :
#        ::pr2motion::TRAJ_GATECH (compute a trajectory using Georgia Tech alg)
#        ::pr2motion::TRAJ_SOFTMOTION (compute a trajectory using softmotion alg)
#        ::pr2motion::TRAJ_PATH (do not compute a trajectory, use the path)
# available path_mode are :
#        ::pr2motion::PATH_PORT (read the trajectory on the traj port)
#        ::pr2motion::PATH_TEST (use a predefined test path - only for debug purpose)
proc pr2motionrightarm { p1 p2 p3 } {
    ::pr2motion::Arm_Right_Move [subst {traj_mode $p1 path_mode ::pr2motion::PATH_PORT}]
}
proc pr2motionleftarm { p1 p2 p3 } {
    ::pr2motion::Arm_Left_Move [subst {traj_mode $p1 path_mode ::pr2motion::PATH_PORT}]
}


#
#
# TEST
#
#
# this function is intended to test available pr2 functions
proc pr2motiontestinit { } {
    pr2motionload
    pr2motioninit
}
proc pr2motiontestgripper { } {
    # test of the grippers
    pr2motion::Gripper_Right_Operate {goal_mode ::pr2motion::GRIPPER_OPEN}
    pr2motion::Gripper_Right_Operate {goal_mode ::pr2motion::GRIPPER_CLOSE}
    pr2motion::Gripper_Right_Operate -ack {goal_mode ::pr2motion::GRIPPER_OPEN}
    pr2motion::Gripper_Right_Stop
    pr2motion::Gripper_Left_Operate {goal_mode ::pr2motion::GRIPPER_OPEN}
    pr2motion::Gripper_Left_Operate {goal_mode ::pr2motion::GRIPPER_CLOSE}
    pr2motion::Gripper_Left_Operate -ack {goal_mode ::pr2motion::GRIPPER_OPEN}
    pr2motion::Gripper_Left_Stop
}
proc pr2motiontesttorso { } {
    # test of the torso
    pr2motion::Torso_Move {torso_position 0.1}
    pr2motion::Torso_Move -ack {torso_position 0.25}
    pr2motion::Torso_Stop
    # these last two should send back invalid_param
    pr2motion::Torso_Move {torso_position 0.0}   
    pr2motion::Torso_Move {torso_position 0.32}      
}
proc pr2motiontesthead { } {
    # test the head
    ::pr2motion::Head_Move {head_mode ::pr2motion::HEAD_LOOKAT head_target_frame base_link head_target_x 3.0 head_target_y 0 head_target_z 0}
    ::pr2motion::Head_Move -ack {head_mode ::pr2motion::HEAD_LOOKAT head_target_frame base_link head_target_x 3.0 head_target_y 0 head_target_z 0}
    ::pr2motion::Head_Stop
    ::pr2motion::Head_Move {head_mode ::pr2motion::HEAD_LOOKAT head_target_frame r_gripper_tool_frame head_target_x 0.0 head_target_y 0 head_target_z 0}  
    ::pr2motion::Head_Move -ack {head_mode ::pr2motion::HEAD_FOLLOWING head_target_frame r_gripper_tool_frame head_target_x 0.0 head_target_y 0 head_target_z 0}
    ::pr2motion::Head_Stop
    ::pr2motion::Head_Move {head_mode ::pr2motion::HEAD_LOOKAT head_target_frame l_gripper_tool_frame head_target_x 0.0 head_target_y 0 head_target_z 0}  
    ::pr2motion::Head_Move -ack {head_mode ::pr2motion::HEAD_FOLLOWING head_target_frame l_gripper_tool_frame head_target_x 0.0 head_target_y 0 head_target_z 0}
    ::pr2motion::Head_Stop
}

proc pr2motiontestrightarm { } {
    # test the right arm
    ::pr2motion::Arm_Right_MoveToQGoal {traj_mode ::pr2motion::TRAJ_SOFTMOTION shoulder_pan_joint -2.0 shoulder_lift_joint 0 upper_arm_roll_joint 0 elbow_flex_joint -0.5 forearm_roll_joint 0 wrist_flex_joint -0.2 wrist_roll_joint 0}
    ::pr2motion::Arm_Right_Stop
    ::pr2motion::Arm_Right_MoveToQGoal {traj_mode ::pr2motion::TRAJ_GATECH shoulder_pan_joint 0 shoulder_lift_joint 0 upper_arm_roll_joint 0 elbow_flex_joint -0.5 forearm_roll_joint 0 wrist_flex_joint -0.2 wrist_roll_joint 0}
    ::pr2motion::Arm_Right_Stop
    ::pr2motion::Arm_Right_MoveToQGoal -ack {traj_mode ::pr2motion::TRAJ_SOFTMOTION shoulder_pan_joint -2.0 shoulder_lift_joint 0 upper_arm_roll_joint 0 elbow_flex_joint -0.5 forearm_roll_joint 0 wrist_flex_joint -0.2 wrist_roll_joint 0}   
    ::pr2motion::Arm_Right_Stop
}

#if there is a trajectory available on the topic
proc pr2motiontestrightarmwithgtp { } {
    ::pr2motion::Arm_Right_Move {traj_mode ::pr2motion::TRAJ_SOFTMOTION path_mode ::pr2motion::PATH_PORT}
}

#if there is a trajectory available on the topic
proc pr2motiontestleftarmwithgtp { } {
    ::pr2motion::Arm_Left_Move {traj_mode ::pr2motion::TRAJ_SOFTMOTION path_mode ::pr2motion::PATH_PORT}
}



proc pr2motiontestleftarm { } {
    # test the left arm
    ::pr2motion::Arm_Left_MoveToQGoal {traj_mode ::pr2motion::TRAJ_SOFTMOTION shoulder_pan_joint 2.0 shoulder_lift_joint 0 upper_arm_roll_joint 0 elbow_flex_joint -0.5 forearm_roll_joint 0 wrist_flex_joint -0.2 wrist_roll_joint 0}
    ::pr2motion::Arm_Left_Stop
    ::pr2motion::Arm_Left_MoveToQGoal {traj_mode ::pr2motion::TRAJ_GATECH shoulder_pan_joint 0 shoulder_lift_joint 0 upper_arm_roll_joint 0 elbow_flex_joint -0.5 forearm_roll_joint 0 wrist_flex_joint -0.2 wrist_roll_joint 0}
    ::pr2motion::Arm_Left_Stop
    ::pr2motion::Arm_Left_MoveToQGoal -ack {traj_mode ::pr2motion::TRAJ_SOFTMOTION shoulder_pan_joint 2.0 shoulder_lift_joint 0 upper_arm_roll_joint 0 elbow_flex_joint -0.5 forearm_roll_joint 0 wrist_flex_joint -0.2 wrist_roll_joint 0}   
    ::pr2motion::Arm_Left_Stop
}

proc pr2motiontestall { } {
    pr2motiontestinit
    pr2motiontestgripper
    pr2motiontesttorso
    pr2motiontesthead
}
#
#
# MISCELANEOUS
#
#
#dans des quotes les variables ne sont pas subsituées, il faut donc faire au choix:
#pr2motion::Head_Move [subst {head_mode ::pr2motion::HEAD_LOOKAT
#head_target_frame base_link head_target_x $p3 head_target_y $p4
#head_target_z $p5}]

#ou alors
#set arg [dict create]
#dict set arg head_target_x $p3
#dict set arg head_target_y $p4
#pr2motion::Head_Move $arg
