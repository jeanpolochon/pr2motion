#ifndef _PR2_GRIPPERSENSOR_CLIENT_H
#define _PR2_GRIPPERSENSOR_CLIENT_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandFeedback.h>
#include <pr2_controllers_msgs/Pr2GripperCommandResult.h>

#include <pr2_gripper_sensor_msgs/PR2GripperFindContactAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactCommand.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperGrabAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperReleaseAction.h>

#include <actionlib/client/simple_action_client.h>

#include <pr2_model.hh>

typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperSlipServoAction> SlipClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperFindContactAction> ContactClient;
// Our Action interface type, provided as a typedef for convenience                   
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction> EventDetectorClient;
// Our Action interface type, provided as a typedef for convenience                   
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
// Our Action interface type, provided as a typedef for convenience                   
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperGrabAction> GrabClient;
// Our Action interface type, provided as a typedef for convenience                   
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperReleaseAction> ReleaseClient;
// Our Action interface type, provided as a typedef for convenience                   


class Gripper{
private:
  GripperClient* gripper_client_;  
  ContactClient* contact_client_;
  SlipClient* slip_client_;
  EventDetectorClient* event_detector_client_;
  GrabClient* grab_client_;
  ReleaseClient* release_client_;

  // open cmd data
  double open_position_;
  double open_position_max_;
  double open_position_min_;
  double open_max_effort_;
  double open_max_effort_max_;
  // close cmd data
  double close_position_;
  double close_position_max_;
  double close_position_min_;
  double close_max_effort_;
  double close_max_effort_max_; 
  //pr2_gripper_sensor_msgs::PR2GripperFindContactCommand 
  //rosmsg show pr2_gripper_sensor_msgs/PR2GripperFindContactCommand 
  // int8 BOTH=0
  // int8 LEFT=1
  // int8 RIGHT=2
  // int8 EITHER=3
  // bool zero_fingertip_sensors
  // int8 contact_conditions
  int8_t findtwo_contact_conditions_;
  bool findtwo_zero_finger_tip_sensors_;
  //pr2_gripper_sensor_msgs/PR2GripperEventDetectorCommand command
  // int8 FINGER_SIDE_IMPACT_OR_ACC=0
  // int8 SLIP_AND_ACC=1
  // int8 FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC=2
  // int8 SLIP=3
  // int8 ACC=4
  // int8 trigger_conditions
  // float64 acceleration_trigger_magnitude
  // float64 slip_trigger_magnitude  
  int8_t place_trigger_conditions_;
  double place_acceleration_trigger_magnitude_;
  double place_slip_trigger_magnitude_;

  /* pr2_gripper_sensor_msgs::PR2GripperFindContactData */
  /* time stamp */
  /* bool contact_conditions_met */
  /* bool left_fingertip_pad_contact */
  /* bool right_fingertip_pad_contact */
  /* float64 left_fingertip_pad_force */
  /* float64 right_fingertip_pad_force */
  /* float64 joint_position */
  /* float64 joint_effort */
  /* pr2_gripper_sensor_msgs/PR2GripperSensorRTState rtstate */
  /*   int8 DISABLED=0 */
  /*   int8 POSITION_SERVO=3 */
  /*   int8 FORCE_SERVO=4 */
  /*   int8 FIND_CONTACT=5 */
  /*   int8 SLIP_SERVO=6 */
  /*   int8 realtime_controller_state */
  pr2_gripper_sensor_msgs::PR2GripperFindContactData findTwo_feedback;
  pr2_gripper_sensor_msgs::PR2GripperFindContactData findTwo_result;

  /* pr2_gripper_sensor_msgs/PR2GripperSlipServoData     */
  /*   time stamp */
  /*   float64 deformation */
  /*   float64 left_fingertip_pad_force */
  /*   float64 right_fingertip_pad_force */
  /*   float64 joint_effort */
  /*   bool slip_detected */
  /*   bool deformation_limit_reached */
  /*   bool fingertip_force_limit_reached */
  /*   bool gripper_empty */
  /*   pr2_gripper_sensor_msgs/PR2GripperSensorRTState rtstate */
  /*      int8 DISABLED=0 */
  /*      int8 POSITION_SERVO=3 */
  /*      int8 FORCE_SERVO=4 */
  /*      int8 FIND_CONTACT=5 */
  /*      int8 SLIP_SERVO=6 */
  /*      int8 realtime_controller_state */
  pr2_gripper_sensor_msgs::PR2GripperSlipServoData slipServo_feedback;
  pr2_gripper_sensor_msgs::PR2GripperSlipServoData slipServo_result;

  /* pr2_gripper_sensor_msgs/PR2GripperEventDetectorData  */
  /*    time stamp */
  /*    bool trigger_conditions_met */
  /*    bool slip_event */
  /*    bool acceleration_event */
  /*    float64[3] acceleration_vector */
  pr2_gripper_sensor_msgs::PR2GripperEventDetectorData place_feedback;
  pr2_gripper_sensor_msgs::PR2GripperEventDetectorData place_result;

  /*  PR2GripperGrabCommand command */
  /*    float64 hardness_gain  */
  double grab_hardness_gain_;
  
public:
  enum SIDE { LEFT, RIGHT, NB_SIDE };
  enum ERROR { OK, INIT_FAILED, INIT_NOT_DONE, CANNOT_READ_LIMITS, UNKNOWN_JOINT, SERVER_NOT_CONNECTED, INVALID_PARAM, NB_ERROR }; 

  //Action client initialization
  Gripper();
  ~Gripper();

  ERROR init(SIDE);

  ERROR isConnected();

  // Get client state
  //Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST. 
  //The goal's state. Returns LOST if this SimpleActionClient isn't tracking a goal. 
  
  // CLOSE
  ERROR setClosePosition(double);
  double getClosePosition();
  ERROR setCloseMaxEffort(double);
  double getCloseMaxEffort();
  bool close_isDone();
  actionlib::SimpleClientGoalState close_getState();
  void close_doneCb(const actionlib::SimpleClientGoalState&, const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr&);
  void close_activeCb();
  void close_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr&);
  void close();
  void close_cancel();

  // OPEN
  ERROR setOpenPosition(double);
  double getOpenPosition();
  ERROR setOpenMaxEffort(double);
  double getOpenMaxEffort();
  bool open_isDone();
  actionlib::SimpleClientGoalState open_getState();
  void open_doneCb(const actionlib::SimpleClientGoalState&, const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr&);
  void open_activeCb();
  void open_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr&);
  void open();
  void open_cancel();

  // FINDTWO
  ERROR setFindTwoContactConditions(int8_t);
  int8_t getFindTwoContactConditions();
  ERROR setFindTwoZeroFingerTipSensors(bool);
  bool getFindTwoZeroFingerTipSensors();
  bool findTwo_isDone();
  actionlib::SimpleClientGoalState findTwo_getState();  
  void findTwo_doneCb(const actionlib::SimpleClientGoalState&, const pr2_gripper_sensor_msgs::PR2GripperFindContactResultConstPtr&);
  void findTwo_activeCb();
  void findTwo_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperFindContactFeedbackConstPtr&);
  void findTwoContacts();
  void findTwo_cancel();

  // SLIPSERVO
  bool slipServo_isDone();
  actionlib::SimpleClientGoalState slipServo_getState();  
  void slipServo_doneCb(const actionlib::SimpleClientGoalState&, const pr2_gripper_sensor_msgs::PR2GripperSlipServoResultConstPtr&);
  void slipServo_activeCb();
  void slipServo_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperSlipServoFeedbackConstPtr&);
  void slipServo();
  void slipServo_cancel();

  // PLACE
  ERROR setPlaceTriggerConditions(int8_t);
  int8_t getPlaceTriggerConditions();
  ERROR setPlaceAccelerationTriggerMagnitude(double);
  double getPlaceAccelerationTriggerMagnitude();
  ERROR setPlaceSlipTriggerMagnitude(double);
  double getPlaceSlipTriggerMagnitude();
  bool place_isDone();
  actionlib::SimpleClientGoalState place_getState();
  void place_doneCb(const actionlib::SimpleClientGoalState&,const pr2_gripper_sensor_msgs::PR2GripperEventDetectorResultConstPtr& result);
  void place_activeCb();
  void place_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedbackConstPtr&);
  void place(); 
  void place_cancel();

 // GRAB
  bool grab_isDone();
  actionlib::SimpleClientGoalState grab_getState();
  void grab_doneCb(const actionlib::SimpleClientGoalState&, const pr2_gripper_sensor_msgs::PR2GripperGrabResultConstPtr&);
  void grab_activeCb();
  void grab_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperGrabFeedbackConstPtr&);
  void grab();
  void grab_cancel();

 // RELEASE
  bool release_isDone();
  actionlib::SimpleClientGoalState release_getState();
  void release_doneCb(const actionlib::SimpleClientGoalState&, const pr2_gripper_sensor_msgs::PR2GripperReleaseResultConstPtr&);
  void release_activeCb();
  void release_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperReleaseFeedbackConstPtr&);
  void release();
  void release_cancel();

};
#endif
