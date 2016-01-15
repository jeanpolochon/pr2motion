#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandFeedback.h>
#include <pr2_controllers_msgs/Pr2GripperCommandResult.h>

#include <pr2_gripper_sensor_msgs/PR2GripperFindContactAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperSlipServoAction> SlipClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperFindContactAction> ContactClient;
// Our Action interface type, provided as a typedef for convenience                   
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction> EventDetectorClient;
// Our Action interface type, provided as a typedef for convenience                   
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
// Our Action interface type, provided as a typedef for convenience                   

class Gripper{
private:
  GripperClient* gripper_client_;  
  ContactClient* contact_client_;
  SlipClient* slip_client_;
  EventDetectorClient* event_detector_client_;
  actionlib::SimpleClientGoalState::StateEnum close_state_;
  actionlib::SimpleClientGoalState::StateEnum open_state_;
  actionlib::SimpleClientGoalState::StateEnum findTwo_state_;
  actionlib::SimpleClientGoalState::StateEnum slipservo_state_;
  actionlib::SimpleClientGoalState::StateEnum place_state_;

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
  bool close_isDone();
  actionlib::SimpleClientGoalState close_getState();
  void close_doneCb(const actionlib::SimpleClientGoalState&, const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr&);
  void close_activeCb();
  void close_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr&);
  void close(pr2_controllers_msgs::Pr2GripperCommandGoal);
  void close_cancel();

  // OPEN
  bool open_isDone();
  actionlib::SimpleClientGoalState open_getState();
  void open_doneCb(const actionlib::SimpleClientGoalState&, const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr&);
  void open_activeCb();
  void open_feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr&);
  void open(pr2_controllers_msgs::Pr2GripperCommandGoal);
  void open_cancel();

  // FINDTWO
  bool findTwo_isDone();
  actionlib::SimpleClientGoalState findTwo_getState();  
  void findTwo_doneCb(const actionlib::SimpleClientGoalState&, const pr2_gripper_sensor_msgs::PR2GripperFindContactResultConstPtr&);
  void findTwo_activeCb();
  void findTwo_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperFindContactFeedbackConstPtr&);
  void findTwoContacts(pr2_gripper_sensor_msgs::PR2GripperFindContactGoal);
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
  bool place_isDone();
  actionlib::SimpleClientGoalState place_getState();
  void place_doneCb(const actionlib::SimpleClientGoalState&,const pr2_gripper_sensor_msgs::PR2GripperEventDetectorResultConstPtr& result);
  void place_activeCb();
  void place_feedbackCb(const pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedbackConstPtr&);
  void place(pr2_gripper_sensor_msgs::PR2GripperEventDetectorGoal); 
  void place_cancel();
};
