#include "captain_interface/RosInterFace/RosInterFace.h"

void RosInterFace::init(ros::NodeHandle* nh, CaptainInterFace* cap) { 
  n = nh; captain = cap; 

  //==================================//
  //=========== Subscribers ==========//
  //==================================//

  //information / other things
  heartbeat_sub  = n->subscribe<std_msgs::Empty>("/lolo/core/heartbeat", 1, &RosInterFace::ros_callback_heartbeat, this);
  //done_sub  = n->subscribe<std_msgs::Empty>("/lolo/core/mission_complete", 1, &RosInterFace::ros_callback_done, this);
  abort_sub  = n->subscribe<std_msgs::Empty>("/lolo/core/abort", 1, &RosInterFace::ros_callback_abort, this);

  //ins
  ins_sub = n->subscribe<ixblue_ins_msgs::Ins>("/ixblue_ins_driver/ix/ins", 1, &RosInterFace::ros_callback_ins, this);

  //USBL
  usbl_sub = n->subscribe<std_msgs::Char>("/lolo/core/usbl/transmit", 1, &RosInterFace::ros_callback_usbl_transmit, this);

  /*
  //Control commands: High level
  waypoint_sub  = n->subscribe<geographic_msgs::GeoPoint>("/lolo/ctrl/waypoint_setpoint"  ,1, &RosInterFace::ros_callback_waypoint, this);
  speed_sub     = n->subscribe<std_msgs::Float64>("/lolo/ctrl/speed_setpoint"       ,1, &RosInterFace::ros_callback_speed,this);
  depth_sub     = n->subscribe<std_msgs::Float64>("/lolo/ctrl/depth_setpoint"       ,1, &RosInterFace::ros_callback_depth,this);
  altitude_sub  = n->subscribe<std_msgs::Float64>("/lolo/ctrl/altitude_setpoint"    ,1, &RosInterFace::ros_callback_altitude,this);

  //Control commands medium level
  yaw_sub       = n->subscribe<std_msgs::Float64>("/lolo/ctrl/yaw_setpoint"          ,1, &RosInterFace::ros_callback_yaw,this);
  yawrate_sub   = n->subscribe<std_msgs::Float64>("/lolo/ctrl/yawrate_setpoint"      ,1, &RosInterFace::ros_callback_yawrate,this);
  pitch_sub     = n->subscribe<std_msgs::Float64>("/lolo/ctrl/pitch_setpoint"        ,1, &RosInterFace::ros_callback_pitch,this);
  rpm_sub       = n->subscribe<smarc_msgs::ThrusterRPM>("/lolo/ctrl/rpm_setpoint"          ,1, &RosInterFace::ros_callback_rpm, this);
  */
 
  //Control commands low level
  //Thruster
  thrusterPort_sub = n->subscribe<smarc_msgs::ThrusterRPM>("/lolo/core/thruster1_cmd", 1, &RosInterFace::ros_callback_thrusterPort, this);
  thrusterStrb_sub = n->subscribe<smarc_msgs::ThrusterRPM>("/lolo/core/thruster2_cmd", 1, &RosInterFace::ros_callback_thrusterStrb, this);

  //control surfaces
  rudder_sub      = n->subscribe<std_msgs::Float32>("/lolo/core/rudder_cmd"   ,1, &RosInterFace::ros_callback_rudder, this);
  elevator_sub    = n->subscribe<std_msgs::Float32>("/lolo/core/elevator_cmd" ,1, &RosInterFace::ros_callback_elevator, this);
  elevon_port_sub = n->subscribe<std_msgs::Float32>("/lolo/core/elevon_port_cmd" ,1, &RosInterFace::ros_callback_elevonPort, this);
  elevon_strb_sub = n->subscribe<std_msgs::Float32>("/lolo/core/elevon_strb_cmd" ,1, &RosInterFace::ros_callback_elevonStrb, this);

  //"Service"
  service_sub     = n->subscribe<lolo_msgs::CaptainService>("/lolo/core/captain_srv_in" ,1, &RosInterFace::ros_callback_service, this);

  //menu
  menu_sub        = n->subscribe<std_msgs::String>("/lolo/console_in", 1, &RosInterFace::ros_callback_menu, this);

  //==================================//
  //=========== Publishers ===========//
  //==================================//

  //USBL
  usbl_pub             = n->advertise<std_msgs::Char>("/lolo/core/usbl/received", 10);

  // --- Thrusters --- //
  thrusterPort_pub     = n->advertise<smarc_msgs::ThrusterFeedback>("/lolo/core/thruster1_fb", 10);
  thrusterStrb_pub     = n->advertise<smarc_msgs::ThrusterFeedback>("/lolo/core/thruster2_fb", 10);

  // --- Rudders --- //
  rudder_angle_pub    = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/rudder_fb", 10);

  // --- Elevator --- //
  elevator_angle_pub      = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/elevator_fb", 10);

  // --- Elevons --- //
  elevon_port_angle_pub   = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/elevon_port_fb", 10);
  elevon_strb_angle_pub   = n->advertise<smarc_msgs::FloatStamped>("/lolo/core/elevon_strb_fb", 10);

  //Battery
  battery_pub = n->advertise<sensor_msgs::BatteryState>("/lolo/core/battery",10);

  //Leak sensors
  leak_dome   = n->advertise<smarc_msgs::Leak>("/lolo/core/leak", 10);

  //control_status_pub        = n->advertise<lolo_msgs::CaptainStatus>("/lolo/core/control_status", 10);
  //ctrl_status_waypoint_pub  = n->advertise<smarc_msgs::ControllerStatus>("/lolo/ctrl/onboard_waypoint_controller_status",10);
  //ctrl_status_yaw_pub       = n->advertise<smarc_msgs::ControllerStatus>("/lolo/ctrl/onboard_yaw_controller_status",10);
  //ctrl_status_yawrate_pub   = n->advertise<smarc_msgs::ControllerStatus>("/lolo/ctrl/onboard_yawrate_controller_status",10);
  //ctrl_status_depth_pub     = n->advertise<smarc_msgs::ControllerStatus>("/lolo/ctrl/onboard_depth_controller_status",10);
  //ctrl_status_altitude_pub  = n->advertise<smarc_msgs::ControllerStatus>("/lolo/ctrl/onboard_altitude_controller_status",10);
  //ctrl_status_pitch_pub     = n->advertise<smarc_msgs::ControllerStatus>("/lolo/ctrl/onboard_pitch_controller_status",10);
  //ctrl_status_speed_pub     = n->advertise<smarc_msgs::ControllerStatus>("/lolo/ctrl/onboard_speed_controller_status",10);
  //ctrl_status_rpm_pub      = n->advertise<lolo_msgs::ControllerStatus>("/lolo/ctrl/onboard_rpm_controller_status");
  //ctrl_status_rpm_strb_pub = n->advertise<lolo_msgs::ControllerStatus>("/lolo/ctrl/onboard_rpm_strb_controller_status");
  //ctrl_status_rpm_port_pub = n->advertise<lolo_msgs::ControllerStatus>("/lolo/ctrl/onboard_rpm_port_controller_status");
  //ctrl_status_elevator_pub = n->advertise<lolo_msgs::ControllerStatus>("/lolo/ctrl/onboard_elevator_controller_status");
  //ctrl_status_rudder_pub   = n->advertise<lolo_msgs::ControllerStatus>("/lolo/ctrl/onboard_rudder_controller_status");
  //ctrl_status_VBS_pub      = n->advertise<lolo_msgs::ControllerStatus>("/lolo/ctrl/onboard_VBS_controller_status");

  //"Service"
  service_pub             = n->advertise<lolo_msgs::CaptainService>("/lolo/core/captain_srv_out", 10);

  //General purpose text message
  text_pub   = n->advertise<std_msgs::String>("/lolo/text", 10);

  //Lolo console menu
  menu_pub  = n->advertise<std_msgs::String>("/lolo/console_out", 10);

  //Log publishers
  missonlog_pub = n->advertise<std_msgs::String>("/lolo/log/mission", 1);
  datalog_pub = n->advertise<std_msgs::String>("/lolo/log/data", 1);
};
