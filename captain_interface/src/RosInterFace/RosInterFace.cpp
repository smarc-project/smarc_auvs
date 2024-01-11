#include "captain_interface/RosInterFace/RosInterFace.h"

void RosInterFace::init(ros::NodeHandle* nh, CaptainInterFace* cap) { 
  n = nh; captain = cap; 

  //==================================//
  //=========== Subscribers ==========//
  //==================================//

  //information / other things
  heartbeat_sub  = n->subscribe<std_msgs::Empty>("/lolo/core/heartbeat", 1, &RosInterFace::ros_callback_heartbeat, this);
  //done_sub  = n->subscribe<std_msgs::Empty>("/lolo/core/mission_complete", 1, &RosInterFace::ros_callback_done, this);
  abort_sub  = n->subscribe<std_msgs::Empty>("/lolo/core/abort", 10, &RosInterFace::ros_callback_abort, this);

  //ins
  ins_sub = n->subscribe<ixblue_ins_msgs::Ins>("/ixblue_ins_driver/ix/ins", 1, &RosInterFace::ros_callback_ins, this);

  //USBL
  usbl_sub = n->subscribe<std_msgs::Char>("/lolo/core/usbl/transmit", 1024, &RosInterFace::ros_callback_usbl_transmit, this);

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
  usbl_pub             = n->advertise<std_msgs::Char>("/lolo/core/usbl/received", 1024);

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

  //Temperature sensors
  temperature_cap_cpu_pub          = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/cap_cpu",  1);
  temperature_time_cpu_pub         = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/time_cpu", 1);
  temperature_cap_sensor1_pub      = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/cap_sensor1", 1);
  temperature_cap_sensor2_pub      = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/cap_sensor2", 1);
  temperature_cap_sensor3_pub      = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/cap_sensor3", 1);
  temperature_isb_usbl_pub         = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/isb_usbl", 1);
  //temperature_isb_4G_pub           = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/isb_4G", 1);
  //temperature_isb_ethernet_pub     = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/isb_ethernet", 1);
  temperature_isb_strobe_pub       = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/isb_strobe", 1);
  //temperature_isb_wifi_pub         = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/isb_wifi", 1);
  temperature_isb_rudders_pub      = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/isb_rudders",  1);
  temperature_isb_elevons_pub      = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/isb_rlevons",  1);
  temperature_isb_elevator_pub     = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/isb_elevator", 1);
  temperature_isb_thruster_pub     = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/isb_thruster",  1);
  temperature_isb_scientist_pub    = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/isb_scientist",  1);
  temperature_isb_edw_pub          = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/isb_edw",  1);
  temperature_isb_battery_pub      = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/isb_battery",  1);
  temperature_battery_sensor1_pub  = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/battery_sensor1",  1);
  temperature_battery_sensor2_pub  = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/battery_sensor2",  1);
  temperature_battery_sensor3_pub  = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/battery_sensor3",  1);
  temperature_battery_sensor4_pub  = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/battery_sensor4",  1);
  temperature_battery_sensor5_pub  = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/temperature/battery_sensor5",  1);

  //pressure sensors
  pressure_isb_usbl_pub = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/pressure/isb_usbl", 1);
  //pressure_isb_4G_pub = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/pressure/isb_4G", 1);
  //pressure_isb_ethernet_pub = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/pressure/isb_ethernet", 1);
  pressure_isb_strobe_pub = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/pressure/isb_strobe", 1);
  //pressure_isb_wifi_pub = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/pressure/isb_wifi", 1);
  pressure_isb_rudders_pub = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/pressure/isb_rudders", 1);
  pressure_isb_elevons_pub = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/pressure/isb_elevons", 1);
  pressure_isb_elevator_pub = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/pressure/isb_elevator", 1);
  pressure_isb_thruster_pub = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/pressure/isb_thruster", 1);
  pressure_isb_scientist_pub = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/pressure/isb_scientist", 1);
  pressure_isb_edw_pub = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/pressure/isb_edw", 1);
  pressure_isb_battery_pub = n->advertise<std_msgs::Float32>("/lolo/core/diagnostic/pressure/isb_battery", 1);

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
