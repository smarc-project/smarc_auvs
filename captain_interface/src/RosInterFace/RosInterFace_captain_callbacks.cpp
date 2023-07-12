#include "captain_interface/RosInterFace/RosInterFace.h"
#include <limits.h>

void RosInterFace::captain_callback_LEAK() {
  smarc_msgs::Leak msg;
  leak_dome.publish(msg);
}

void RosInterFace::captain_callback_CONTROL() {
  //TODO send control feedback information
}

void RosInterFace::captain_callback_RUDDER() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  smarc_msgs::FloatStamped angle_message;
  angle_message.data = current_angle;
  angle_message.header.stamp = ros::Time(sec,usec*1000);
  angle_message.header.seq = sequence;
  angle_message.header.frame_id = "lolo/rudder_port";
  rudder_angle_pub.publish(angle_message);
}

void RosInterFace::captain_callback_ELEVATOR() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  smarc_msgs::FloatStamped angle_message;
  angle_message.data = current_angle;
  angle_message.header.stamp = ros::Time(sec,usec*1000);
  angle_message.header.seq = sequence;
  angle_message.header.frame_id = "lolo/elvator";
  elevator_angle_pub.publish(angle_message);
}

void RosInterFace::captain_callback_ELEVON_PORT() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  smarc_msgs::FloatStamped angle_message;
  angle_message.data = current_angle;
  angle_message.header.stamp = ros::Time(sec,usec*1000);
  angle_message.header.seq = sequence;
  angle_message.header.frame_id = "lolo/elevon_port";
  elevon_port_angle_pub.publish(angle_message);
}

void RosInterFace::captain_callback_ELEVON_STRB() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long(); 
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float target_angle    = captain->parse_float();
  float current_angle   = captain->parse_float();

  smarc_msgs::FloatStamped angle_message;
  angle_message.data = current_angle;
  angle_message.header.stamp = ros::Time(sec,usec*1000);
  angle_message.header.seq = sequence;
  angle_message.header.frame_id = "lolo/elevon_stbd";
  elevon_strb_angle_pub.publish(angle_message);
}

void RosInterFace::captain_callback_THRUSTER_PORT() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float rpm_setpoint    = captain->parse_float();
  float rpm             = captain->parse_float();
  float input_current   = captain->parse_float();
  float input_voltage   = captain->parse_float();
  float motor_current   = captain->parse_float();
  float esc_temp        = captain->parse_float();

  smarc_msgs::ThrusterFeedback thruster_msg;
  thruster_msg.header.stamp = ros::Time(sec,usec*1000);
  thruster_msg.header.seq = sequence;
  thruster_msg.header.frame_id = "lolo/thruster_port";
  thruster_msg.rpm.rpm = rpm;
  thruster_msg.current = input_current;
  thruster_msg.torque = motor_current;
  thrusterPort_pub.publish(thruster_msg);
}

void RosInterFace::captain_callback_THRUSTER_STRB() {
  uint64_t timestamp    = captain->parse_llong();
  uint32_t sequence     = captain->parse_long();
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float rpm_setpoint    = captain->parse_float();
  float rpm             = captain->parse_float();
  float input_current   = captain->parse_float();
  float input_voltage   = captain->parse_float();
  float motor_current   = captain->parse_float();
  float esc_temp        = captain->parse_float();
  
  smarc_msgs::ThrusterFeedback thruster_msg;
  thruster_msg.header.stamp = ros::Time(sec,usec*1000);
  thruster_msg.header.seq = sequence;
  thruster_msg.header.frame_id = "lolo/thruster_stbd";
  thruster_msg.rpm.rpm = rpm;
  thruster_msg.current = input_current;
  thruster_msg.torque = motor_current;
  thrusterStrb_pub.publish(thruster_msg);
}

void RosInterFace::captain_callback_BATTERY() {
  //TODO parse and publish battery information
}

void RosInterFace::captain_callback_CTRL_STATUS() {
  bool scientistinterface_enable_waypoint = captain->parse_byte();
  bool scientistinterface_enable_yaw      = captain->parse_byte();
  bool scientistinterface_enable_yawrate  = captain->parse_byte();
  bool scientistinterface_enable_depth    = captain->parse_byte();
  bool scientistinterface_enable_altitude = captain->parse_byte();
  bool scientistinterface_enable_pitch    = captain->parse_byte();
  bool scientistinterface_enable_speed    = captain->parse_byte();
  bool scientistinterface_enable_rpm      = captain->parse_byte();
  bool scientistinterface_enable_rpm_strb = captain->parse_byte();
  bool scientistinterface_enable_rpm_port = captain->parse_byte();
  bool scientistinterface_enable_elevator = captain->parse_byte();
  bool scientistinterface_enable_rudder   = captain->parse_byte();
  bool scientistinterface_enable_VBS      = captain->parse_byte();

  smarc_msgs::ControllerStatus msg_waypoint;
  msg_waypoint.control_status = scientistinterface_enable_waypoint;
  msg_waypoint.service_name = "/lolo/ctrl/toggle_onboard_waypoint_ctrl";
  ctrl_status_waypoint_pub.publish(msg_waypoint);

  smarc_msgs::ControllerStatus msg_yaw;
  msg_yaw.control_status = scientistinterface_enable_yaw;
  msg_yaw.service_name = "/lolo/ctrl/toggle_onboard_yaw_ctrl";
  ctrl_status_yaw_pub.publish(msg_waypoint);

  smarc_msgs::ControllerStatus msg_yawrate;
  msg_yawrate.control_status = scientistinterface_enable_yawrate;
  msg_yawrate.service_name = "/lolo/ctrl/toggle_onboard_yawrate_ctrl";
  ctrl_status_yawrate_pub.publish(msg_yawrate);

  smarc_msgs::ControllerStatus msg_depth;
  msg_depth.control_status = scientistinterface_enable_depth;
  msg_depth.service_name = "/lolo/ctrl/toggle_onboard_depth_ctrl";
  ctrl_status_depth_pub.publish(msg_depth);

  smarc_msgs::ControllerStatus msg_altitude;
  msg_altitude.control_status = scientistinterface_enable_altitude;
  msg_altitude.service_name = "/lolo/ctrl/toggle_onboard_altitude_ctrl";
  ctrl_status_altitude_pub.publish(msg_altitude);

  smarc_msgs::ControllerStatus msg_pitch;
  msg_pitch.control_status = scientistinterface_enable_pitch;
  msg_pitch.service_name = "/lolo/ctrl/toggle_onboard_pitch_ctrl";
  ctrl_status_pitch_pub.publish(msg_pitch);

  smarc_msgs::ControllerStatus msg_speed;
  msg_speed.control_status = scientistinterface_enable_speed;
  msg_speed.service_name = "/lolo/ctrl/toggle_onboard_speed_ctrl";
  ctrl_status_speed_pub.publish(msg_speed);
};

void RosInterFace::captain_callback_SERVICE() {
  std::cout << "Received service response from captain" << std::endl;
  lolo_msgs::CaptainService msg;
  msg.ref = captain->parse_int();
  msg.reply = captain->parse_byte();
  //TODO Add data to array if it ever gets used
  service_pub.publish(msg);
}

void RosInterFace::captain_callback_TEXT() {
  int length = captain->parse_byte();
  std::string text = captain->parse_string(length);
  std_msgs::String msg;
  msg.data = text.c_str();
  text_pub.publish(msg);
}

void RosInterFace::captain_callback_MENUSTREAM() {
  int length = captain->parse_byte();
  std::string text = captain->parse_string(length);
  printf("%s\n",text.c_str());
  std_msgs::String msg;
  msg.data = text.c_str();
  menu_pub.publish(msg);
}

void RosInterFace::captain_callback_MISSIONLOG() {
  int length = captain->parse_byte();
  std::string text = captain->parse_string(length);
  //printf("%s\n",text.c_str());
  std_msgs::String msg;
  msg.data = text.c_str();
  missonlog_pub.publish(msg);
}

void RosInterFace::captain_callback_DATALOG() {
  int length = captain->parse_byte();
  std::string text = captain->parse_string(length);
  //printf("%s\n",text.c_str());
  std_msgs::String msg;
  msg.data = text.c_str();
  datalog_pub.publish(msg);
}

void RosInterFace::captain_callback_USBL_RECEIVED() {
  int length = captain->parse_byte();
  std::cout << "USBL data received. length " << length << std::endl;
  for (int i=0;i<length;i++) {
	  std_msgs::Char msg;
	  msg.data = captain->parse_byte();
	  usbl_pub.publish(msg);
  }
  //std_msgs::String msg;
  //msg.data = text.c_str();
  //USBL_out_pub.publish(msg);
}

