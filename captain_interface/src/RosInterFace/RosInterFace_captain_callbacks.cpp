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
  // parse and publish battery information
  uint8_t battery_state = captain->parse_byte(); //Battery state. Chech ros message doc for definition
  float BatteryVoltage  = captain->parse_float();
  float BatteryCurrent  = captain->parse_float();
  float SOC             = captain->parse_float();
  float CellVoltage_0   = captain->parse_float();
  float CellVoltage_1   = captain->parse_float();
  float CellVoltage_2   = captain->parse_float();
  float CellVoltage_3   = captain->parse_float();
  float CellVoltage_4   = captain->parse_float();
  float CellVoltage_5   = captain->parse_float();
  float CellVoltage_6   = captain->parse_float();
  float CellVoltage_7   = captain->parse_float();
  float CellVoltage_8   = captain->parse_float();
  float CellVoltage_9   = captain->parse_float();
  float temp1           = captain->parse_float();
  float temp2           = captain->parse_float();
  float temp3           = captain->parse_float();
  float temp4           = captain->parse_float();
  float temp5           = captain->parse_float();

  sensor_msgs::BatteryState battery_msg;
  
  //std_msgs/Header header
  battery_msg.voltage = BatteryVoltage;
  battery_msg.current = BatteryCurrent;
  battery_msg.charge = SOC*210.0/100.0;
  battery_msg.capacity = 210;
  battery_msg.design_capacity = 210;
  battery_msg.percentage = SOC;
  battery_msg.power_supply_status = battery_state;
  battery_msg.power_supply_health = battery_msg.POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_msg.power_supply_technology = battery_msg.POWER_SUPPLY_TECHNOLOGY_LIFE;
  battery_msg.present = true;
  battery_msg.cell_voltage.push_back(CellVoltage_0);
  battery_msg.cell_voltage.push_back(CellVoltage_1);
  battery_msg.cell_voltage.push_back(CellVoltage_2);
  battery_msg.cell_voltage.push_back(CellVoltage_3);
  battery_msg.cell_voltage.push_back(CellVoltage_4);
  battery_msg.cell_voltage.push_back(CellVoltage_5);
  battery_msg.cell_voltage.push_back(CellVoltage_6);
  battery_msg.cell_voltage.push_back(CellVoltage_7);
  battery_msg.cell_voltage.push_back(CellVoltage_8);
  battery_msg.cell_voltage.push_back(CellVoltage_9);
  battery_msg.location = "Starboard battery";
  battery_pub.publish(battery_msg);

}

void RosInterFace::captain_callback_CTRL_STATUS() {
  /*
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
  */
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

void RosInterFace::captain_callback_TEMP() 
{
  float temperature_cap_cpu = captain->parse_float();
  float temperature_time_cpu = captain->parse_float();
  float temperature_cap_sensor1 = captain->parse_float();
  float temperature_cap_sensor2 = captain->parse_float();
  float temperature_cap_sensor3 = captain->parse_float();
  float temperature_isb_usbl = captain->parse_float();
  float temperature_isb_4G = captain->parse_float();
  float temperature_isb_ethernet = captain->parse_float();
  float temperature_isb_strobe = captain->parse_float();
  float temperature_isb_wifi = captain->parse_float();
  float temperature_isb_rudders = captain->parse_float();
  float temperature_isb_elevons = captain->parse_float();
  float temperature_isb_elevator = captain->parse_float();
  float temperature_isb_thruster = captain->parse_float();
  float temperature_isb_scientist = captain->parse_float();
  float temperature_isb_edw = captain->parse_float();
  float temperature_isb_battery = captain->parse_float();
  float temperature_battery_sensor1 = captain->parse_float();
  float temperature_battery_sensor2 = captain->parse_float();
  float temperature_battery_sensor3 = captain->parse_float();
  float temperature_battery_sensor4 = captain->parse_float();
  float temperature_battery_sensor5 = captain->parse_float();

  std_msgs::Float32 temperature_cap_cpu_msg; temperature_cap_cpu_msg.data = temperature_cap_cpu;
  std_msgs::Float32 temperature_time_cpu_msg; temperature_time_cpu_msg.data = temperature_time_cpu;
  std_msgs::Float32 temperature_cap_sensor1_msg; temperature_cap_sensor1_msg.data = temperature_cap_sensor1;
  std_msgs::Float32 temperature_cap_sensor2_msg; temperature_cap_sensor2_msg.data = temperature_cap_sensor2;
  std_msgs::Float32 temperature_cap_sensor3_msg; temperature_cap_sensor3_msg.data = temperature_cap_sensor3;
  std_msgs::Float32 temperature_isb_usbl_msg; temperature_isb_usbl_msg.data = temperature_isb_usbl;
  std_msgs::Float32 temperature_isb_4G_msg; temperature_isb_4G_msg.data = temperature_isb_4G;
  std_msgs::Float32 temperature_isb_ethernet_msg; temperature_isb_ethernet_msg.data = temperature_isb_ethernet;
  std_msgs::Float32 temperature_isb_strobe_msg; temperature_isb_strobe_msg.data = temperature_isb_strobe;
  std_msgs::Float32 temperature_isb_wifi_msg; temperature_isb_wifi_msg.data = temperature_isb_wifi;
  std_msgs::Float32 temperature_isb_rudders_msg; temperature_isb_rudders_msg.data = temperature_isb_rudders;
  std_msgs::Float32 temperature_isb_elevons_msg; temperature_isb_elevons_msg.data = temperature_isb_elevons;
  std_msgs::Float32 temperature_isb_elevator_msg; temperature_isb_elevator_msg.data = temperature_isb_elevator;
  std_msgs::Float32 temperature_isb_thruster_msg; temperature_isb_thruster_msg.data = temperature_isb_thruster;
  std_msgs::Float32 temperature_isb_scientist_msg; temperature_isb_scientist_msg.data = temperature_isb_scientist;
  std_msgs::Float32 temperature_isb_edw_msg; temperature_isb_edw_msg.data = temperature_isb_edw;
  std_msgs::Float32 temperature_isb_battery_msg; temperature_isb_battery_msg.data = temperature_isb_battery;
  std_msgs::Float32 temperature_battery_sensor1_msg; temperature_battery_sensor1_msg.data = temperature_battery_sensor1;
  std_msgs::Float32 temperature_battery_sensor2_msg; temperature_battery_sensor2_msg.data = temperature_battery_sensor2;
  std_msgs::Float32 temperature_battery_sensor3_msg; temperature_battery_sensor3_msg.data = temperature_battery_sensor3;
  std_msgs::Float32 temperature_battery_sensor4_msg; temperature_battery_sensor4_msg.data = temperature_battery_sensor4;
  std_msgs::Float32 temperature_battery_sensor5_msg; temperature_battery_sensor5_msg.data = temperature_battery_sensor5;
  
  temperature_cap_cpu_pub.publish(temperature_cap_cpu_msg);
  temperature_time_cpu_pub.publish(temperature_time_cpu_msg);
  temperature_cap_sensor1_pub.publish(temperature_cap_sensor1_msg);
  temperature_cap_sensor2_pub.publish(temperature_cap_sensor2_msg);
  temperature_cap_sensor3_pub.publish(temperature_cap_sensor3_msg);
  temperature_isb_usbl_pub.publish(temperature_isb_usbl_msg);
  temperature_isb_4G_pub.publish(temperature_isb_4G_msg);
  temperature_isb_ethernet_pub.publish(temperature_isb_ethernet_msg);
  temperature_isb_strobe_pub.publish(temperature_isb_strobe_msg);
  temperature_isb_wifi_pub.publish(temperature_isb_wifi_msg);
  temperature_isb_rudders_pub.publish(temperature_isb_rudders_msg);
  temperature_isb_elevons_pub.publish(temperature_isb_elevons_msg);
  temperature_isb_elevator_pub.publish(temperature_isb_elevator_msg);
  temperature_isb_thruster_pub.publish(temperature_isb_thruster_msg);
  temperature_isb_scientist_pub.publish(temperature_isb_scientist_msg);
  temperature_isb_edw_pub.publish(temperature_isb_edw_msg);
  temperature_isb_battery_pub.publish(temperature_isb_battery_msg);
  temperature_battery_sensor1_pub.publish(temperature_battery_sensor1_msg);
  temperature_battery_sensor2_pub.publish(temperature_battery_sensor2_msg);
  temperature_battery_sensor3_pub.publish(temperature_battery_sensor3_msg);
  temperature_battery_sensor4_pub.publish(temperature_battery_sensor4_msg);
  temperature_battery_sensor5_pub.publish(temperature_battery_sensor5_msg);
}

void RosInterFace::captain_callback_BARO() {

  float pressure_isb_usbl = captain->parse_float();
  float pressure_isb_4G = captain->parse_float();
  float pressure_isb_ethernet = captain->parse_float();
  float pressure_isb_strobe = captain->parse_float();
  float pressure_isb_wifi = captain->parse_float();
  float pressure_isb_rudders = captain->parse_float();
  float pressure_isb_elevons = captain->parse_float();
  float pressure_isb_elevator = captain->parse_float();
  float pressure_isb_thruster = captain->parse_float();
  float pressure_isb_scientist = captain->parse_float();
  float pressure_isb_edw = captain->parse_float();
  float pressure_isb_battery = captain->parse_float();


  std_msgs::Float32 pressure_isb_usbl_msg; pressure_isb_usbl_msg.data = pressure_isb_usbl;
  std_msgs::Float32 pressure_isb_4G_msg; pressure_isb_4G_msg.data = pressure_isb_4G;
  std_msgs::Float32 pressure_isb_ethernet_msg; pressure_isb_ethernet_msg.data = pressure_isb_ethernet;
  std_msgs::Float32 pressure_isb_strobe_msg; pressure_isb_strobe_msg.data = pressure_isb_strobe;
  std_msgs::Float32 pressure_isb_wifi_msg; pressure_isb_wifi_msg.data = pressure_isb_wifi;
  std_msgs::Float32 pressure_isb_rudders_msg; pressure_isb_rudders_msg.data = pressure_isb_rudders;
  std_msgs::Float32 pressure_isb_elevons_msg; pressure_isb_elevons_msg.data = pressure_isb_elevons;
  std_msgs::Float32 pressure_isb_elevator_msg; pressure_isb_elevator_msg.data = pressure_isb_elevator;
  std_msgs::Float32 pressure_isb_thruster_msg; pressure_isb_thruster_msg.data = pressure_isb_thruster;
  std_msgs::Float32 pressure_isb_scientist_msg; pressure_isb_scientist_msg.data = pressure_isb_scientist;
  std_msgs::Float32 pressure_isb_edw_msg; pressure_isb_edw_msg.data = pressure_isb_edw;
  std_msgs::Float32 pressure_isb_battery_msg; pressure_isb_battery_msg.data = pressure_isb_battery;

  pressure_isb_usbl_pub.publish(pressure_isb_usbl_msg);
  pressure_isb_4G_pub.publish(pressure_isb_4G_msg);
  pressure_isb_ethernet_pub.publish(pressure_isb_ethernet_msg);
  pressure_isb_strobe_pub.publish(pressure_isb_strobe_msg);
  pressure_isb_wifi_pub.publish(pressure_isb_wifi_msg);
  pressure_isb_rudders_pub.publish(pressure_isb_rudders_msg);
  pressure_isb_elevons_pub.publish(pressure_isb_elevons_msg);
  pressure_isb_elevator_pub.publish(pressure_isb_elevator_msg);
  pressure_isb_thruster_pub.publish(pressure_isb_thruster_msg);
  pressure_isb_scientist_pub.publish(pressure_isb_scientist_msg);
  pressure_isb_edw_pub.publish(pressure_isb_edw_msg);
  pressure_isb_battery_pub.publish(pressure_isb_battery_msg);
}

