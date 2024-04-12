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
  uint64_t timestamp    = captain->parse_llong(); // timestamp from ISB
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float target = captain->parse_float();               // actuator target in radians
  float angle = captain->parse_float();                // actuator angle in radians
  float voltage = captain->parse_float();              // voltage
  float current = captain->parse_float();              // current
  float humidity = captain->parse_float();             // humidity
  float pcbtemp = captain->parse_float();              // pcbtemp
  float motortemp = captain->parse_float();            // motortemp

  //Detailed feedback
  lolo_msgs::VolzServo feedback_msg;
  feedback_msg.target = target;
  feedback_msg.angle = angle;
  feedback_msg.voltage = voltage;
  feedback_msg.current = current;
  //feedback_msg.humidity = humidity;
  feedback_msg.pcbtemp = pcbtemp;
  feedback_msg.motortemp = motortemp;
  rudder_feedback_pub.publish(feedback_msg);

  //Simple feedback
  smarc_msgs::FloatStamped angle_message;
  angle_message.data = angle;
  angle_message.header.stamp = ros::Time(sec,usec*1000);
  angle_message.header.frame_id = "lolo/rudder_port";
  rudder_angle_pub.publish(angle_message);

}

void RosInterFace::captain_callback_ELEVATOR() {
  uint64_t timestamp    = captain->parse_llong(); // timestamp from ISB
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float target = captain->parse_float();               // actuator target in radians
  float angle = captain->parse_float();                // actuator angle in radians
  float voltage = captain->parse_float();              // voltage
  float current = captain->parse_float();              // current
  float humidity = captain->parse_float();             // humidity
  float pcbtemp = captain->parse_float();              // pcbtemp
  float motortemp = captain->parse_float();            // motortemp

  //Detailed feedback
  lolo_msgs::VolzServo feedback_msg;
  feedback_msg.target = target;
  feedback_msg.angle = angle;
  feedback_msg.voltage = voltage;
  feedback_msg.current = current;
  //feedback_msg.humidity = humidity;
  feedback_msg.pcbtemp = pcbtemp;
  feedback_msg.motortemp = motortemp;
  elevator_feedback_pub.publish(feedback_msg);
  
  smarc_msgs::FloatStamped angle_message;
  angle_message.data = angle;
  angle_message.header.stamp = ros::Time(sec,usec*1000);
  angle_message.header.frame_id = "lolo/elvator";
  elevator_angle_pub.publish(angle_message);
  
}

void RosInterFace::captain_callback_ELEVON_PORT() {
  uint64_t timestamp    = captain->parse_llong(); // timestamp from ISB
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float target = captain->parse_float();               // actuator target in radians
  float angle = captain->parse_float();                // actuator angle in radians
  float voltage = captain->parse_float();              // voltage
  float current = captain->parse_float();              // current
  float humidity = captain->parse_float();             // humidity
  float pcbtemp = captain->parse_float();              // pcbtemp
  float motortemp = captain->parse_float();            // motortemp

  //Detailed feedback
  lolo_msgs::VolzServo feedback_msg;
  feedback_msg.target = target;
  feedback_msg.angle = angle;
  feedback_msg.voltage = voltage;
  feedback_msg.current = current;
  //feedback_msg.humidity = humidity;
  feedback_msg.pcbtemp = pcbtemp;
  feedback_msg.motortemp = motortemp;
  elevon_port_feedback_pub.publish(feedback_msg);

  smarc_msgs::FloatStamped angle_message;
  angle_message.data = angle;
  angle_message.header.stamp = ros::Time(sec,usec*1000);
  angle_message.header.frame_id = "lolo/elevon_port";
  elevon_port_angle_pub.publish(angle_message);
}

void RosInterFace::captain_callback_ELEVON_STRB() {
  uint64_t timestamp    = captain->parse_llong(); // timestamp from ISB
  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  float target = captain->parse_float();               // actuator target in radians
  float angle = captain->parse_float();                // actuator angle in radians
  float voltage = captain->parse_float();              // voltage
  float current = captain->parse_float();              // current
  float humidity = captain->parse_float();             // humidity
  float pcbtemp = captain->parse_float();              // pcbtemp
  float motortemp = captain->parse_float();            // motortemp

  //Detailed feedback
  lolo_msgs::VolzServo feedback_msg;
  feedback_msg.target = target;
  feedback_msg.angle = angle;
  feedback_msg.voltage = voltage;
  feedback_msg.current = current;
  //feedback_msg.humidity = humidity;
  feedback_msg.pcbtemp = pcbtemp;
  feedback_msg.motortemp = motortemp;
  elevon_strb_feedback_pub.publish(feedback_msg);

  smarc_msgs::FloatStamped angle_message;
  angle_message.data = angle;
  angle_message.header.stamp = ros::Time(sec,usec*1000);
  angle_message.header.frame_id = "lolo/elevon_stbd";
  elevon_strb_angle_pub.publish(angle_message);
}

void RosInterFace::captain_callback_THRUSTER_PORT() {
  uint64_t timestamp    = captain->parse_llong(); //timestamp from ISB
  uint32_t sequence     = captain->parse_long();  //sequence of this message
      
  float target          = captain->parse_float();
  float rpm             = captain->parse_float();
  float input_current   = captain->parse_float();
  float input_voltage   = captain->parse_float();
  float motor_current   = captain->parse_float();
  float tempMosfet      = captain->parse_float();

  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;

  smarc_msgs::ThrusterFeedback thruster_msg;
  thruster_msg.header.stamp = ros::Time(sec,usec*1000);
  thruster_msg.header.seq = sequence;
  thruster_msg.header.frame_id = "lolo/thruster_port";
  thruster_msg.rpm.rpm = rpm;
  thruster_msg.current = input_current;
  thruster_msg.torque = motor_current;
  thrusterPort_pub.publish(thruster_msg);

  lolo_msgs::VescFeedback vesc_feedback_msg;
  vesc_feedback_msg.target_rpm = target;
  vesc_feedback_msg.rpm = rpm;
  vesc_feedback_msg.input_current = input_current;
  vesc_feedback_msg.input_voltage = input_voltage;
  vesc_feedback_msg.motor_current = motor_current;
  vesc_feedback_msg.tempMosfet = tempMosfet;
  
  thrusterPortFeedback_pub.publish(vesc_feedback_msg);
}

void RosInterFace::captain_callback_THRUSTER_STRB() {
  uint64_t timestamp    = captain->parse_llong(); //timestamp from ISB
  uint32_t sequence     = captain->parse_long();  //sequence of this message
      
  float target          = captain->parse_float();
  float rpm             = captain->parse_float();
  float input_current   = captain->parse_float();
  float input_voltage   = captain->parse_float();
  float motor_current   = captain->parse_float();
  float tempMosfet      = captain->parse_float();

  uint64_t sec = timestamp / 1000000;
  uint64_t usec = timestamp % 1000000;
  
  smarc_msgs::ThrusterFeedback thruster_msg;
  thruster_msg.header.stamp = ros::Time(sec,usec*1000);
  thruster_msg.header.seq = sequence;
  thruster_msg.header.frame_id = "lolo/thruster_stbd";
  thruster_msg.rpm.rpm = rpm;
  thruster_msg.current = input_current;
  thruster_msg.torque = motor_current;
  thrusterStrb_pub.publish(thruster_msg);

  lolo_msgs::VescFeedback vesc_feedback_msg;
  vesc_feedback_msg.target_rpm = target;
  vesc_feedback_msg.rpm = rpm;
  vesc_feedback_msg.input_current = input_current;
  vesc_feedback_msg.input_voltage = input_voltage;
  vesc_feedback_msg.motor_current = motor_current;
  vesc_feedback_msg.tempMosfet = tempMosfet;
  
  thrusterStrbFeedback_pub.publish(vesc_feedback_msg);
}

void RosInterFace::captain_callback_VERTICAL_THRUSTER(int thruster_id) {
  uint64_t timestamp    = captain->parse_llong(); //timestamp from ISB    
  float target          = captain->parse_float();
  float rpm             = captain->parse_float();
  float input_current   = captain->parse_float();
  float input_voltage   = captain->parse_float();
  float motor_current   = captain->parse_float();
  float tempMosfet      = captain->parse_float();
  
  lolo_msgs::VescFeedback vesc_feedback_msg;
  vesc_feedback_msg.target_rpm = target;
  vesc_feedback_msg.rpm = rpm;
  vesc_feedback_msg.input_current = input_current;
  vesc_feedback_msg.input_voltage = input_voltage;
  vesc_feedback_msg.motor_current = motor_current;
  vesc_feedback_msg.tempMosfet = tempMosfet;
  switch (thruster_id)
  {
    case CS_VTHRUSTER_1: vertical_thruster_1_Feedback_pub.publish(vesc_feedback_msg); break;
    case CS_VTHRUSTER_2: vertical_thruster_2_Feedback_pub.publish(vesc_feedback_msg); break;
    case CS_VTHRUSTER_3: vertical_thruster_3_Feedback_pub.publish(vesc_feedback_msg); break;
    case CS_VTHRUSTER_4: vertical_thruster_4_Feedback_pub.publish(vesc_feedback_msg); break;  
  default:
    break;
  }
  thrusterStrbFeedback_pub.publish(vesc_feedback_msg);
}

void RosInterFace::captain_callback_BATTERY(int id) {
  // parse and publish battery information
  uint8_t battery_state = captain->parse_byte(); //Battery state. Check ros message doc for definition
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
  switch (id)
  {
    case CS_BATTERY1: 
      battery_msg.location = "Port battery";
      battery1_pub.publish(battery_msg); 
      break;
    case CS_BATTERY2: 
      battery_msg.location = "strb battery";
      battery2_pub.publish(battery_msg); 
      break;
  default:
    break;
  }
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
  lolo_msgs::Temperatures temperature_msg;
  temperature_msg.captain_cpu = captain->parse_byte();
  temperature_msg.time_cpu = captain->parse_byte();
  temperature_msg.captain_top = captain->parse_byte();
  temperature_msg.captain_eth = captain->parse_byte();
  temperature_msg.captain_nuc = captain->parse_byte();
  temperature_msg.usbl_isb = captain->parse_byte();
  temperature_msg.actuator_cpu = captain->parse_byte();
  temperature_msg.elevator_pcb = captain->parse_byte();
  temperature_msg.elevator_motor = captain->parse_byte();
  temperature_msg.rudder_motor = captain->parse_byte();
  temperature_msg.rudder_pcb = captain->parse_byte();
  temperature_msg.elevon_port_motor = captain->parse_byte();
  temperature_msg.elevon_port_pcb = captain->parse_byte();
  temperature_msg.elevon_strb_motor = captain->parse_byte();
  temperature_msg.elevon_strb_pcb = captain->parse_byte();
  temperature_msg.thruster_isb = captain->parse_byte();
  temperature_msg.port_esc = captain->parse_byte();
  temperature_msg.strb_esc = captain->parse_byte();
  temperature_msg.vertical_thruster_isb = captain->parse_byte();
  temperature_msg.vertical_thruster_1_esc = captain->parse_byte();
  temperature_msg.vertical_thruster_2_esc = captain->parse_byte();
  temperature_msg.vertical_thruster_3_esc = captain->parse_byte();
  temperature_msg.vertical_thruster_4_es = captain->parse_byte();
  temperature_msg.prevco_isb = captain->parse_byte();
  temperature_msg.edw_isb = captain->parse_byte();
  temperature_msg.battery1_isb = captain->parse_byte();
  temperature_msg.battery1_temp1 = captain->parse_byte();
  temperature_msg.battery1_temp2 = captain->parse_byte();
  temperature_msg.battery1_temp3 = captain->parse_byte();
  temperature_msg.battery1_temp4 = captain->parse_byte();
  temperature_msg.battery1_temp5 = captain->parse_byte();
  temperature_msg.battery2_isb = captain->parse_byte();
  temperature_msg.battery2_temp1 = captain->parse_byte();
  temperature_msg.battery2_temp2 = captain->parse_byte();
  temperature_msg.battery2_temp3 = captain->parse_byte();
  temperature_msg.battery2_temp4 = captain->parse_byte();
  temperature_msg.battery2_temp5 = captain->parse_byte();
  temperature_pub.publish(temperature_msg);
}

void RosInterFace::captain_callback_BARO() {

    lolo_msgs::Pressures pressure_msg;
    pressure_msg.usbl_isb = captain->parse_float();
    pressure_msg.thrusters_isb = captain->parse_float();
    pressure_msg.vertical_thrusters_isb  = captain->parse_float();
    pressure_msg.prevco_isb = captain->parse_float();
    pressure_msg.edw_isb = captain->parse_float();
    pressure_msg.battery1_isb = captain->parse_float();
    pressure_msg.battery2_isb = captain->parse_float();
    pressure_pub.publish(pressure_msg);
}

