#include "captain_interface/RosInterFace/RosInterFace.h"

void RosInterFace::ros_callback_heartbeat(const std_msgs::Empty::ConstPtr &_msg) {
  captain->new_package(SC_HEARTBEAT); // Heartbeat message
  captain->send_package();
};

void RosInterFace::ros_callback_abort(const std_msgs::Empty::ConstPtr &_msg) {
  captain->new_package(SC_ABORT); // Tell captain to go into emergency mode
  captain->send_package();
};

/*
void RosInterFace::ros_callback_done(const std_msgs::Empty::ConstPtr &_msg) {
  captain->new_package(SC_DONE); // Tell captain that scientist is done
  captain->send_package();
};
*/

void RosInterFace::ros_callback_ins(const ixblue_ins_msgs::Ins::ConstPtr & msg) {

  captain->new_package(SC_NAV_DATA); // INS data
  captain->add_double(msg->latitude);
  captain->add_double(msg->longitude);
  float depth = -1.0*msg->altitude;
  captain->add_float(depth);

  captain->add_float((-3.1415/180.0) * msg->pitch);
  captain->add_float((3.1415/180.0) * msg->roll);
  captain->add_float((3.1415/180.0) * msg->heading);

  captain->add_float(msg->speed_vessel_frame.x);
  captain->add_float(msg->speed_vessel_frame.y);
  captain->add_float(msg->speed_vessel_frame.z);
  captain->send_package();
}

void RosInterFace::ros_callback_usbl_transmit(const std_msgs::Char::ConstPtr& msg) {
  captain->new_package(SC_USBL_TRANSMIT);
  captain->add_byte(1);
  captain->add_byte(msg->data);
  captain->send_package();
}

void RosInterFace::ros_callback_waypoint(const geographic_msgs::GeoPoint::ConstPtr &_msg) {
  double lat = _msg->latitude;
  double lon = _msg->longitude;
  captain->new_package(SC_SET_TARGET_WAYPOINT); // set target waypoint
  captain->add_double((PI / 180) * lat);
  captain->add_double((PI / 180) * lon);
  captain->send_package();
};

void RosInterFace::ros_callback_speed(const std_msgs::Float64::ConstPtr &_msg) {
  float targetSpeed = _msg->data;
  captain->new_package(SC_SET_TARGET_SPEED);
  captain->add_float(targetSpeed);
  captain->send_package();
};

void RosInterFace::ros_callback_depth(const std_msgs::Float64::ConstPtr &_msg) {
  float targetDepth = _msg->data;
  captain->new_package(SC_SET_TARGET_DEPTH);
  captain->add_float(targetDepth);
  captain->send_package();
};

void RosInterFace::ros_callback_altitude(const std_msgs::Float64::ConstPtr &_msg) {
  float targetAltitude = _msg->data;
  captain->new_package(SC_SET_TARGET_ALTITUDE);
  captain->add_float(targetAltitude);
  captain->send_package();
};

void RosInterFace::ros_callback_yaw(const std_msgs::Float64::ConstPtr &_msg) {
  float targetYaw = _msg->data;
  captain->new_package(SC_SET_TARGET_YAW);
  captain->add_float(targetYaw);
  captain->send_package();
};

void RosInterFace::ros_callback_yawrate(const std_msgs::Float64::ConstPtr &_msg) {
  float targetYawRate = _msg->data;
  captain->new_package(SC_SET_TARGET_YAW_RATE);
  captain->add_float(targetYawRate);
  captain->send_package();
};

void RosInterFace::ros_callback_pitch(const std_msgs::Float64::ConstPtr &_msg) {
  float targetPitch = _msg->data;
  captain->new_package(SC_SET_TARGET_PITCH);
  captain->add_float(targetPitch);
  captain->send_package();
};

void RosInterFace::ros_callback_rpm(const smarc_msgs::ThrusterRPM::ConstPtr &_msg) {
  float targetRPM = _msg->rpm;
  captain->new_package(SC_SET_TARGET_RPM);
  captain->add_float(targetRPM);
  captain->send_package();
};


void RosInterFace::ros_callback_rudder(const std_msgs::Float32::ConstPtr &_msg) {
  float angle = _msg->data;
  captain->new_package(SC_SET_RUDDER);
  captain->add_float(angle);
  captain->send_package();
};

void RosInterFace::ros_callback_elevator(const std_msgs::Float32::ConstPtr &_msg) {
  float angle = _msg->data;
  captain->new_package(SC_SET_ELEVATOR);
  captain->add_float(angle);
  captain->send_package();
};

void RosInterFace::ros_callback_elevonPort(const std_msgs::Float32::ConstPtr &_msg) {
  float angle = _msg->data;
  captain->new_package(SC_SET_ELEVON_PORT);
  captain->add_float(angle);
  captain->send_package();
};

void RosInterFace::ros_callback_elevonStrb(const std_msgs::Float32::ConstPtr &_msg) {
  float angle = _msg->data;
  captain->new_package(SC_SET_ELEVON_STRB);
  captain->add_float(angle);
  captain->send_package();
};

void RosInterFace::ros_callback_thrusterPort(const smarc_msgs::ThrusterRPM::ConstPtr &_msg) {
  captain->new_package(SC_SET_THRUSTER_PORT);
  captain->add_float(_msg->rpm);
  captain->send_package();
};

void RosInterFace::ros_callback_thrusterStrb(const smarc_msgs::ThrusterRPM::ConstPtr &_msg) {
  captain->new_package(SC_SET_THRUSTER_STRB);
  captain->add_float(_msg->rpm);
  captain->send_package();
};

void RosInterFace::ros_callback_service(const lolo_msgs::CaptainService::ConstPtr &_msg) {
  std::cout << "Send service request to captain" << std::endl;
  captain->new_package(SC_REQUEST_IN);
  captain->add_int(_msg->ref);
  captain->add_byte(_msg->id);
  captain->add_byte(_msg->action);
  //for(int i=0; i< _msg->data.size() && i < 200; i++) {
  //  captain->add_byte(data[i]);
  //}
  captain->send_package();
};

void RosInterFace::ros_callback_menu(const std_msgs::String::ConstPtr &_msg) {
  captain->new_package(SC_MENUSTREAM);
  std::string s = _msg->data;
  uint8_t bytes = std::min(200, (int) s.size());
  captain->add_byte(bytes);
  for(int i=0;i<bytes;i++) {
    captain->add_byte(s[i]);
  }
  captain->send_package();
};
