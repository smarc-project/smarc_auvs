#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

#include "ros/ros.h"
#include "../CaptainInterFace/CaptainInterFace.h"

#include "captain_interface/scientistmsg.h"

#ifndef PI 
#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286
#endif

//Ros messages
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/BatteryState.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <nav_msgs/Odometry.h>
#include <smarc_msgs/UTMToLatLon.h>
#include <smarc_msgs/LatLonToUTMOdometry.h>
#include <smarc_msgs/LatLonOdometry.h>
#include <smarc_msgs/DVL.h>
#include <smarc_msgs/FloatStamped.h>
#include <smarc_msgs/ThrusterRPM.h>
#include <smarc_msgs/ThrusterFeedback.h>
#include <smarc_msgs/Leak.h>
#include <lolo_msgs/CaptainStatus.h>
#include <lolo_msgs/CaptainService.h>
#include <smarc_msgs/ControllerStatus.h>
#include <smarc_msgs/SensorStatus.h>
#include <ixblue_ins_msgs/Ins.h>

struct RosInterFace {

  //================== ROS Nodehandle ====================//
  ros::NodeHandle* n;
  CaptainInterFace* captain;

  void init(ros::NodeHandle* nh, CaptainInterFace* cap);

  //======================================================//
  //================== Service clients ===================//
  //======================================================//
  ros::ServiceClient utm_to_latlon_client;
  ros::ServiceClient odom_client;

  //======================================================//
  //================== ROS subscribers ===================//
  //======================================================//

  //TODO
  // USBL/RF/etc

  //Navigation data
  ros::Subscriber ins_sub;

  //usbl
  ros::Subscriber usbl_sub;

  //information / other things
  ros::Subscriber heartbeat_sub;        //heartbeat message
  ros::Subscriber done_sub;             //scientist mission completed
  ros::Subscriber abort_sub;            //abort

  //Control commands: High level
  ros::Subscriber waypoint_sub;         //target waypoint
  ros::Subscriber speed_sub;            //target speed
  ros::Subscriber depth_sub;            //target depth
  ros::Subscriber altitude_sub;         //target altitude

  //Control commands medium level
  ros::Subscriber yaw_sub;              //target course
  ros::Subscriber yawrate_sub;          //target turn rate
  ros::Subscriber pitch_sub;            //target pitch
  ros::Subscriber rpm_sub;              //target RPM (average if differential thrust is enabled)

  //Control commands low level
  //thrusters
  ros::Subscriber thrusterPort_sub;
  ros::Subscriber thrusterStrb_sub;

  //control surfaces
  ros::Subscriber rudder_sub;
  ros::Subscriber elevator_sub;
  ros::Subscriber elevon_port_sub;
  ros::Subscriber elevon_strb_sub;

  //"Service"
  ros::Subscriber service_sub;

  //Lolo onboard console
  ros::Subscriber menu_sub;

  //======================================================//
  //=================== ROS pubishers ====================//
  //======================================================//

  //usbl
  ros::Publisher usbl_pub;

  //thrusters
  ros::Publisher thrusterPort_pub;
  ros::Publisher thrusterStrb_pub;

  //constrol surfaces
  ros::Publisher rudder_angle_pub;
  ros::Publisher elevator_angle_pub;
  ros::Publisher elevon_port_angle_pub;
  ros::Publisher elevon_strb_angle_pub;

  //Battery
  ros::Publisher battery_pub;

  //Leak sensors
  ros::Publisher leak_dome;

  //control / status
  ros::Publisher status_orientation_pub;
  ros::Publisher status_altitude_pub;
  ros::Publisher status_position_pub;
  ros::Publisher status_depth_pub;
  ros::Publisher status_twist_pub;

  //Status publishers
  ros::Publisher control_status_pub;
  ros::Publisher vehiclestate_pub;

  //"Service"
  ros::Publisher service_pub;

  //General purpose text output
  ros::Publisher text_pub;

  //Lolo onboard console
  ros::Publisher menu_pub;

  //Temperature sensor publishers
  ros::Publisher temperature_cap_cpu_pub;
  ros::Publisher temperature_time_cpu_pub;
  ros::Publisher temperature_cap_sensor1_pub;
  ros::Publisher temperature_cap_sensor2_pub;
  ros::Publisher temperature_cap_sensor3_pub;
  ros::Publisher temperature_isb_usbl_pub;
  //ros::Publisher temperature_isb_4G_pub;
  //ros::Publisher temperature_isb_ethernet_pub;
  ros::Publisher temperature_isb_strobe_pub;
  //ros::Publisher temperature_isb_wifi_pub;
  ros::Publisher temperature_isb_rudders_pub;
  ros::Publisher temperature_isb_elevons_pub;
  ros::Publisher temperature_isb_elevator_pub;
  ros::Publisher temperature_isb_thruster_pub;
  ros::Publisher temperature_isb_scientist_pub;
  ros::Publisher temperature_isb_edw_pub;
  ros::Publisher temperature_isb_battery_pub;
  ros::Publisher temperature_battery_sensor1_pub;
  ros::Publisher temperature_battery_sensor2_pub;
  ros::Publisher temperature_battery_sensor3_pub;
  ros::Publisher temperature_battery_sensor4_pub;
  ros::Publisher temperature_battery_sensor5_pub;

  //Pressure sensor publishers
  ros::Publisher pressure_isb_usbl_pub;
  //ros::Publisher pressure_isb_4G_pub;
  //ros::Publisher pressure_isb_ethernet_pub;
  ros::Publisher pressure_isb_strobe_pub;
  //ros::Publisher pressure_isb_wifi_pub;
  ros::Publisher pressure_isb_rudders_pub;
  ros::Publisher pressure_isb_elevons_pub;
  ros::Publisher pressure_isb_elevator_pub;
  ros::Publisher pressure_isb_thruster_pub;
  ros::Publisher pressure_isb_scientist_pub;
  ros::Publisher pressure_isb_edw_pub;
  ros::Publisher pressure_isb_battery_pub;


  //Controller status publishers
  ros::Publisher ctrl_status_waypoint_pub;
  ros::Publisher ctrl_status_yaw_pub;
  ros::Publisher ctrl_status_yawrate_pub;
  ros::Publisher ctrl_status_depth_pub;
  ros::Publisher ctrl_status_altitude_pub;
  ros::Publisher ctrl_status_pitch_pub;
  ros::Publisher ctrl_status_speed_pub;
  ros::Publisher ctrl_status_rpm_pub;
  ros::Publisher ctrl_status_rpm_strb_pub;
  ros::Publisher ctrl_status_rpm_port_pub;
  ros::Publisher ctrl_status_elevator_pub;
  ros::Publisher ctrl_status_rudder_pub;
  ros::Publisher ctrl_status_VBS_pub;

  //Log publishers
  ros::Publisher missonlog_pub;
  ros::Publisher datalog_pub;

  //======================================================//
  //=================== ROS callbacks ====================//
  //======================================================//

  void ros_callback_heartbeat(const std_msgs::Empty::ConstPtr &_msg);
  void ros_callback_abort(const std_msgs::Empty::ConstPtr &_msg);
  //void ros_callback_done(const std_msgs::Empty::ConstPtr &_msg);
  void ros_callback_waypoint_utm(const geometry_msgs::Point::ConstPtr &_msg);
  void ros_callback_waypoint(const geographic_msgs::GeoPoint::ConstPtr &_msg);
  void ros_callback_speed(const std_msgs::Float64::ConstPtr &_msg);
  void ros_callback_depth(const std_msgs::Float64::ConstPtr &_msg);
  void ros_callback_altitude(const std_msgs::Float64::ConstPtr &_msg);
  void ros_callback_yaw(const std_msgs::Float64::ConstPtr &_msg);
  void ros_callback_yawrate(const std_msgs::Float64::ConstPtr &_msg);
  void ros_callback_pitch(const std_msgs::Float64::ConstPtr &_msg);
  void ros_callback_rpm(const smarc_msgs::ThrusterRPM::ConstPtr &_msg);
  void ros_callback_rudder(const std_msgs::Float32::ConstPtr &_msg);
  void ros_callback_elevator(const std_msgs::Float32::ConstPtr &_msg);
  void ros_callback_elevonPort(const std_msgs::Float32::ConstPtr &_msg);
  void ros_callback_elevonStrb(const std_msgs::Float32::ConstPtr &_msg);
  void ros_callback_thrusterPort(const smarc_msgs::ThrusterRPM::ConstPtr &_msg);
  void ros_callback_thrusterStrb(const smarc_msgs::ThrusterRPM::ConstPtr &_msg);
  void ros_callback_service(const lolo_msgs::CaptainService::ConstPtr &_msg);
  void ros_callback_menu(const std_msgs::String::ConstPtr &_msg);
  void ros_callback_ins(const ixblue_ins_msgs::Ins::ConstPtr & msg);
  void ros_callback_usbl_transmit(const std_msgs::Char::ConstPtr& msg);

  //======================================================//
  //================= Captain callbacks ==================//
  //======================================================//

  void captain_callback_LEAK();
  void captain_callback_CONTROL();
  void captain_callback_RUDDER();
  void captain_callback_ELEVATOR();
  void captain_callback_ELEVON_PORT();
  void captain_callback_ELEVON_STRB();
  void captain_callback_THRUSTER_PORT();
  void captain_callback_THRUSTER_STRB();
  void captain_callback_BATTERY();
  void captain_callback_SERVICE();
  void captain_callback_CTRL_STATUS();
  void captain_callback_TEXT();
  void captain_callback_MENUSTREAM();
  void captain_callback_MISSIONLOG();
  void captain_callback_DATALOG();
  void captain_callback_USBL_RECEIVED();
  void captain_callback_TEMP();
  void captain_callback_BARO();

  void captain_callback() {
    int msgID = captain->messageID();
    switch (msgID) {
      case CS_LEAK: {         captain_callback_LEAK(); }; break; //Leak
      case CS_CONTROL: {      captain_callback_CONTROL(); } break; //control
      case CS_RUDDER: {       captain_callback_RUDDER(); } break; // rudder
      case CS_ELEVATOR: {     captain_callback_ELEVATOR(); } break; //elevator
      case CS_ELEVON_PORT: {  captain_callback_ELEVON_PORT(); } break; //Port elevon
      case CS_ELEVON_STRB: {  captain_callback_ELEVON_STRB(); } break; //Strb elevon
      case CS_THRUSTER_PORT: {captain_callback_THRUSTER_PORT(); } break; //port thruster
      case CS_THRUSTER_STRB: {captain_callback_THRUSTER_STRB(); } break; //strb thruster
      case CS_BATTERY: {      captain_callback_BATTERY(); } break; //battery
      case CS_TEXT: {         captain_callback_TEXT(); } break;  //General purpose text message
      case CS_REQUEST_OUT:{   captain_callback_SERVICE(); } break; //"service call"
      case CS_MENUSTREAM: {   captain_callback_MENUSTREAM(); } break; //Menu stream data
      case CS_MISSIONLOG: {   captain_callback_MISSIONLOG(); } break; //Mission log stream data}
      case CS_DATALOG: {      captain_callback_DATALOG(); } break; //Data log stream data}
      case CS_USBL_RECEIVED: {captain_callback_USBL_RECEIVED(); } break; //USBL received}
      case CS_TEMP: {         captain_callback_TEMP();} break; //Data from temperature sensors inside lolo
      case CS_BARO: {         captain_callback_BARO();} break; //Data from pressure sensors inside lolo
    };
  };
};

#endif //ROSINTERFACE_H
