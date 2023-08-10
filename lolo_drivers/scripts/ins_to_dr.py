#! /usr/bin/env python3
# -*- coding: utf-8 -*-


"""
Since the INS handles all DR on Lolo, we need
a node that will publish what the DR stack used to
publish in SAM:
    for the BT:
        dr/lat_lon
    for nodered, in addition to BT stuff above:
        dr/
            depth
            roll
            pitch
            yaw

there is some node in lolo_common that does roll/pitch/yaw from dr/odom,
but who publishes dr/odom in the new lolo?

# Ins message:
# int8 ALT_REF_GEOID=0
# int8 ALT_REF_ELLIPSOID=1
# std_msgs/Header header
  # uint32 seq
  # time stamp
  # string frame_id
# float64 latitude
# float64 longitude
# int8 altitude_ref
# float32 altitude
# float64[9] position_covariance
# float32 heading
# float32 roll
# float32 pitch
# float64[9] attitude_covariance
# geometry_msgs/Vector3 speed_vessel_frame
  # float64 x
  # float64 y
  # float64 z
# float64[9] speed_vessel_frame_covariance
"""

import rospy, tf
# dr/lat_lon
from geographic_msgs.msg import GeoPoint
# dr/depth, roll, pitch, yaw
from std_msgs.msg import Float64
# core/ins
from ixblue_ins_msgs.msg import Ins
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from smarc_msgs.srv import LatLonToUTM

class INSDr(object):
    def __init__(self,
                 ins_topic = "core/ins",
                 imu_topic = "core/imu",
                 robot_name = "lolo",
                 latlontoutm_service_name = "lat_lon_to_utm"):
        self.robot_name = robot_name
        self.twist_msg = Twist()


        topic_root = "/"+robot_name+"/dr/"
        self.latlon_pub = rospy.Publisher(topic_root+"lat_lon", GeoPoint, queue_size=1)
        self.depth_pub = rospy.Publisher(topic_root+"depth", Float64, queue_size=1)
        self.roll_pub = rospy.Publisher(topic_root+"roll", Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher(topic_root+"pitch", Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher(topic_root+"yaw", Float64, queue_size=1)
        self.twist_pub = rospy.Publisher(topic_root+"twist", Twist, queue_size=1)

        latlontoutm_service_name = topic_root + latlontoutm_service_name

        self.ins_sub = rospy.Subscriber("/"+robot_name+"/"+ins_topic,
                                        Ins,
                                        self.ins_cb,
                                        queue_size=1)
        self.imu_sub = rospy.Subscriber("/"+robot_name+"/"+imu_topic,
                                        Imu,
                                        self.imu_cb,
                                        queue_size=1)


        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Waiting for {} service...".format(latlontoutm_service_name))
                rospy.wait_for_service(latlontoutm_service_name, timeout=5)
                rospy.loginfo("Got it.")
                break
            except:
                rospy.loginfo("{} service not found, will keep waiting.".format(latlontoutm_service_name))

        self.ll2utm_service = rospy.ServiceProxy(latlontoutm_service_name,
                                                 LatLonToUTM)
        self.tfcaster = tf.TransformBroadcaster()
        self.tflistener = tf.TransformListener()

    def imu_cb(self,msg):
        self.twist_msg.angular.x = msg.angular_velocity.x
        self.twist_msg.angular.y = msg.angular_velocity.y
        self.twist_msg.angular.z = msg.angular_velocity.z

    def ins_cb(self, msg):

        msg.roll = msg.roll*3.1415/180
        msg.pitch = msg.pitch*3.1415/180
        msg.heading = (90-msg.heading)%360
        msg.heading = msg.heading*3.1415/180

        ll = GeoPoint()
        ll.latitude = msg.latitude
        ll.longitude = msg.longitude
        ll.altitude = msg.altitude
        self.latlon_pub.publish(ll)
        self.depth_pub.publish(-msg.altitude)
        self.roll_pub.publish(msg.roll)
        self.pitch_pub.publish(msg.pitch)
        self.yaw_pub.publish(msg.heading)

        self.twist_msg.linear = msg.speed_vessel_frame
        self.twist_pub.publish(self.twist_msg)

        # okay, now publish the same thing, as part of TF
        utm_res = self.ll2utm_service(ll)
        # we could have published utm->base link, but we should probably do map->baselink instead
        # so gotta transform this utm point to a world_ned point first
        ps = PoseStamped()
        ps.header.frame_id = "utm"
        ps.pose.position.x = utm_res.utm_point.x
        ps.pose.position.y = utm_res.utm_point.y
        ps.pose.position.z = utm_res.utm_point.z
        quat = tf.transformations.quaternion_from_euler(msg.roll, msg.pitch, msg.heading)
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]

        # now we publish the world_ned version of this utm pose
        world_ned_ps = self.tflistener.transformPose("map", ps)
        self.tfcaster.sendTransform((world_ned_ps.pose.position.x,
                                     world_ned_ps.pose.position.y,
                                     world_ned_ps.pose.position.z),
                                    (world_ned_ps.pose.orientation.x,
                                     world_ned_ps.pose.orientation.y,
                                     world_ned_ps.pose.orientation.z,
                                     world_ned_ps.pose.orientation.w),
                                    rospy.Time.now(),
                                    self.robot_name+"/base_link",
                                    "map")





if __name__ == "__main__":
    rospy.init_node("ins_to_dr")
    ins2dr = INSDr()
    while not rospy.is_shutdown():
        rospy.spin()























