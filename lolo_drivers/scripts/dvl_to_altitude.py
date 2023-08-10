#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import rospy
from cola2_msgs.msg import DVL
from std_msgs.msg import Float32

def dvl_cb(msg):
    global depth_pub
    alt_pub.publish(msg.altitude)


if __name__ == "__main__":
    rospy.init_node("dvl_to_altitude")
    robot_name = "lolo"

    alt_pub = rospy.Publisher("/"+robot_name+"/dr/altitude", Float32, queue_size=1)
    dvl_sub = rospy.Subscriber("/"+robot_name+"/core/dvl", DVL, dvl_cb, queue_size=1)

    while not rospy.is_shutdown():
        rospy.spin()
