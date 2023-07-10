#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from math import radians
from math import pi
from math import sin
from smarc_msgs.msg import ThrusterRPM

def main():
    pub_elevon_port = rospy.Publisher('/lolo/core/elevon_port_cmd', Float32, queue_size=1)
    pub_elevon_strb = rospy.Publisher('/lolo/core/elevon_strb_cmd', Float32, queue_size=1)
    pub_rudder = rospy.Publisher('/lolo/core/rudder_cmd', Float32, queue_size=1)
    pub_thruster_port = rospy.Publisher('/lolo/core/thruster1_cmd', ThrusterRPM, queue_size=1)
    pub_thruster_strb = rospy.Publisher('/lolo/core/thruster2_cmd', ThrusterRPM, queue_size=1)

    t = 0

    rospy.init_node('control_tester', anonymous=True)
    rate = rospy.Rate(10) # 1hz
    while not rospy.is_shutdown():
        rate.sleep()
        t += 0.1
        #t = t%2*pi

        angle = radians(45)*sin(t)
        RPM = 200

        msg = Float32()
        msg.data = angle
        pub_elevon_port.publish(msg)
        pub_elevon_strb.publish(msg)
        pub_rudder.publish(msg)
        print("Publishing message: " + str(msg))

        thruster_msg = ThrusterRPM()
        thruster_msg.rpm = RPM

        if(angle < 0):
            pub_thruster_port.publish(thruster_msg)
            thruster_msg.rpm = 0
            pub_thruster_strb.publish(thruster_msg)
        else:
            pub_thruster_strb.publish(thruster_msg)
            thruster_msg.rpm = 0
            pub_thruster_port.publish(thruster_msg)

        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
