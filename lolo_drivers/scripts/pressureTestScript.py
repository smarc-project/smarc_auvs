#! /usr/bin/env python3

import math
import numpy as np
import rospy
from std_msgs.msg import Float64, Float32
from smarc_msgs.msg import ThrusterRPM

class asdf(object):

    def __init__(self):
        
        #Output amplitude
        self.servo_amplitude = 0.5


        self.thruster_rpm_msg = ThrusterRPM()
        self.thruster_rpm_msg.rpm = 500


        #"Speed" 
        self.dx = 0.02
        self.x = 0
        self.y = 0
        
        #Outputs
        self.rudder_pub = rospy.Publisher("/lolo/core/rudder_cmd", Float32, queue_size=1)
        self.elevon_port_pub = rospy.Publisher("/lolo/core/elevon_port_cmd", Float32, queue_size=1)
        self.elevon_strb_pub = rospy.Publisher("/lolo/core/elevon_strb_cmd", Float32, queue_size=1)
        self.elevator_pub = rospy.Publisher("/lolo/core/elevator_cmd", Float32, queue_size=1)
        self.thruster_port_pub = rospy.Publisher("/lolo/core/thruster1_cmd", ThrusterRPM, queue_size=1)
        self.thruster_strb_pub = rospy.Publisher("/lolo/core/thruster2_cmd", ThrusterRPM, queue_size=1)
    
    def update(self):
        print("update")

        servo_anlge = self.servo_amplitude*np.sin(self.x)
        self.x += self.dx
        if(self.x > 2*math.pi):
            self.x -=2*math.pi
        #self.x = self.x % 2*math.pi
        
        self.rudder_pub.publish(servo_anlge)
        self.elevator_pub.publish(servo_anlge)
        self.elevon_port_pub.publish(servo_anlge)
        self.elevon_strb_pub.publish(servo_anlge)

        self.thruster_port_pub.publish(self.thruster_rpm_msg)
        self.thruster_strb_pub.publish(self.thruster_rpm_msg)


if __name__ == '__main__':
    rospy.init_node("setpointsender")
    update_rate = float(rospy.get_param("~update_rate", 10))
    
    tester = asdf()

    r = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        tester.update()
        r.sleep()
