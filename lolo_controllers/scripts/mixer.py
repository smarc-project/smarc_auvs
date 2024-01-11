#! /usr/bin/env python3

import math
import numpy as np
import rospy
from std_msgs.msg import Float64, Float32
from smarc_msgs.msg import ThrusterRPM

#Basic PID regulator
class control_mixer(object):

    def __init__(self):
        #Mixer gain parameters
        self.pitch_gain = 1
        self.yaw_gain = 1000

        #Last time an input was received. Used for timeouts
        self.lastyaw_time = 0
        self.lastroll_time = 0
        self.lastpitch_time = 0
        self.lastrpm_time = 0
        self.lastsurge_time = 0

        #last input values
        self.pitch_actuation = 0
        self.roll_actuation = 0
        self.yaw_actuation = 0
        self.rpm_actuation = 0
        self.vehicle_surge = 0

        #Control inputs
        self.yaw_sub = rospy.Subscriber("/lolo/ctrl/yaw_actuation", Float64, self.yaw_cb, queue_size=1)
        self.roll_sub = rospy.Subscriber("/lolo/ctrl/roll_actuation", Float64, self.roll_cb, queue_size=1)
        self.pitch_sub = rospy.Subscriber("/lolo/ctrl/pitch_actuation", Float64, self.pitch_cb, queue_size=1)
        self.rpm_sub = rospy.Subscriber("/lolo/ctrl/rpm_actuation", Float64, self.rpm_cb, queue_size=1)
        
        #Vehicle inputs
        self.surge_sub = rospy.Subscriber("/lolo/dr/surge", Float64, self.surge_cb, queue_size=1)
        
        #Output limits
        self.rudder_limit = 0.5 # ~30 deg
        self.elevon_limit = 0.5 # ~30 deg
        self.elevator_limit = 0.5 # ~30 deg
        self.thruster_limit = 800
        
        #Outputs
        self.rudder_pub = rospy.Publisher("/lolo/core/rudder_cmd", Float32, queue_size=1)
        self.elevon_port_pub = rospy.Publisher("/lolo/core/elevon_port_cmd", Float32, queue_size=1)
        self.elevon_strb_pub = rospy.Publisher("/lolo/core/elevon_strb_cmd", Float32, queue_size=1)
        self.thruster_port_pub = rospy.Publisher("/lolo/core/thruster1_cmd", ThrusterRPM, queue_size=1)
        self.thruster_strb_pub = rospy.Publisher("/lolo/core/thruster2_cmd", ThrusterRPM, queue_size=1)
        self.elevator_pub = rospy.Publisher("/lolo/core/elevator_cmd", Float32, queue_size=1)

    def pitch_cb(self,msg):
        self.pitch_actuation = msg.data
        self.lastpitch_time = rospy.get_time()
    def roll_cb(self,msg):
        self.roll_actuation = msg.data
        self.lastroll_time = rospy.get_time()
    def yaw_cb(self,msg):
        self.yaw_actuation = msg.data
        self.lastyaw_time = rospy.get_time()
    def rpm_cb(self,msg):
        self.rpm_actuation = msg.data
        self.lastrpm_time = rospy.get_time()
    def surge_cb(self,msg):
        self.surge_actuation = msg.data
        self.lastsurge_time = rospy.get_time()
    
    def update(self):
        #print("mixer update")
        now = rospy.get_time()

        #elevons
        elevon_port = None
        elevon_strb = None

        #Thrusters
        thruster_port = None
        thruster_strb = None
        
        #yaw
        if self.lastyaw_time is not None and now-self.lastyaw_time < 1:
            yaw_actuation = max(-self.rudder_limit, min(self.rudder_limit, self.yaw_actuation))
            self.rudder_pub.publish(yaw_actuation)
            thruster_port = self.yaw_gain*yaw_actuation
            thruster_strb = -self.yaw_gain*yaw_actuation


        #pitch
        if self.lastpitch_time is not None and now-self.lastpitch_time < 1:
            elevator_actuation = -max(-self.elevator_limit, min(self.elevator_limit, self.pitch_actuation))
            self.elevator_pub.publish(elevator_actuation)
            elevon_port = self.pitch_gain*elevator_actuation
            elevon_strb = self.pitch_gain*elevator_actuation

        #roll
        if self.lastpitch_time is not None and now-self.lastpitch_time < 1:
            if(elevon_port is not None): elevon_port += self.roll_actuation
            else: elevon_port=self.roll_actuation
            if(elevon_port is not None): elevon_strb -= self.roll_actuation
            else: elevon_strb=-self.roll_actuation
        if(elevon_port is not None): self.elevon_port_pub.publish(elevon_port) #TODO add limits
        if(elevon_strb is not None): self.elevon_strb_pub.publish(elevon_strb)

        #Thrusters 1: scale the values added by the yaw controller based on vehicle speed
        fadeout_scaling = 1
        if self.lastsurge_time is not None and now - self.lastsurge_time < 1:
            if self.vehicle_surge is not None and abs(self.vehicle_surge) < 1:
                fadeout_scaling = 1 - abs(self.vehicle_surge)
                fadeout_scaling = max(0, min(1, fadeout_scaling))
                pass
        if thruster_port is not None:
            thruster_port *= fadeout_scaling
        if thruster_strb is not None:
            thruster_strb *= fadeout_scaling

        #Thrusters 2: Add the desired RPM from the rpm setpoint
        if self.rpm_actuation is not None and now - self.lastrpm_time < 1:
            if thruster_port is not None: thruster_port += self.rpm_actuation
            else: thruster_port = self.rpm_actuation
            if thruster_strb is not None: thruster_strb += self.rpm_actuation
            else: thruster_strb = self.rpm_actuation
        
        #Publish thruster setpoints
        if thruster_port is not None:
            rpm = max(-self.thruster_limit, min(self.thruster_limit, thruster_port))
            self.thruster_port_pub.publish(int(rpm))
        if thruster_strb is not None:
            rpm = max(-self.thruster_limit, min(self.thruster_limit, thruster_strb))
            self.thruster_strb_pub.publish(int(rpm))
            


if __name__ == '__main__':
    rospy.init_node("contol_mixer")
    update_rate = float(rospy.get_param("~update_rate", 10))
    
    mixer =control_mixer()

    r = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        mixer.update()
        r.sleep()
