#! /usr/bin/env python3

import math
import numpy as np
import rospy
from std_msgs.msg import Float64
import geometry as geom

#Basic PID regulator
class PID(object):

    def __init__(self,kP=None,kI=None,kD=None,max_output = None):
        self.kP = kP if kP is not None else 0
        self.kI = kI if kI is not None else 0
        self.kD = kD if kD is not None else 0
        self.max_output = max_output
        self.integral = 0
        self.last_update = None
        self.last_error = None
        self.last_derivative = None
        self._fCut = 20
    
    def reset(self):
        self.integral = 0
        self.last_update = None
        self.last_meassurement = None

    def update_error(self, error):
        #print("PID update: " + str(error))
        current_time_s = rospy.get_time()
        dt = current_time_s - self.last_update if self.last_update is not None else None

        #reset I of dt > 1s
        if(dt is not None and dt > 1):
            self.integral = 0
            self.last_derivative = None

        proportional = self.kP*error
        if dt is not None and dt < 1: self.integral +=  dt*self.kI*error 

        if dt is not None and self.last_error is not None and self.last_derivative is not None:
            derivative = self.kD*(error - self.last_error) / dt

            # discrete low pass filter, cuts out the
            # high frequency noise that can drive the controller crazy
            RC = 1/(2*math.pi*self._fCut)
            derivative = self.last_derivative + ((dt / (RC + dt))*(derivative - self.last_derivative))
        else:
            derivative = 0
        self.last_derivative = derivative

        #prevent integral windup
        if self.max_output is not None:
            if(self.integral > self.max_output): self.integral = self.max_output
            if(self.integral < -self.max_output): self.integral = -self.max_output

        output = proportional + self.integral + derivative
        self.last_update = current_time_s
        self.last_error = error

        return max(-self.max_output, min(self.max_output, output)) if self.max_output is not None else output

        
    
#Wrapper for the PID class with subscribers and publishers
class PID_wrapper(object):
    def __init__(self, PID, meassurement_topic, setpoint_topic ,output_topic, version='Normal'):
        self.meassurement = 0
        self.meassurement_time = 0
        self.setpoint = 0
        self.setpoint_time = 0
        self.pid = PID
        self.version = version

        self.meassurement_sub = rospy.Subscriber(meassurement_topic, Float64, self.meassurement_cb, queue_size=1)
        self.setpoint_sub = rospy.Subscriber(setpoint_topic, Float64, self.setpoint_cb, queue_size=1)
        self.output_pub = rospy.Publisher(output_topic, Float64, queue_size=1)

    
    def meassurement_cb(self, msg):
        self.meassurement_time = rospy.get_time()
        self.meassurement = msg.data

    def setpoint_cb(self, msg):
        self.setpoint_time = rospy.get_time()
        self.setpoint = msg.data

    def update(self):
        #Check if we have timed out
        #print("wrapper update")
        now = rospy.get_time()
        if(now - self.meassurement_time < 1 and now - self.setpoint_time < 1):
            if self.version == 'yaw':
                setpoint = np.array([np.cos(self.setpoint) , np.sin(self.setpoint)])
                meassurement = np.array([np.cos(self.meassurement) , np.sin(self.meassurement)])
                error = -geom.vec2_directed_angle(setpoint, meassurement)
                #print("Yaw controller update")
            else:
                error = self.setpoint - self.meassurement
            output = self.pid.update_error(error)
            self.output_pub.publish(output)
        else:
            #print("setpoint or meassurement timout")
            pass


if __name__ == '__main__':
    rospy.init_node("pid_controller")
    update_rate = float(rospy.get_param("~update_rate", 20))
    p_gain = float(rospy.get_param("~p_gain", 0))
    i_gain = float(rospy.get_param("~i_gain", 0 ))
    d_gain = float(rospy.get_param("~d_gain", 0))
    output_limit = float(rospy.get_param("~output_limit", 0))
    controller_type = rospy.get_param("~controller_type", 'Normal')

    meassurement_topic = rospy.get_param("~meassurement_topic", "/pid_test/meassurement")
    setpoint_topic = rospy.get_param("~setpoint_topic", "/pid_test/setpoint")
    output_topic = rospy.get_param("~output_topic", "/pid_test/output")

    print("Meassurement topic " + meassurement_topic)
    print("setpoint topic " + setpoint_topic)
    print("Output topic " + output_topic)
    
    pid = PID(p_gain,i_gain,d_gain, output_limit)
    wrapper = PID_wrapper(pid, meassurement_topic, setpoint_topic, output_topic, controller_type)

    r = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        wrapper.update()
        r.sleep()