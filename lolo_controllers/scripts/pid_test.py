#! /usr/bin/env python3

#Test script for PID regulator
import rospy
from std_msgs.msg import Float64
from random import random

#system: Xn+1 = Xn + actuation*0.5
class dummy_system(object):
    def __init__(self):
        self.X = 0
        self.actuation = 0
        self.last_update = None
        self.actuation_sub   = rospy.Subscriber("/lolo/ctrl/pid_testing_output", Float64, self.actuation_cb, queue_size=1)
        self.meassurement_pub = rospy.Publisher("/lolo/ctrl/pid_testing_meassurement", Float64, queue_size=1)

    def update(self):
        now = rospy.get_time()
        if self.last_update is None:
            self.last_update = now
            return
        dt = now - self.last_update
        self.X = self.X + 0.5*self.actuation*dt
        self.last_update = now

        self.meassurement_pub.publish(self.X)

    def actuation_cb(self, msg):
        self.actuation = msg.data

        



if __name__ == '__main__':
    rospy.init_node('pid_tester')

    setpoint_pub = rospy.Publisher("/lolo/ctrl/pid_testing_setpoint", Float64, queue_size=1)

    s = dummy_system()
    r = rospy.Rate(10)
    setpoint = 1
    i=0
    while not rospy.is_shutdown():
        s.update()
        r.sleep()
        i+=1
        setpoint_pub.publish(setpoint)
        if i % 200 == 0:
            setpoint = random()*5


        






#system: 
# Xdotn+1 = Xdotn + actuation*0.5*dt
# Xn+1 = Xn + X_dot*dt