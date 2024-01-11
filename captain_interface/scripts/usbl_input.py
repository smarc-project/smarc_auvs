#!/usr/bin/env python

import rospy
from std_msgs.msg import Char

def main():
    pub = rospy.Publisher('/lolo/core/usbl/transmit', Char, queue_size=400)
    rospy.init_node('menuWriter', anonymous=True)
    while not rospy.is_shutdown():
        try:
            cmd = input("Write command: ")
            if(cmd == "EXIT"): break
            cmd += '\r'
            for c in cmd:
                msg = Char()
                msg.data = ord(c)
                pub.publish(msg)
        except:
            pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
