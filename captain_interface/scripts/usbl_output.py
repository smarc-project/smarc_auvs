#!/usr/bin/env python

import rospy
from std_msgs.msg import Char
import sys

class USBL:
    def __init__(self):
        self.message = ""

    def callback(self, msg):
        sys.stdout.write(chr(msg.data))
        #self.message += chr(msg.data)
        #if(self.message[-1] == '\n'):
        #    sys.stdout.write(self.message)
        #    self.message = ""


def main():

    aa = USBL();

    rospy.init_node('USBL_output', anonymous=True)

    rospy.Subscriber('/lolo/core/usbl/received', Char, aa.callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
