#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial
import time

def main():

    port_str = '/dev/ttyUSB1'
    baud = 115200

    rospy.init_node('serial_reader', anonymous=True)

    pub = rospy.Publisher('/lolo/sensors/ctd/raw', String, queue_size=2)
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        try:            
            ser = serial.Serial( port=port_str,baudrate=baud,timeout=0 )

            while not rospy.is_shutdown():
                line = ser.readline()
                line = line.decode()
                if(len(line) > 0):

                    pub.publish(line)
                r.sleep()

        except Exception as e:
            print('Failed to open port ' + str(port_str))
            print(e)
            time.sleep(2)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
