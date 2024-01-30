#!/usr/bin/env python3
import rospy
import teensy_ping_driver.srv
import rosparam
import sys

def main():
    print("Test client started")
    rospy.init_node('ping_schedule_testclient')
    configService = rospy.ServiceProxy('/ping_schedule_service', teensy_ping_driver.srv.pingScheduleService)

    print("Waiting for service")
    rospy.wait_for_service('/ping_schedule_service', timeout=None)

    print("Calling service with config: " + str(sys.argv[1]))
    rsp = configService(int(sys.argv[1]))
    print("Service response: " + rsp.PingScheduleResponse)

    #rospy.spin()

if __name__ == "__main__":
    main()