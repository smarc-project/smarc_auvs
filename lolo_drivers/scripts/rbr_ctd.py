#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys
from lolo_msgs.msg import RBR_ctd


class CTDdecoder():

    def __init__(self, input_topic, output_topic):
        rospy.init_node('rbr_ctd_decoder', anonymous=True)
        self.publisher = rospy.Publisher(output_topic, RBR_ctd, queue_size= 10)
        self.subscriber = rospy.Subscriber(input_topic, String, self.callback)

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        msg = str(data.data)
        msg = msg.strip('\r\n')
        fields = msg.split(',')
        if len(fields) != 9:
            print("something is wrong with data: " + str(len(fields)))
            return
        sample = {}
        sample['time'] = fields[0]
        sample['conductivity'] = float(fields[1])
        sample['temperature'] = float(fields[2])
        sample['pressure'] = float(fields[3])
        sample['sea_pressure'] = float(fields[4])
        sample['depth'] = float(fields[5])
        sample['sailinity'] = float(fields[6])
        sample['samples'] = float(fields[7])
        sample['temperature_conductivity_correction'] = float(fields[8])

        msg = RBR_ctd()
        msg.time = sample['time']
        msg.conductivity = sample['conductivity']
        msg.temperature = sample['temperature']
        msg.pressure = sample['pressure']
        msg.sea_pressure = sample['sea_pressure']
        msg.depth = sample['depth']
        msg.sailinity = sample['sailinity']
        msg.samples = sample['samples']
        msg.temperature_conductivity_correction = sample['temperature_conductivity_correction']
        self.publisher.publish(msg)

        print(sample)


def main():
    
    decoder = CTDdecoder('/lolo/core/ctd/raw', "/lolo/core/ctd")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
