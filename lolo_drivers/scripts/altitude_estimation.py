#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

import rospy
from nortek_dvl333_driver.msg import BottomTrack
from norbit_wbms_driver.msg import Bathymetry
from std_msgs.msg import Float32, Float64
import math

class AltitudeEstimation():

    def __init__(self,altitude_topic, dvl_topic, mbes_topic, fls_topic, pitch_topic, roll_topic):
        #Altitude sources
        self.dvl_sub = rospy.Subscriber(dvl_topic, BottomTrack, self.dvl_cb, queue_size=1)
        self.mbes_sub = rospy.Subscriber(mbes_topic, Bathymetry, self.mbes_cb, queue_size=1)
        #self.fls_sub = rospy.Subscriber("/lolo/sensors/mbes/bathy/data", Bathymetry, dvl_cb, queue_size=1)

        #Vehicle attitude
        self.pitch_sub = rospy.Subscriber(pitch_topic, Float64, self.pitch_cb)
        self.roll_sub = rospy.Subscriber(roll_topic, Float64, self.roll_cb)

        self.altitude_pub = rospy.Publisher(altitude_topic, Float32, queue_size=1)
        
        self.roll = 0
        self.pitch = 0

        self.dvl_altitude = None
        self.mbes_altitude = None
        self.fls_altitude = None

        self.last_dvl = 0
        self.last_mbes = 0
        self.last_fls = 0

    def pitch_cb(self, msg):
        self.pitch = msg.data

    def roll_cb(self, msg):
        self.roll = msg.data

    def dvl_cb(self, msg: BottomTrack) -> None:
        #Check if dvl data is valid
        beam_sum = 0
        valid_beams = 0

        beamlist = [msg.distBeam0,
                    msg.distBeam1,
                    msg.distBeam2,
                    msg.distBeam3]
        
        for beamRange in beamlist:
            if beamRange > 0.3 and beamRange < 400:
                beam_sum+=beamRange
                valid_beams+=1

        if(valid_beams > 0):
            self.last_dvl = rospy.Time.now()
            self.dvl_altitude = beam_sum / valid_beams
            print("dvl altitude valid")

            #Compensate for pitch (Should be done in a better way...)
            pitch_compensation = self.dvl_altitude*math.sin(self.pitch)
            print("dvl pitch compensation: " + str(pitch_compensation))
            self.dvl_altitude = self.dvl_altitude-pitch_compensation
        else:
            self.dvl_altitude = None

    def mbes_cb(self, msg):
        #Find nadir beam and use that as altitude
        print("MBES altitude")

        #Sort the beams
        beams = msg.beams
        beams.sort(key=lambda x: abs(self.roll - x.angle))

        #Take the median of the most "nadir" beams as altitude
        nadir_beams = beams[:5]
        nadir_beams.sort(key=lambda x: x.range)
        nadir_range = nadir_beams[3].range
        
        #print("Mbes altitude received")
        self.last_mbes = rospy.Time.now()
        self.mbes_altitude = nadir_range
            
        #Compensate for pitch
        pitch_compensation = self.mbes_altitude*math.sin(self.pitch)
        print("mbes pitch compensation: " + str(pitch_compensation))
        self.mbes_altitude = self.mbes_altitude-pitch_compensation

    def fls_cb(self,msg):
        pass

    def publish_altitude(self):
        alt_sum = 0
        valid_est = 0
        if self.dvl_altitude != None and rospy.Time.now() - self.last_dvl < rospy.Duration(2):
            alt_sum+=self.dvl_altitude
            valid_est+=1
        if self.mbes_altitude != None and rospy.Time.now() - self.last_mbes < rospy.Duration(1):
            alt_sum+=self.mbes_altitude
            valid_est+=1
            
        if(valid_est > 0):
            altitude = alt_sum / valid_est
            self.altitude_pub.publish(altitude)

            

        


if __name__ == "__main__":
    rospy.init_node("altitude_estimator")

    altitude_topic  = "/lolo/dr/altitude_new"
    dvl_topic = "/dvl/bottomtrack" #"/lolo/sensors/dvl/bottomtrack/data"
    mbes_topic = "/lolo/sensors/mbes/bathymetry"
    fls_topic = "unclear"
    pitch_topic = "/lolo/dr/pitch"
    roll_topic = "/lolo/dr/roll"

    estimator = AltitudeEstimation(altitude_topic=altitude_topic, dvl_topic=dvl_topic, mbes_topic=mbes_topic, fls_topic=fls_topic, pitch_topic=pitch_topic, roll_topic=roll_topic)

    r =rospy.Rate(1)
    while not rospy.is_shutdown():
        estimator.publish_altitude()
        r.sleep()
