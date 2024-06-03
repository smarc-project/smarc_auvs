#!/usr/bin/env python

import rospy
from std_msgs.msg import Char
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from geographic_msgs.msg import GeoPoint
from lolo_msgs.msg import UsblTopsideCorrection

# Useful USBL AT commands.
DROP_MSG = "+++ATZ4\r"
SET_PWR_LOW = "+++AT!L3\r"

# Device address for the local (Lolo) acoustic modem.
LOCAL_ADDRESS = 1
# Device address for the remote (topside) acoustic modem.
REMOTE_ADDRESS = 2

class UsblInterface:

    def __init__(self):

        self.command = ""

        # Important flags.
        self.usbl_in_air = True # Will assume it's always starting on air.
        self.incoming_correction = False

        self.cur_lat = None
        self.cur_lon = None
        self.cur_depth = None

        # Publisher for the transmit message.
        self.transmit_pub = rospy.Publisher(rospy.get_param("/usbl_transmit_topic",
                                                            "/lolo/core/usbl/transmit"),
                                            Char, queue_size=100)
        # Publisher for the abort message.
        self.abort_pub = rospy.Publisher(rospy.get_param("/abort_topic",
                                                        '/lolo/core/abort'),
                                         Empty, queue_size=1)
        # Publisher for the USBL correction sent by the topside unit.
        self.usbl_correction_pub = rospy.Publisher(rospy.get_param("/usbl_correction_topic",
                                                                   "/lolo/core/usbl/correction"),
                                                   UsblTopsideCorrection, queue_size=1)

       # Subscribe from the get-go.
        rospy.Subscriber(rospy.get_param("/usbl_received_topic",
                                        "/lolo/core/usbl/received"),
                        Char, self.usbl_callback)
        rospy.Subscriber(rospy.get_param("/lolo_position_topic",
                                         "/lolo/dr/lat_lon"),
                         GeoPoint, self.position_callback)
        rospy.Subscriber(rospy.get_param("/lolo_depth_topic",
                                         "/lolo/dr/depth"),
                         Float64, self.depth_callback)

        #self.configure_modem()

    def usbl_callback(self, msg):
        """
        Incrementally build the inbound command until a newline appears.
        """
        self.command += chr(msg.data)

        # If we get a newline, we assume we've got the whole command.
        if self.command[-1] == chr(10):
            #TODO: (debug).
            rospy.loginfo(self.command)
            if self.incoming_correction:
                # Publish the USBL correction for the INS to read.
                self.publish_usbl_correction(self.command)
                self.incoming_correction = False
            else:
                # Do something.
                success = self.usbl_menu(self.command[:-1])
            # Reset the command buffer.
            self.command = ""
            # TODO: Maybe let the topside unit you understood it or not?
            # self.usbl_transmit(success)

    def position_callback(self, msg):
        """
        Just store the lat/lon.
        """
        self.cur_lat = round(msg.latitude,6)
        self.cur_lon = round(msg.longitude,6)

    def depth_callback(self, msg):
        """
        Store depth, trigger usbl_in_air flag.
        """
        self.cur_depth = round(msg.data,6)

        # Trigger usbl_in_air flag if lolo is at the surface.
        if self.cur_depth >= 1.0:
            self.usbl_in_air = False
        else:
            self.usbl_in_air = True

    def configure_modem(self):
        """
        Configure the communication modem to comply with the necessary
        settings for burst message communication (has to be the same as
        the topside unit).
        """
        # Set the transmission level to the lowest to be safe.
        self.send_for_transmission("+++AT&F", is_command=True)
        rospy.loginfo("(UsblInterface) Configuring transmission to lowest power.")
        rospy.sleep(1.0)


    def usbl_menu(self, command):
        """
        Use the input command to do something useful.
        """

        if command.lower() == 'abort':
            self.abort_pub.publish(Empty())
            success = True
        elif command.lower() == 'pos':
            rospy.loginfo("(UsblInterface) Position requested by topside, sending...")
            self.relay_position()
            success = True
        elif command.lower() == 'usbl':
            rospy.loginfo("(UsblInterface) Receiving USBL correction...")
            self.incoming_correction = True
            success = True
        else:
            success = False

        return success

    def relay_position(self):
        """
        Relay the most recent position of LoLo in lat/lon [deg] and depth [m].
        """

        #rospy.sleep(10)
        msg = "POS " + str(self.cur_lat) + " " + str(self.cur_lon) + " " + str(self.cur_depth)
        rospy.loginfo("(UsblInterface) Relaying position to topside: {0}".format(msg))

        self.send_for_transmission(msg)

    def send_for_transmission(self, msg, is_command=False):
        """
        Send the input msg to the topside unit. Checks if lolo is at the
        surface before transmitting anything.
        """
        # Do not transmit if lolo is at the surface.
        if self.usbl_in_air and not is_command:
            rospy.logwarn("(UsblInterface) USBL in air! Won't transmit to topside!")
            return

        # Append a carriage return at the end of the msg.
        msg += "\r"
        msg += '\n'
        for c in msg:
            #print(c)
            #rospy.sleep(0.015)
            self.transmit_pub.publish(Char(ord(c)))

    def drop_message(self):
        """
        Send an ATZ4 command to the transducer to drop messages.
        """
        rospy.logwarn("(UsblInterface) In air, dropping messages...")
        for c in DROP_MSG:
            self.transmit_pub.publish(Char(ord(c)))

    def publish_usbl_correction(self, msg):
        """
        Publish the USBL's position and measured rangeso the INS can
        use it for correction.

        msg:
            type: string
            format: "usbl_lat usbl_lon usbl_range stamp"
        """
        # Split string by the whitespace between each variable.
        lat, lon, meas_range, stamp = [float(n) for n in msg.split(" ")]

        # Build message and publish.
        usbl_correction = UsblTopsideCorrection()
        usbl_correction.lat = lat
        usbl_correction.lon = lon
        usbl_correction.range = meas_range
        usbl_correction.stamp = stamp
        self.usbl_correction_pub.publish(usbl_correction)

def main():
    rospy.init_node("usbl_lolo_interface")

    # Only instantiating the class should be enough.
    interface = UsblInterface()

    # FIXME: What rate does this have to run?
    rate = rospy.Rate(50)
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now()

        # Drop message every 5 seconds if in air.
        if ((now - start).to_sec() > 5.0) and interface.usbl_in_air:
            interface.drop_message()
            start = rospy.Time.now()

        rate.sleep()


if __name__ == "__main__":
    main()