#!/usr/bin/env python

import rospy
from std_msgs.msg import Char
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from geographic_msgs.msg import GeoPoint
from lolo_msgs.msg import UsblTopsideCorrection
from lolo_msgs.msg import CustomCommand
from geometry_msgs.msg import PoseStamped

# For UTC time.
import time

# Useful USBL AT commands.
DROP_MSG = "+++ATZ4\r"
SET_PWR_LOW = "+++AT!L3\r"

# Device address for the local (Lolo) acoustic modem.
LOCAL_ADDRESS = 1
# Device address for the remote (topside) acoustic modem.
REMOTE_ADDRESS = 2

# Device address for the local (Lolo) acoustic modem.
LOCAL_ADDRESS = 2
# Device address for the remote (topside) acoustic modem.
REMOTE_ADDRESS = 1

# Newline in ASCII.
NEWLINE = 10

# TODO: Max length for the command buffer, assuming 1000 chars is too long.
CMD_MAX_LEN = 500

# In air depth threshold in [m]
IN_AIR_DEPTH = 1.0

# Length of the checksum.
CHECKSUM_LEN = 3

class UsblInterface:

    def __init__(self):

        self.command = ""

        # Important flags.
        self.usbl_in_air = True # Will assume it's always starting on air.

        self.cur_lat = None
        self.cur_lon = None
        self.cur_depth = None

        self.topside_frame_id = rospy.get_param("/usbl_topside_frame",
                                                "/usbl_topside_enu")

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
                                                   UsblTopsideCorrection, queue_size=10)
        # Publisher for a custom command from the topside unit.
        self.custom_cmd_pub = rospy.Publisher(rospy.get_param("/custom_command_topic",
                                                              "/lolo/custom_cmd"),
                                              CustomCommand, queue_size=10)
        # Publisher for the measured relative position from the topside unit.
        self.usbl_relative_pub = rospy.Publisher(rospy.get_param("/usbl_relative_topic",
                                                                 "/lolo/core/usbl/relative_pos"),
                                                 PoseStamped, queue_size=10)

        # ------ STATUS SUBSCRIBERS ----------
        # TODO: Subscribe to a bunch (or a custom) status topics that include relevant
        # info to send to the topside modem.
        # ------------------------- ----------

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

        # FIXME: self.configure_modem()

    def usbl_callback(self, msg):
        """
        Incrementally build the inbound command until a newline appears.
        """
        self.command += chr(msg.data)

        # If we get a newline, we assume we've got the whole command.
        if self.command[-1] == chr(NEWLINE):

            rospy.loginfo("(UsblInterface) Got command {0} from topside modem.".format(self.command))
            success = self.usbl_menu(self.command[:-1])
            if not success:
                rospy.logwarn("(UsblInterface) Did not understand command {0}.".format(self.command))
            # Reset the command buffer.
            self.command = ""

        if len(self.command) > CMD_MAX_LEN:
            rospy.logwarn("(UsblInterface) Newline hasn't been received, dropping buffer.")
            self.command = ""

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
        if self.cur_depth >= IN_AIR_DEPTH:
            self.usbl_in_air = False
        else:
            self.usbl_in_air = True

    def configure_modem(self):
        # FIXME: We're not using it ATM, we have to test it more,
        # last time it broke the communication between devices...
        """
        Configure the communication modem to comply with the necessary
        settings for burst message communication (has to be the same as
        the topside unit).
        """
        # Set the transmission level to the lowest to be safe.
        self.send_for_transmission("+++AT!L3", is_command=True)
        rospy.loginfo("(UsblInterface) Configuring transmission to lowest power.")
        rospy.sleep(1.0)

        # Set the receiving gain to normal.
        self.send_for_transmission("+++AT!G0", is_command=True)
        rospy.loginfo("(UsblInterface) Setting reciever gain to normal.")
        rospy.sleep(1.0)

        # Set the local and remote device addresses for lolo (1) and the topside (2).
        self.send_for_transmission("+++AT!AL2", is_command=True)
        rospy.loginfo("(UsblInterface) Setting local address to 2.")
        rospy.sleep(1.0)
        self.send_for_transmission("+++AT!AR1", is_command=True)
        rospy.loginfo("(UsblInterface) Setting remote address to 1.")
        rospy.sleep(1.0)

        # Set the packet transmission and reception time. Setting it to max (1200ms).
        self.send_for_transmission("+++AT!ZP1200", is_command=True)
        rospy.loginfo("(UsblInterface) Setting max packet transmission time to MAX (1200ms).")
        rospy.sleep(1.0)

    def usbl_menu(self, msg):

        """
        Use the input command to do something useful.
        You can edit this function to include more features.

        msg:
            type: string
            format: "CMD x x x x ..."
        """
        arg_list = [m for m in msg.split(" ")]
        command = arg_list[0]

        if command.lower() == 'abort':
            rospy.logwarn("(UsblInterface) ABORT ABORT ABORT!!")
            self.abort_pub.publish(Empty())
            success = True
        elif command.lower() == 'pos':
            rospy.loginfo("(UsblInterface) Position requested by topside, sending...")
            self.relay_position()
            success = True
        elif command.lower() == 'stat':
            rospy.loginfo("(UsblInterface) Status requested by topside, sending...")
            self.relay_status()
            success = True
        elif command.lower() == 'cor':
            rospy.loginfo("(UsblInterface) Receiving USBL correction...")
            self.publish_usbl_correction(msg)
            success = True
        elif command.lower() == 'runcmd':
            rospy.loginfo("(UsblInterface) Getting custom command from topside...")
            self.publish_custom_cmd(msg)
            success = True
        elif command.lower() == 'rel':
            rospy.loginfo("(UsblInterface) Receiving a relative position from topside...")
            self.publish_relative_position(msg)
            success = True
        else:
            success = False

        return success

    def relay_status(self):
        """
        TODO: Relay the most recent status update.
        """
        msg = "STAT " + str(self.cur_lat) + " " + str(self.cur_lon) + " " + str(self.cur_depth)
        rospy.loginfo("(UsblInterface) Relaying status to topside: {0}".format(msg))

        self.send_for_transmission(msg)

    def relay_position(self):
        """
        Relay the most recent position of LoLo in lat/lon [deg] and depth [m].
        """

        #rospy.sleep(10)
        msg = "POS " + str(self.cur_lat) + " " + str(self.cur_lon) + " " + str(self.cur_depth)
        rospy.loginfo("(UsblInterface) Relaying position to topside: {0}".format(msg))

        self.send_for_transmission(msg)

    def publish_usbl_correction(self, msg):
        """
        Publish the USBL's position and measured rangeso the INS can
        use it for correction.

        msg:
            type: string
            format: "COR usbl_lat usbl_lon usbl_range stamp"
        """
        # Split string by the whitespace between each variable.
        try:
            cmd, lat, lon, meas_range, stamp = [m for m in msg.split(" ")]
        except:
            rospy.logerr("(UsblInterface) Too many values to unpack, message is wrong.")
            self.command = ""
            return

        # Build message and publish.
        usbl_correction = UsblTopsideCorrection()
        usbl_correction.lat = float(lat)
        usbl_correction.lon = float(lon)
        usbl_correction.range = float(meas_range)
        usbl_correction.stamp = float(stamp)
        self.usbl_correction_pub.publish(usbl_correction)

    def publish_custom_cmd(self, msg):
        """
        Publish the custom commands as a CustomCommand msg
        for other nodes to do something with it.

        msg:
            type: string
            format: "cmd customcmd sent_stamp checksum"
        """
        # Split string by the whitespace between each variable.
        command = [m for m in msg.split(" ")]

        # Checksum will be the sum of the bytes in the msg without the checksum.
        bytelist = [ord(c) for c in msg[:-CHECKSUM_LEN]]
        checksum = hex(sum(bytearray(bytelist)) % 256)[2:]
        if command[-1] != checksum:
            rospy.logerr("(UsblInterface) Custom cmd checksum failed! Ignoring.")
            rospy.logerr("(UsblInterface) Expected {0} and got {1}.".format(command[-1], checksum))
            return

        custom_msg = CustomCommand()
        custom_msg.cmd = command[1]
        custom_msg.sent_stamp = int(command[2])
        # Stamp it with UTC time in [s].
        custom_msg.recv_stamp = int(time.time())

        self.custom_cmd_pub.publish(custom_msg)

    def publish_relative_position(self, msg):
        """
        Publish the relayed relative position of Lolo with respect to the
        topside unit, in ENU convention.

        msg:
            type: string
            format: "REL East North Up stamp"
        """
        # Split string by the whitespace between each variable.
        try:
            cmd, east, north, up, stamp = [m for m in msg.split(" ")]
        except:
            rospy.logerr("(UsblInterface) Too many values to unpack, message is wrong.")
            self.command = ""
            return

        pos_msg = PoseStamped()
        # TODO: will the fact that the stamp is in the past affect ros in a way?
        pos_msg.header.stamp = rospy.Duration(float(stamp))
        pos_msg.header.frame_id = self.topside_frame_id
        pos_msg.pose.position.x = float(east)
        pos_msg.pose.position.y = float(north)
        pos_msg.pose.position.z = float(up)
        self.usbl_relative_pub.publish(pos_msg)

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
