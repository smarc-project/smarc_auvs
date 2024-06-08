# This message is used to send a custom command to the vehicle
# through an acoustic communication link.

# Time stamped by the topside modem when sending the command.
# TODO: which convention should it have?
float32 sent_stamp
# Time stamped by the vehicle when the message was recieved.
# TODO: which convention should it have?
float32 recv_stamp
# Command sent by the topside unit.
string cmd
# Arguments following the command.
string arg1
string arg2
string arg3
string arg4
string arg5
