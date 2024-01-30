#!/usr/bin/env python3
import rospy
import teensy_ping_driver.srv
import rosparam
import configparser
import socket

cfg_dir = ""
cfg = ""

def send_schedule(req: teensy_ping_driver.srv.pingScheduleServiceRequest):
    cfg_dir = rosparam.get_param(rospy.get_name() + '/cfg_dir')
    cfg = rosparam.get_param(rospy.get_name() + '/cfg')
    ip = rosparam.get_param(rospy.get_name() + '/teensy_ip')
    port = rosparam.get_param(rospy.get_name() + '/teensy_port')
    inifile = cfg_dir+cfg

    print("Loading config file: " + inifile)
    config = configparser.ConfigParser(defaults=None, 
                                    delimiters=(':', '='), 
                                    comment_prefixes=('#', ';'),
                                    inline_comment_prefixes=('#'),
                                    interpolation=configparser.ExtendedInterpolation())#configparser.ExtendedInterpolation)
    config.read(inifile)

    #for section in config.sections():
    #    for key in config[section].keys():
    #        print(key + " " + config[section][key])  

    schedules = config['schedules']

    req_str = str(req.PingScheduleRequest)
    print("Request for schedule: " + req_str)
    
    if req_str in schedules.keys():
        print("Found schedule: " + schedules[req_str])
        b = bytes([int(s) for s in schedules[req_str].split()])
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.settimeout(2)
        print("Connecting to " + ip + " " + port)

        try:
            udp_socket.connect((ip, int(port)))
            print("UDP successfully connected.")
        except:
            print("Failed to bind socket.")
            return -1
        
        print("Transmitting")
        out = bytes('#', 'utf-8') + b + bytes('*', 'utf-8')
        print(out)
        udp_socket.send(out)

        teensy_response = udp_socket.recv(1024)
        print("Teensy reply: " + str(teensy_response))
        return teensy_ping_driver.srv.pingScheduleServiceResponse(schedules[req_str])
    else: 
        return teensy_ping_driver.srv.pingScheduleServiceResponse("schedule not found")

def main():
    print("Ping schedule service started")
    # Params: sonar ip, sonar port
    print(rospy.get_param_names())
    
    rospy.init_node('ping_schedule_server')
    configServ = rospy.Service('/ping_schedule_service', teensy_ping_driver.srv.pingScheduleService, send_schedule)
    rospy.spin()

if __name__ == "__main__":
    main()