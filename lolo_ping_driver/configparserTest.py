
import configparser
inifile = "cfg/ping_schedules.ini"
config = configparser.ConfigParser(defaults=None, 
                                delimiters=(':', '='), 
                                comment_prefixes=('#', ';'),
                                inline_comment_prefixes=('#'),
                                interpolation=configparser.ExtendedInterpolation())#configparser.ExtendedInterpolation)
config.read(inifile)

rsp = 99

for section in config.sections():
    for key in config[section].keys():
        print(key + " " + config[section][key])  

schedules = config['schedules']

print("Schedule 2: " + schedules['2'])

b = bytes([int(s) for s in schedules['2'].split()])
print(b)

