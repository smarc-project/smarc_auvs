SESSION=lolo_bringup


LATITUDE=58.811480
LONGITUDE=17.596177
UTM_ZONE=33
UTM_BAND=V
ORIGIN_OFFSET_NORTH=6521967.47 #Utm coordinates for the LAT/LON in zone 33V
ORIGIN_OFFSET_EAST=649947.08 

#TODO calculate UTM zone offset based on lat lon origin and utm zone and band
#<arg name="origin_latitude" default="58.811480"/>
#<arg name="origin_longitude" default="17.596177"/>


#Ip adress of the captain
CAPTAIN_IP=192.168.1.90
LOLO_IP=192.168.1.100

tmux -2 new-session -d -s $SESSION

tmux rename-window "roscore"
tmux send-keys "roscore" C-m

tmux new-window -n 'Hardware1'
tmux send-keys "sleep 3; roslaunch lolo_drivers lolo_core_hardware1.launch utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND captain_ip:=$CAPTAIN_IP LOLO_IP:=$LOLO_IP" C-m

tmux new-window -n 'Hardware2'
tmux send-keys "sleep 3; mon launch lolo_drivers lolo_core_hardware2.launch utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND captain_ip:=$CAPTAIN_IP LOLO_IP:=$LOLO_IP" C-m

tmux new-window -n 'tf'
tmux send-keys "sleep 5; roslaunch lolo_drivers lolo_core_tf.launch utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND map_origin_north:=$ORIGIN_OFFSET_NORTH map_origin_east:=$ORIGIN_OFFSET_EAST captain_ip:=$CAPTAIN_IP LOLO_IP:=$LOLO_IP" C-m

tmux new-window -n 'control'
tmux send-keys "sleep 5; roslaunch lolo_controllers control.launch" C-m

tmux new-window -n 'action'
tmux send-keys "sleep 5; roslaunch lolo_action_servers lolo_actions.launch robot_name:=lolo" C-m

tmux new-window -n 'bt'
tmux send-keys "sleep 8; roslaunch smarc_bt mission.launch robot_name:=lolo" C-m

#tmux new-window -n 'mbes_node'
#tmux send-keys "sleep 5; roslaunch r2sonic_mbes r2sonic_mbes.launch" C-m

#tmux new-window -n 'ptcloud_to_pcd'
#tmux send-keys "sleep 5; rosrun pcl_ros pointcloud_to_pcd input:=/lolo/mbes_pointcloud _prefix:=/xavier_ssd/LOGS/pcd/"

#tmux new-window -n 'bathy_node'
#tmux send-keys "sleep 5; roslaunch r2sonic_mbes r2sonic_bathy.launch"

#tmux new-window -n 'bathy_node'
#tmux send-keys "sleep 5; rosrun r2sonic_mbes image_array_node"

tmux new-window -n 'logging'
tmux send-keys "cd /xavier_ssd/LOGS/" C-m
tmux send-keys "sleep 5; rosbag record -a --split --duration=1800"

# Set default window
tmux select-window -t $SESSION:0


# Attach to session
tmux -2 attach-session -t $SESSION