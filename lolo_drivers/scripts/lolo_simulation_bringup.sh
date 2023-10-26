SESSION=lolo_bringup

#Settings
SIMULATION_RATE=300
GFX_QUALITY="low" # high/medium/low
#SCENARIO="biograd_world"
SCENARIO="kristineberg_world"
#SCENARIO="asko_world"

# ADD other environments to the list here
# the initial position of the robots are defined in the scenario files
case "$SCENARIO" in
	"asko_world")
		# asko 
		UTM_ZONE=33
		UTM_BAND=V
		LATITUDE=58.811480
		LONGITUDE=17.596177
		ORIGIN_OFFSET_NORTH=6521967.47
		ORIGIN_OFFSET_EAST=649947.08 
		;;
	"biograd_world")
		# Biograd
		UTM_ZONE=33
		UTM_BAND=T
		LATITUDE=43.93183
		LONGITUDE=15.44264
		ORIGIN_OFFSET_NORTH=4864396.61
		ORIGIN_OFFSET_EAST=535528.89
		;;
	"algae_world")
        # Algae farm (2 long ropes)
		UTM_ZONE=32
		UTM_BAND=V
		LATITUDE=58.250833
		LONGITUDE=11.450283
		ORIGIN_OFFSET_NORTH=6459252.08
		ORIGIN_OFFSET_EAST=643800.54
		;;
	"kristineberg_world")
		UTM_ZONE=32
		UTM_BAND=V
		LATITUDE=58.249721
		LONGITUDE=11.44624
		ORIGIN_OFFSET_NORTH=6459119.70
		ORIGIN_OFFSET_EAST=643567.83
		;;
	*)
		echo "UNKNOWN SCENARIO!"
		exit 1
esac

SIM_PKG_PATH="$(rospack find lolo_stonefish_sim)"
SMARC_STONEFISH_WORLDS_PATH="$(rospack find smarc_stonefish_worlds)"

#SCENARIO_DESC=$SAM_STONEFISH_SIM_PATH/data/scenarios/"$SCENARIO".scn
#CONFIG_FILE="${SAM_STONEFISH_SIM_PATH}/config/${SCENARIO}.yaml"
WORLD_CONFIG="${SCENARIO}.yaml"
WORLD_CONFIG_FILE="${SMARC_STONEFISH_WORLDS_PATH}/config/${WORLD_CONFIG}"
ROBOT_CONFIG="lolo.yaml"
ROBOT_CONFIG_FILE="${SIM_PKG_PATH}/config/${ROBOT_CONFIG}"
SCENARIO_DESC="${SMARC_STONEFISH_WORLDS_PATH}/data/scenarios/default.scn"

# Lidingo
#UTM_ZONE=34
#UTM_BAND=V

# Kristineberg
#UTM_ZONE=32
#UTM_BAND=V

# Rest of Sweden
#UTM_ZONE=33
#UTM_BAND=V

#Ip adress of the captain
#CAPTAIN_IP=192.168.1.90
#LOLO_IP=192.168.1.100

tmux -2 new-session -d -s $SESSION

tmux rename-window "roscore"
tmux send-keys "roscore" C-m

#tmux new-window -n 'Hardware1'
#tmux send-keys "sleep 3; roslaunch lolo_drivers lolo_core_hardware1.launch utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND captain_ip:=$CAPTAIN_IP LOLO_IP:=$LOLO_IP" C-m

#tmux new-window -n 'Hardware2'
#tmux send-keys "sleep 3; roslaunch lolo_drivers lolo_core_hardware2.launch utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND captain_ip:=$CAPTAIN_IP LOLO_IP:=$LOLO_IP" C-m

tmux new-window -n 'simulator'
tmux send-keys "sleep 1; mon launch lolo_stonefish_sim simulator.launch robot_config_file:=$ROBOT_CONFIG_FILE world_config_file:=$WORLD_CONFIG_FILE scenario_description:=$SCENARIO_DESC simulation_rate:=$SIMULATION_RATE graphics_quality:=$GFX_QUALITY --name=$(tmux display-message -p 'p#I_#W') " C-m

tmux new-window -n 'simulated hardware 1'
tmux send-keys "sleep 5; mon launch lolo_stonefish_sim robot_bridge.launch robot_name:=lolo origin_latitude:=$LATITUDE origin_longitude:=$LONGITUDE" c-m

tmux new-window -n 'tf'
tmux send-keys "sleep 5; roslaunch lolo_drivers lolo_core_tf.launch utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND map_origin_north:=$ORIGIN_OFFSET_NORTH map_origin_east:=$ORIGIN_OFFSET_EAST captain_ip:=$CAPTAIN_IP LOLO_IP:=$LOLO_IP" C-m

tmux new-window -n 'action'
tmux send-keys "sleep 5; roslaunch lolo_action_servers lolo_actions.launch robot_name:=lolo" C-m

#tmux new-window -n 'bt'
#tmux send-keys "sleep 8; roslaunch smarc_bt mission.launch robot_name:=lolo" C-m

#tmux new-window -n 'mbes_node'
#tmux send-keys "sleep 5; roslaunch r2sonic_mbes r2sonic_mbes.launch" C-m

#tmux new-window -n 'ptcloud_to_pcd'
#tmux send-keys "sleep 5; rosrun pcl_ros pointcloud_to_pcd input:=/lolo/mbes_pointcloud _prefix:=/xavier_ssd/LOGS/pcd/"

#tmux new-window -n 'bathy_node'
#tmux send-keys "sleep 5; roslaunch r2sonic_mbes r2sonic_bathy.launch"

#tmux new-window -n 'bathy_node'
#tmux send-keys "sleep 5; rosrun r2sonic_mbes image_array_node"

# tmux new-window -n 'logging'
# tmux send-keys "cd /xavier_ssd/LOGS/" C-m
# tmux send-keys "sleep 5; rosbag record -a --split --duration=1800"

# Set default window
tmux select-window -t $SESSION:0


# Attach to session
tmux -2 attach-session -t $SESSION
