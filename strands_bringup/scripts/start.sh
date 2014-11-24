#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'strands_core'
tmux new-window -t $SESSION:2 -n 'strands_robot'
tmux new-window -t $SESSION:3 -n 'strands_cameras'
tmux new-window -t $SESSION:4 -n 'strands_ui'
tmux new-window -t $SESSION:5 -n 'strands_navigation'
tmux new-window -t $SESSION:6 -n 'RViz'


tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "roslaunch strands_bringup strands_core.launch machine:=localhost user:=$USER db_path:=/opt/strands/mongodb_store port:=62345"

tmux select-window -t $SESSION:2
tmux send-keys "roslaunch strands_bringup strands_robot.launch machine:=localhost user:=$USER with_mux:=False js:=/dev/js1 laser:=/dev/ttyUSB0 scitos_config:=$(rospack find scitos_mira)/resources/SCITOSDriver.xml"

tmux select-window -t $SESSION:3
tmux send-keys "roslaunch strands_bringup strands_cameras.launch machine:=localhost	user:=$USER head_camera:=True head_ip:=localhost head_user:=$USER chest_camera:=True chest_ip:=localhost chest_user:=$USER"

tmux select-window -t $SESSION:4
tmux send-keys "roslaunch strands_bringup strands_ui.launch machine:=localhost user:=$USER"

tmux select-window -t $SESSION:5
tmux send-keys "roslaunch strands_bringup strands_navigation.launch machine:=localhost user:=$USER with_camera:=True camera:=chest_xtion camera_ip:=localhost camera_user:=$USER map:=/path/to/map.yaml with_no_go_map:=False no_go_map:=/path/to/no_go_map.yaml with_mux:=False topological_map:=name_of_topological_map"

tmux select-window -t $SESSION:9
tmux send-keys "rosrun rviz rviz"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on
