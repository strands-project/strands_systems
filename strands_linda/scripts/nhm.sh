#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'linda_core'
tmux new-window -t $SESSION:2 -n 'linda_robot'
tmux new-window -t $SESSION:3 -n 'linda_navigation_nhm'
tmux new-window -t $SESSION:4 -n 'linda_interfaces'
tmux new-window -t $SESSION:5 -n 'linda_people_perception'
tmux new-window -t $SESSION:6 -n 'patrol'
tmux new-window -t $SESSION:7 -n 'nhm'
tmux new-window -t $SESSION:8 -n 'RViz'


tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "roslaunch strands_linda linda_core.launch"

tmux select-window -t $SESSION:2
tmux send-keys "roslaunch strands_linda linda_robot_nhm.launch"

tmux select-window -t $SESSION:3
tmux send-keys "roslaunch strands_linda linda_navigation_nhm.launch topological_map:=WW_2014_Jan_19_human"

tmux select-window -t $SESSION:4
tmux send-keys "roslaunch strands_linda linda_interfaces.launch"

tmux select-window -t $SESSION:5
tmux send-keys "roslaunch strands_linda linda_people_perception.launch"

tmux select-window -t $SESSION:6
tmux send-keys "roslaunch topological_patroller patrol.launch"

tmux select-window -t $SESSION:7
tmux send-keys "roslaunch nhm nhm.launch dialogue_option:=nhm behaviour:=test2"

tmux select-window -t $SESSION:8
tmux send-keys "rosrun rviz rviz"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on
