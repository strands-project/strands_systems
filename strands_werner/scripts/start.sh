#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'werner_core'
tmux new-window -t $SESSION:2 -n 'werner_robot'
tmux new-window -t $SESSION:3 -n 'werner_navigation'
tmux new-window -t $SESSION:4 -n 'werner_recognition'
tmux new-window -t $SESSION:5 -n 'werner_people'
tmux new-window -t $SESSION:6 -n 'werner_hri'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "HOSTNAME=werner roslaunch strands_werner werner_core.launch"

tmux select-window -t $SESSION:2
tmux send-keys "roslaunch strands_werner werner_robot.launch"

tmux select-window -t $SESSION:3
tmux send-keys "roslaunch strands_werner werner_navigation.launch"

tmux select-window -t $SESSION:4
tmux send-keys "roslaunch strands_werner werner_recognition_perception.launch"

tmux select-window -t $SESSION:5
tmux send-keys "roslaunch strands_werner werner_people_perception.launch"

tmux select-window -t $SESSION:6
tmux send-keys "roslaunch strands_werner werner_hri.launch"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on
