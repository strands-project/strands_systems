#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'linda_core'
tmux new-window -t $SESSION:2 -n 'linda_robot'
tmux new-window -t $SESSION:3 -n 'linda_mapping'


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
tmux send-keys "roslaunch strands_linda linda_robot.launch"

tmux select-window -t $SESSION:3
tmux send-keys "roslaunch strands_linda linda_mapping_move_base.launch"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on
