#!/bin/bash

SESSION_NAME="HE-Nav"

# Create a new tmux session
tmux new-session -d -s "$SESSION_NAME"

# Execute the commands in each window
tmux send-keys -t "$SESSION_NAME:0" 'source ~/HE-Nav/devel/setup.bash; roslaunch ego_planner simple_run.launch' C-m
tmux new-window -t "$SESSION_NAME" 'sleep 1; source ~/HE-Nav/devel/setup.bash; roslaunch ego_planner rviz.launch'
tmux new-window -t "$SESSION_NAME" 'sleep 5; source ~/HE-Nav/devel/setup.bash; roslaunch perception pointcloud_listener.launch'
tmux new-window -t "$SESSION_NAME" 'sleep 5; source ~/HE-Nav/devel/setup.bash; roslaunch perception inference.launch'
# tmux new-window -t "$SESSION_NAME" 'sleep 2;rostopic echo /non_intersection_coordinates'

# Attach to the tmux session
tmux attach-session -t "$SESSION_NAME"