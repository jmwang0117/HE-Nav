#!/bin/bash

SESSION_NAME="HE-Nav"

# Create a new tmux session
tmux new-session -d -s "$SESSION_NAME"

# Execute the commands in each window
tmux send-keys -t "$SESSION_NAME:0" 'source ~/HE-Nav/devel/setup.bash; roslaunch ego_planner single_run_in_sim.launch' C-m
tmux new-window -t "$SESSION_NAME" 'sleep 2; source ~/HE-Nav/devel/setup.bash; roslaunch ego_planner rviz.launch'
# tmux new-window -t "$SESSION_NAME" 'sleep 2; source ~/EH-Nav/devel/setup.bash; roslaunch ego_planner pointcloud_listener.launch'
# tmux new-window -t "$SESSION_NAME" 'sleep 2; source ~/EH-Nav/devel/setup.bash; roslaunch perception inference.launch'
# tmux new-window -t "$SESSION_NAME" 'sleep 2;rostopic echo /planning/pos_cmd'

# Attach to the tmux session
tmux attach-session -t "$SESSION_NAME"
