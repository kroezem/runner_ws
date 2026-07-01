#!/usr/bin/env bash
# Runner bring-up sequencer — persistent tmux panes for the Phase 0 perception stack.
# Base (LiDAR + RF2O) auto-starts. SLAM is staged but launched manually so you watch it live.

SESSION=runner
SRC='source /opt/ros/jazzy/setup.bash; source ~/runner_ws/install/setup.bash'

# If the session already exists, just attach — don't double-launch nodes.
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Session '$SESSION' already running — attaching. (kill with: tmux kill-session -t $SESSION)"
  exec tmux attach -t "$SESSION"
fi

# ---- window 0: base (LiDAR | RF2O) ----
tmux new-session  -d -s "$SESSION" -n base

# left pane: LD19 driver
tmux send-keys -t "$SESSION":base.0 "$SRC; echo '[base/lidar] starting LD19...'; ros2 launch ldlidar_stl_ros2 ld19.launch.py" C-m

# right pane: RF2O, after the LiDAR has time to start publishing /scan
tmux split-window -h -t "$SESSION":base
tmux send-keys -t "$SESSION":base.1 "$SRC; echo '[base/rf2o] waiting 6s for /scan...'; sleep 6; ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py" C-m

# ---- window 1: slam (staged, NOT launched) ----
tmux new-window -t "$SESSION" -n slam
tmux send-keys -t "$SESSION":slam "$SRC" C-m
tmux send-keys -t "$SESSION":slam "clear; echo '[slam] base must be confirmed live first (see diag window).'" C-m
tmux send-keys -t "$SESSION":slam "echo '[slam] then press Up + Enter to launch and WATCH this pane.'" C-m
# Pre-load the launch command into history — recall with Up, do not auto-run.
tmux send-keys -t "$SESSION":slam "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false"

# ---- window 2: diag (free shell) ----
tmux new-window -t "$SESSION" -n diag
tmux send-keys -t "$SESSION":diag "$SRC" C-m
tmux send-keys -t "$SESSION":diag "clear; echo '[diag] base checks:  ros2 topic hz /scan   |   ros2 run tf2_ros tf2_echo odom base_link'" C-m

# Land on the base window so you see the stack come up.
tmux select-window -t "$SESSION":base
exec tmux attach -t "$SESSION"
