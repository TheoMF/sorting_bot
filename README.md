# sorting_bot
Project to do selective sort with open source robots.

<img width="288" src="SO-101_pick_and_place.gif" />

## Installation

```bash
cd ~/ros2_ws
git clone git@github.com:TheoMF/sorting_bot.git src/sorting_bot
vcs import --shallow --recursive src < src/sorting_bot/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch

### Simulation
Simulation is launched with commands
```bash
ros2 launch so_100_arm gazebo.launch.py
ros2 launch sorting_bot detection_simulator.launch.py
ros2 launch sorting_bot joint_trajectory_publisher.launch.py
```

### Experiment
Experiment is launched with commands
```bash
ros2 launch so_100_arm hardware.launch.py
ros2 launch so_100_arm rviz.launch.py
ros2 launch sorting_bot apriltag_detection.launch.py
ros2 launch sorting_bot joint_trajectory_publisher.launch.py