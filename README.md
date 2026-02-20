# sorting_bot
Project to do selective sort with open source robots.

<img width="288" src="SO-101_pick_and_place.gif" />  

Demos of the project are available for mobile manipulator robot LeKiwi [here](https://youtu.be/8z1jm2o3kwE?si=UkZ776oeIiujN0lV) and for manipulator arm robot SO-101 [here](https://youtu.be/Cu2WPRdQeTQ?si=pOZc8sWhSyopvORJ).

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
# Simulation for SO-101
ros2 launch so_100_arm gazebo.launch.py gz_headless:=false

# Simulation for Le Kiwi
ros2 launch lekiwi lekiwi_full.launch.py use_rviz:=true use_fake_hardware:=true

# Remaining nodes to launch
ros2 launch sorting_bot detection_simulator.launch.py
ros2 launch sorting_bot joint_trajectory_publisher.launch.py robot_name:=<robot_name>
```

### Experiment
Experiment is launched with commands
```bash
# Experiment on SO-101
ros2 launch so_100_arm hardware.launch.py

# Experiment on LeKiwi
ros2 launch lekiwi lekiwi_full.launch.py use_rviz:=false use_fake_hardware:=false

# Remaining nodes to launch
ros2 launch sorting_bot apriltag_detection.launch.py camera:=base
ros2 launch sorting_bot apriltag_detection.launch.py camera:=hand
ros2 launch sorting_bot joint_trajectory_publisher.launch.py robot_name:=<robot_name>
