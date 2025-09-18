# sorting_bot
Project to do selective sort with open source robots.

## Installation

```bash
cd ~/ros2_ws
git clone git@github.com:TheoMF/sorting_bot.git src/sorting_bot
vcs import --shallow --recursive src < src/sorting_bot/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```