# jackal-follow

# Overview
Two jackal robots in Gazebo with one robot following the other using LiDAR and path planning

Multi-Jackal Simulator using Gazebo ROS found from https://github.com/NicksSimulationsROS/multi_jackal

# Files
## multi_jackal_tutorials
The starting point for simulating the robots. Contains launch and config files.
Starts up a Gazebo session and launches robots using `multi_jackal_base`.

# Running
Make sure the file `multi_jackal_description/scripts/env_run` is executable.

`roslaunch multi_jackal_tutorials two_jackal.launch`

`python multi_jackal_tutorials/scripts/follow.py`

