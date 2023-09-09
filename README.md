# orange_navigation
This repository provides ROS packages to support waypoint navigation and a Python project for editing waypoints, mainly for Tsukuba Challenge.

## Setup
1. Clone this repository into src directory of ROS workspace
2. Install waypoint_navigation package dependencies
```
$ rosdep install -r -y -i --from-paths .
$ catkin build
```
3. Create environment for waypoint_manager
```
$ cd waypoint_manager
$ python3 -m venv venv
$ source venv/bin/active
(venv) $ pip install -r requirements.txt
(venv) $ deactivate
```
