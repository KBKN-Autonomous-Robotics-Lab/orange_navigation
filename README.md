# orange_navigation
This branch provides ROS2 humble packages to support waypoint navigation and a Python project for editing waypoints, mainly for Tsukuba Challenge.

## Setup
1. Clone this repository into src directory of ROS2 workspace
2. Install waypoint_navigation package dependencies
```
$ rosdep install -r -y -i --from-paths .
$ colcon build
```
3. Create environment for waypoint_manager
```
$ cd waypoint_manager
$ python3 -m venv venv
$ source venv/bin/active
(venv) $ pip install -r requirements.txt
(venv) $ deactivate
```
4. With ubuntu, it is recommended to register for alias
```
$ echo "alias waypoint_manager='path/to/orange_navigation/waypoint_manager/run_app.sh'" >> ~/.bashrc
```
