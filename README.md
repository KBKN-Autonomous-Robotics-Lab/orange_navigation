# orange_navigation
This branch provides the setup for using the waypoint editing application in ROS2.

## Setup
1.Create environment for waypoint_manager
```
$ cd waypoint_manager
$ python3 -m venv venv
$ source venv/bin/active
(venv) $ pip install -r requirements.txt
(venv) $ deactivate
```
2. With ubuntu, it is recommended to register for alias
```
$ echo "alias waypoint_manager='/home/ubuntu/ros2_ws/src/orange_navigation/waypoint_manager/run_app.sh'" >> ~/.bashrc
```
