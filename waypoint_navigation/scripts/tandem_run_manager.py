#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
import ruamel.yaml
import numpy as np
from std_msgs.msg import UInt16
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


NODE_NAME = "tandem_run_manager"


class TandemManager():

    def __init__(self):
        ## Read waypoints file
        yaml = ruamel.yaml.YAML()
        waypoints_path = rospy.get_param(NODE_NAME+"/waypoints_file")
        with open(waypoints_path) as file:
            waypoints_yaml = yaml.load(file)
        ## Register waypoint number of start/end tandem area 
        self.tandem_start_list = []
        self.tandem_end_list = []
        self.tandem_id = 0
        for i, data in enumerate(waypoints_yaml["waypoints"]):
            if ("tandem_start" in data["point"]):
                self.tandem_start_list.append(i+2)
            if ("tandem_end" in data["point"]):
                self.tandem_end_list.append(i+2)
        ## Waypoint navigation service clients
        self.stop_nav = rospy.ServiceProxy("/stop_wp_nav", Trigger)
        self.resume_nav = rospy.ServiceProxy("/resume_nav", Trigger)
        ## Dynamic reconfigure clients
        costmap_names = rospy.get_param(NODE_NAME+"/switch_costmap", [])
        self.clients = []
        for name in costmap_names:
            self.clients.append(dynamic_reconfigure.client.Client(name))
        ## Variable
        self.front_angle = rospy.get_param(NODE_NAME+"/use_angle", 20) # degree
        self.danger_dist = rospy.get_param(NODE_NAME+"/danger_dist", 1.0) # meter
        self.waypoint_num = 0
        self.front_range = None
        self.in_tandem_area = False
        self.stop = False
        ## Subscribers
        self.wp_num_sub = rospy.Subscriber("/waypoint_num", UInt16, self.waypoint_num_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.laserscan_callback)
        return


    ## Subscribe current waypoint number
    def waypoint_num_callback(self, msg):
        if (msg.data == self.waypoint_num) or (self.stop): return
        if (self.waypoint_num == 0) and (msg.data > 1):
            for i in range(0, len(self.tandem_start_list)):
                if (msg.data >= self.tandem_end_list[i]):
                    self.tandem_id += 1
                    continue
                if (msg.data >= self.tandem_start_list[i]) and (msg.data < self.tandem_end_list[i]):
                    self.tandem_id = i
                    self.in_tandem_area = True
                    self.update_costmap_config(False)
                    rospy.loginfo("Enter tandem area.")
                    break
            self.waypoint_num = msg.data
            return
        
        if (msg.data == self.tandem_start_list[self.tandem_id]):
            self.in_tandem_area = True
            self.update_costmap_config(False)
            rospy.loginfo("Enter tandem area.")
        
        elif (msg.data == self.tandem_end_list[self.tandem_id]):
            self.in_tandem_area = False
            self.update_costmap_config(True)
            self.tandem_id += 1
            self.stop = (self.tandem_id >= len(self.tandem_start_list))  # if True, self.stop will never be False
            rospy.loginfo("Exit from tandem area.")
        
        self.waypoint_num = msg.data
        return
    

    def laserscan_callback(self, msg):
        try:
            if self.front_range is None:
                front = round(-msg.angle_min / msg.angle_increment)
                ran = int(round(np.deg2rad(self.front_angle/2) / msg.angle_increment))
                self.front_range = [front-ran, front+ran]
                return
            
            if not self.in_tandem_area: return
            ranges = np.array(msg.ranges[self.front_range[0]:self.front_range[1]])
            ## Use simply minimum
            ranges[ranges <= msg.range_min] = msg.range_max
            min_range = min(ranges)
            ## or use sort
            #sort_ranges = np.sort(ranges)
            #min_range = np.mean(sort_ranges[:5])
            if (not self.stop) and (min_range < self.danger_dist):
                self.stop_nav()
                self.stop = True
                rospy.loginfo("Stop because of obstacle within {}m ahead.".format(self.danger_dist))
            
            elif (self.stop) and (min_range >= self.danger_dist+0.1):
                self.resume_nav()
                self.stop = False
                rospy.loginfo("Resumed navigation because the obstacle ahead was more than {}m away.".format(self.danger_dist+0.1))
        
        except AttributeError:
            pass
        return
    
    
    def update_costmap_config(self, enable: bool):
        if len(self.clients) > 0:
            for client in self.clients:
                client.update_configuration({"enabled": enable})
        return




if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    tandem_manager = TandemManager()
    if (len(tandem_manager.tandem_start_list) == 0):
        rospy.logwarn("Shut down " + NODE_NAME)
    else:
        rospy.spin()
