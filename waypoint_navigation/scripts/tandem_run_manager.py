#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import UInt16
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
import numpy as np
import ruamel.yaml


NODE_NAME = "tandem_run_manager"


class TandemManager(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        ## Parameters
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("switch_costmap.node_names", [])
        self.declare_parameter("switch_costmap.layer_names", [])
        self.declare_parameter("use_angle", 20)  # degree
        self.declare_parameter("danger_dist", 1.0)  # meter

        self.costmap_nodes = (
            self.get_parameter("switch_costmap.node_names")
            .get_parameter_value()
            .string_array_value
        )
        self.costmap_layers = (
            self.get_parameter("switch_costmap.layer_names")
            .get_parameter_value()
            .string_array_value
        )
        self.front_angle = (
            self.get_parameter("use_angle").get_parameter_value().double_value
        )
        self.danger_dist = (
            self.get_parameter("danger_dist").get_parameter_value().double_value
        )

        ## Read waypoints file
        yaml = ruamel.yaml.YAML()
        waypoints_path = (
            self.get_parameter("waypoints_file").get_parameter_value().string_value
        )
        with open(waypoints_path) as file:
            waypoints_yaml = yaml.load(file)

        ## Register waypoint number of start/end tandem area
        self.tandem_start_list = []
        self.tandem_end_list = []
        self.tandem_id = 0
        for i, data in enumerate(waypoints_yaml["waypoints"]):
            if "tandem_start" in data["point"]:
                self.tandem_start_list.append(i + 2)
            if "tandem_end" in data["point"]:
                self.tandem_end_list.append(i + 2)

        ## Clients changing costmaps parameter
        self.param_clients = []
        for node_name in self.costmap_nodes:
            self.param_clients.append(
                self.create_client(SetParameters, node_name + "/set_parameters")
            )

        ## Variables
        self.waypoint_num = 0
        self.front_range = None
        self.in_tandem_area = False
        self.stop = False

        ## Service Clients
        self.stop_nav_client = self.create_client(Trigger, "/stop_wp_nav")
        self.resume_nav_client = self.create_client(Trigger, "/resume_nav")

        ## Subscribers
        self.wp_num_sub = self.create_subscription(
            UInt16, "/waypoint_num", self.waypoint_num_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.laserscan_callback, 10
        )
        return

    ## Subscribe current waypoint number
    def waypoint_num_callback(self, msg: UInt16):
        if (msg.data == self.waypoint_num) or (self.stop):
            return
        if (self.waypoint_num == 0) and (msg.data > 1):
            for i in range(0, len(self.tandem_start_list)):
                if msg.data >= self.tandem_end_list[i]:
                    self.tandem_id += 1
                    continue
                if (msg.data >= self.tandem_start_list[i]) and (
                    msg.data < self.tandem_end_list[i]
                ):
                    self.tandem_id = i
                    self.in_tandem_area = True
                    self.update_costmap_config(False)
                    self.get_logger().info("Enter tandem area.")
                    break
            self.waypoint_num = msg.data
            return

        if msg.data == self.tandem_start_list[self.tandem_id]:
            self.in_tandem_area = True
            self.update_costmap_config(False)
            self.get_logger().info("Enter tandem area.")

        elif msg.data == self.tandem_end_list[self.tandem_id]:
            self.in_tandem_area = False
            self.update_costmap_config(True)
            self.tandem_id += 1
            self.stop = self.tandem_id >= len(
                self.tandem_start_list
            )  # if True, self.stop will never be False
            self.get_logger().info("Exit from tandem area.")

        self.waypoint_num = msg.data
        return

    ## Subscribe LaserScan to observe obstacles in front of robot
    def laserscan_callback(self, msg: LaserScan):
        try:
            if self.front_range is None:
                front = round(-msg.angle_min / msg.angle_increment)
                ran = int(round(np.deg2rad(self.front_angle / 2) / msg.angle_increment))
                self.front_range = [front - ran, front + ran]
                return

            if not self.in_tandem_area:
                return
            ranges = np.array(msg.ranges[self.front_range[0] : self.front_range[1]])
            ## Use simply minimum
            ranges[ranges <= msg.range_min] = msg.range_max
            min_range = min(ranges)
            ## or use sort
            # sort_ranges = np.sort(ranges)
            # min_range = np.mean(sort_ranges[:5])
            if (not self.stop) and (min_range < self.danger_dist):
                self.stop_nav_client.call_async(SetParameters.Request())
                self.stop = True
                self.get_logger().info(
                    "Stop because of obstacle within {}m ahead.".format(
                        self.danger_dist
                    )
                )

            elif (self.stop) and (min_range >= self.danger_dist + 0.1):
                self.resume_nav_client.call_async(SetParameters.Request())
                self.stop = False
                self.get_logger().info(
                    f"Resumed navigation because the obstacle ahead was more than {self.danger_dist + 0.1}m away."
                )
        except AttributeError:
            pass
        return

    def update_costmap_config(self, enabled: bool):
        for i, client in enumerate(self.clients):
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting again...")
            req = SetParameters.Request()
            req.parameters.append(
                rclpy.Parameter(
                    name=self.costmap_layers[i] + ".enabled",
                    value=enabled,
                ).to_parameter_msg()
            )
            client.call_async(req)
        return


def main(args=None):
    rclpy.init(args=args)
    tandem_manager = TandemManager()

    print(tandem_manager.tandem_start_list)

    if len(tandem_manager.tandem_start_list) > 0:
        try:
            rclpy.spin(tandem_manager)
        except Exception as e:
            print(e)

    tandem_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
