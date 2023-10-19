#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy import qos
from std_msgs.msg import UInt16
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
import numpy as np
import ruamel.yaml
import time


NODE_NAME = "tandem_run_manager"


class TandemManager(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        ## Parameters
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter(
            "switch_costmap.node_names", ["global_costmap/global_costmap"]
        )
        self.declare_parameter("switch_costmap.layer_names", ["obstacle_layer"])
        self.declare_parameter("use_angle", 20.0)  # degree
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
                self.create_client(SetParameters, f"{node_name}/set_parameters")
            )

        ## Variables
        self.waypoint_num = 0
        self.front_range = None
        self.in_tandem_area = False
        self.stop = False
        self.resume_dist = self.danger_dist + 0.3

        ## Service Clients
        self.stop_nav_client = self.create_client(Trigger, "stop_wp_nav")
        self.resume_nav_client = self.create_client(Trigger, "resume_nav")

        ## Subscribers
        # qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.wp_num_sub = self.create_subscription(
            UInt16, "waypoint_num", self.waypoint_num_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.laserscan_callback,
            qos.qos_profile_sensor_data,
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
            self.stop_nav_client.call_async(Trigger.Request())
            self.stop = True
            self.get_logger().info(
                f"Stop because of obstacle within {self.danger_dist}m ahead."
            )

        elif (self.stop) and (min_range >= self.resume_dist):
            self.resume_nav_client.call_async(Trigger.Request())
            self.stop = False
            self.get_logger().info(
                f"Resumed navigation because the obstacle ahead was more than {self.resume_dist}m away."
            )
        return

    def update_costmap_config(self, enabled: bool):
        self.stop_nav_client.call_async(Trigger.Request())
        for i, client in enumerate(self.param_clients):
            req = SetParameters.Request()
            req.parameters.append(
                rclpy.Parameter(
                    name=f"{self.costmap_layers[i]}.enabled",
                    value=enabled,
                ).to_parameter_msg()
            )
            client.call_async(req)
        time.sleep(1)
        self.resume_nav_client.call_async(Trigger.Request())
        return


def main(args=None):
    rclpy.init(args=args)
    tandem_manager = TandemManager()

    if len(tandem_manager.tandem_start_list) > 0:
        try:
            rclpy.spin(tandem_manager)
            tandem_manager.destroy_node()
            rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
