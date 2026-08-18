#!/usr/bin/env python3

import argparse
import datetime
import requests
import os
import time
import pandas as pd
import numpy as np
import logging

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from rclpy.utilities import remove_ros_args
import sys

class uw_gps_logger(Node):

    def __init__(self):
        super().__init__('underwater_gps_logger')

        self.declare_parameter('verbose', False)
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        self.uw_data_pub = self.create_publisher(Float32MultiArray, '/uw_gps_data', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.parser = argparse.ArgumentParser(description=__doc__)
        self.parser.add_argument(
            "-u",
            "--url",
            help = "Base URL to use",
            type = str,
            default = "http://192.168.2.94")
        self.parser.add_argument(
            '-t', 
            '--temp',
            help='Temperature to send', 
            type=float, 
            default=10)
        self.parser.add_argument(
            "-a",
            "--antenna",
            action = "store_true",
            help = (
                "Use mid-point of base of antenna as origin for the acoustic " +
                "position. Default origin is the point at sea level directly " +
                "below/above that which the positions of the receivers/antenna " +
                "are defined with respect to"))
        self.args = self.parser.parse_args(remove_ros_args(sys.argv)[1:])

        self.base_url = self.args.url

        self.get_logger().info(f'Using base_url: {self.base_url}')
        self.get_logger().info("Start Logging")

    def get_data(self, url):
        try:
            r = requests.get(url)
        except requests.exceptions.RequestException as exc:
            if self.verbose:
                self.get_logger().info(f'Exception occured {format(exc)}.')
            return None

        if r.status_code != requests.codes.ok:
            if self.verbose:
                self.get_logger().info(f'Got error {r.status_code}: {r.text}.')
            return None

        return r.json()

    def get_antenna_position(self):
        return self.get_data("{}/api/v1/config/antenna".format(self.base_url))

    def get_acoustic_position(self):
        return self.get_data("{}/api/v1/position/acoustic/raw".format(self.base_url))

    def get_acoustic_position_filtered(self):
        return self.get_data("{}/api/v1/position/acoustic/filtered".format(self.base_url))

    def get_global_position(self):
        return self.get_data("{}/api/v1/position/global".format(self.base_url))

    def get_master_position(self):
        return self.get_data("{}/api/v1/position/master".format(self.base_url))

    def timer_callback(self):
        timestamp = datetime.datetime.now()

        df_tmp = [0]*19

        df_tmp[0] = timestamp.year
        df_tmp[1] = timestamp.month
        df_tmp[2] = timestamp.day
        df_tmp[3] = timestamp.hour
        df_tmp[4] = timestamp.minute
        df_tmp[5] = timestamp.second
        df_tmp[6] = timestamp.microsecond

        acoustic_position = self.get_acoustic_position()
        acoustic_position_filtered = self.get_acoustic_position_filtered()

        antenna_position = None
        if self.args.antenna:
            antenna_position = self.get_antenna_position()
        depth = None
        global_position = self.get_global_position()

        if acoustic_position:
            df_tmp[7] = acoustic_position["x"]
            df_tmp[8] = acoustic_position["y"]
            df_tmp[9] = acoustic_position["z"]
            depth = acoustic_position["z"]
        else:
            if self.verbose:
                self.get_logger().info('no acoustic position')

        if acoustic_position_filtered:
            df_tmp[16] = acoustic_position_filtered["x"]
            df_tmp[17] = acoustic_position_filtered["y"]
            df_tmp[18] = acoustic_position_filtered["z"]
        else:
            if self.verbose:
                self.get_logger().info('no filtered acoustic position')

        if antenna_position:
            df_tmp[10] = antenna_position["x"]
            df_tmp[11] = antenna_position["y"]
            df_tmp[12] = antenna_position["z"]

        if global_position:
            df_tmp[13] = global_position["lat"]
            df_tmp[14] = global_position["lon"]

        if depth:
            df_tmp[15] = depth

        # Publish data
        msg = Float32MultiArray()
        msg.data = df_tmp
        self.uw_data_pub.publish(msg)

rclpy.init()
node = uw_gps_logger()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()