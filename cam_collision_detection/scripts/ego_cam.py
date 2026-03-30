#!/usr/bin/env python

# ==============================================================================
# MIT License
#
# Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ==============================================================================

import numpy as np


import rclpy
from rclpy.node import Node
from etsi_its_cam_msgs.msg import *
import utils

from sensor_msgs.msg import NavSatFix

from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import TwistStamped

from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.msg import VelocityReport
from autoware_vehicle_msgs.srv import ControlModeCommand
from autoware_control_msgs.msg import *

class ego_cam(Node):

    def __init__(self):

        super().__init__("cam_publisher")
        
        self.ego_cam = self.create_publisher(CAM, "/v2x/etsi_parser/cam/in", 1)
        
        self.velocity_report = self.create_subscription(
            VelocityReport, '/vehicle/status/velocity_status', self.speedCallback, 1)
        
        self.gnss_sub = self.create_subscription(
            NavSatFix, '/gnss', self.gnssCallback, 1)
        
        self.heading_sub = self.create_subscription(
            Vector3Stamped, '/filter/euler', self.headingCallback, 1)
        
        self.velocity = 0.0
        self.heading = 0.0
        self.heading_i = 0.0
        
    def speedCallback(self, msg):
        self.velocity = msg.longitudinal_velocity
        
    def headingCallback(self, msg):
        
        self.heading = msg.vector.z
        
        if(self.heading < 0.0):
            self.heading = self.heading + 360.0
    
    def gnssCallback(self, msggnss):
        
        msg = CAM()

        msg.header.protocol_version = 2
        msg.header.message_id = msg.header.MESSAGE_ID_CAM
        msg.header.station_id.value = 32

        msg.cam.generation_delta_time.value = int(utils.get_t_its(self.get_clock().now().nanoseconds) % 65536)

        msg.cam.cam_parameters.basic_container.station_type.value = msg.cam.cam_parameters.basic_container.station_type.PASSENGER_CAR
        msg.cam.cam_parameters.basic_container.reference_position.latitude.value = int(msg.cam.cam_parameters.basic_container.reference_position.latitude.ONE_MICRODEGREE_NORTH * 1e6 * msggnss.latitude)
        msg.cam.cam_parameters.basic_container.reference_position.longitude.value = int(msg.cam.cam_parameters.basic_container.reference_position.longitude.ONE_MICRODEGREE_EAST * 1e6 * msggnss.longitude)
        msg.cam.cam_parameters.basic_container.reference_position.altitude.altitude_value.value = int(100 * msggnss.altitude)

        basic_vehicle_container_high_frequency = BasicVehicleContainerHighFrequency()
        basic_vehicle_container_high_frequency.heading.heading_value.value = int(self.heading * 10)
        basic_vehicle_container_high_frequency.heading.heading_confidence.value = basic_vehicle_container_high_frequency.heading.heading_confidence.EQUAL_OR_WITHIN_ONE_DEGREE
        basic_vehicle_container_high_frequency.speed.speed_value.value = int(self.velocity * 100)
        basic_vehicle_container_high_frequency.speed.speed_confidence.value = basic_vehicle_container_high_frequency.speed.speed_confidence.EQUAL_OR_WITHIN_ONE_CENTIMETER_PER_SEC
        basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_value.value = basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_value.TEN_CENTIMETERS * 42
        basic_vehicle_container_high_frequency.vehicle_width.value = basic_vehicle_container_high_frequency.vehicle_width.TEN_CENTIMETERS * 18
        msg.cam.cam_parameters.high_frequency_container.choice = msg.cam.cam_parameters.high_frequency_container.CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY
        msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency = basic_vehicle_container_high_frequency

        self.get_logger().debug(f"Publishing CAM (/gnss)")
        self.ego_cam.publish(msg)
        


if __name__ == "__main__":

    rclpy.init()
    ego_cam = ego_cam()
    rclpy.spin(ego_cam)
    rclpy.shutdown()
