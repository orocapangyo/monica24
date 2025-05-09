#!/usr/bin/env python
# coding:utf-8
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.clock import Clock
import os
import sys
from geometry_msgs.msg import Twist

class Stop_Car(Node):
    def __init__(self,name):
        super().__init__(name)
        
    def exit_pro(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

def main():
    rclpy.init()
    stop_car = Stop_Car("StopCarNode")
    try:
        rclpy.spin(stop_car)
    except KeyboardInterrupt:
        pass
    finally:
        stop_car.exit_pro()
        stop_car.destroy_node()
        rclpy.shutdown()
