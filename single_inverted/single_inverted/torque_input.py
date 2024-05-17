import numpy as np
import matplotlib.pyplot as plt
import time
from math import sin, cos, tan

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from custom_msgs.msg import TorqueInput, States

class torque_input(Node):
    
    def __init__(self):
        super().__init__('controller')
        self.state_subscriber = self.create_subscription(States, "/state_feedback", self.torque_update, 10)
        self.torque_publisher = self.create_publisher(TorqueInput, "/torque_input", 10)
        self.prev_error = 0
        self.integral = 0
        
    def torque_update(self, state: States):
        torque = TorqueInput()
        if state.theta >=0:
            x = (np.pi - state.theta)
        if state.theta <0:
            x = -(np.pi + state.theta)
            
        error = x
        p = 50
        i = 1
        d = 0.4
        dt = 1/500
        self.integral = self.integral + x*dt
        self.derivative = (error - self.prev_error)/dt
        torque.torque_value = (p*error + i*self.integral + d*self.derivative) 
        self.torque_publisher.publish(torque)
        self.prev_error = error
        self.get_logger().info(f"error:{x} current_w:{state.theta_dot} net_torque:{torque.torque_value }")
        
        
def main(args = None):

    rclpy.init(args = args)
    torque = torque_input()
    rclpy.spin(torque)

    torque.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
                