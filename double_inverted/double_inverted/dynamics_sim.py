import numpy as np
import matplotlib.pyplot as plt
import time
from math import sin, cos

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from custom_msgs.msg import TorqueInput, States

class double_inverted_pendulum(Node):
    def __init__(self):
        super().__init__('main')
        self.state_update_frequency = 500
        
        self.state_update_timeperiod = 1/self.state_update_frequency
        
        # Timers
        update_states_timer = self.create_timer(1 / self.state_update_frequency, self.update_pendulum_states)
        
        # Publishers/Subscribers/Services
        self.visualizer_1 = self.create_publisher(Marker, '/double_pendulum_viz_1', 1)
        self.visualizer_2 = self.create_publisher(Marker, '/double_pendulum_viz_2', 1)
        
        # Attributes
        self.t_start = time.time()
        self.t_prev = time.time() - 0.0001   # 0.001 - This value won't matter much as t_prev will keep getting updated
        self.obj_id = 0
        
        #initialising states
        self.theta0 = np.pi/2
        self.theta1 = np.pi
        self.theta0_dot = 0.0
        self.theta1_dot = 0.0
        self.theta0_dot_dot =0.0
        self.theta1_dot_dot =0.0
        self.total_time = 0
        self.amp = 0
        self.freq = 180
        
        #properties
        self.mass1 = 1.0
        self.mass2 = 1.0
        self.length1 = 1.0
        self.length2 = 1.0
        self.g = 9.81
        
    def f1(self, x):
        self.theta0_dot_dot = (1/((self.mass1+self.mass2)*self.length1*self.length2)  *  ((self.amp*self.freq)**2*cos(self.freq*self.total_time)*sin(self.theta1)+self.g*sin(self.theta1)-self.length1*(self.theta0_dot**2)*sin(self.theta0-self.theta1))  *  self.length2*self.mass2*cos(self.theta0-self.theta1)  -  1/((self.mass1+self.mass2)*self.length1)  *  ((self.mass1+self.mass2)*sin(self.theta0)*((self.amp*self.freq)**2*cos(self.amp*self.total_time)+self.g)+self.length2*self.mass2*(self.theta1_dot**2)*sin(self.theta0-self.theta1)))  /  (1  -  1/((self.mass1+self.mass2)*self.length1*self.length2)  *  self.length2*self.mass2*cos(self.theta0-self.theta1)  *  self.length1*cos(self.theta0-self.theta1))
        return np.array([x[1], self.theta0_dot_dot])
    
    def f2(self, y):
        #inertia = self.mass*self.length*self.length
        #torque = (self.mass*self.length*self.length*self.theta1_dot_dot + self.mass*self.length*self.length*self.theta0_dot_dot*cos(self.theta0-self.theta1) -self.mass*self.g*self.length*sin(self.theta1) - self.mass*self.length*self.length*self.theta0_dot*self.theta1_dot*sin(self.theta0-self.theta1))

        self.theta1_dot_dot = (1/((self.mass1+self.mass2)*self.length1*self.length2)  *  ((self.mass1+self.mass2)*sin(self.theta0)*((self.amp*self.freq)**2*cos(self.freq*self.total_time)+self.g)+self.length2*self.mass2*(self.theta1_dot**2)*sin(self.theta0-self.theta1))  *  self.length1*cos(self.theta0-self.theta1)  -  1/self.length2  *  ((self.amp*self.freq)**2*cos(self.freq*self.total_time)*sin(self.theta1)+self.g*sin(self.theta1)-self.length1*(self.theta0_dot**2)*sin(self.theta0-self.theta1)))  /  (1  -  1/((self.mass1+self.mass2)*self.length1*self.length2)  *  self.length2*self.mass2*cos(self.theta0-self.theta1)  *  self.length1*cos(self.theta0-self.theta1))
        return np.array([y[1], self.theta1_dot_dot])        
        
    def update_pendulum_states(self):
        
        dt = time.time() - self.t_prev
        self.t_prev = time.time()
        
        x = np.array([self.theta0,self.theta0_dot])
        y = np.array([self.theta1, self.theta1_dot])
        
        #trying to get accuracy of dt^4
        x_intermediate_1 = x + 0.125*dt*self.f1(x)
        self.theta0, self.theta0_dot = x_intermediate_1
        self.theta0 = (self.theta0 + np.pi)%(2*np.pi) - np.pi # Keeping theta between -pi to pi
        y_intermediate_1 = y + 0.125*dt*self.f2(y)
        self.theta1, self.theta1_dot = y_intermediate_1
        self.theta1 = (self.theta1 + np.pi)%(2*np.pi) - np.pi # keeping theta between -pi to pi
        x_intermediate_2 = x + 0.25*dt*self.f1(x_intermediate_1)
        self.theta0, self.theta0_dot = x_intermediate_2
        self.theta0 = (self.theta0 + np.pi)%(2*np.pi) - np.pi # Keeping theta between -pi to pi
        y_intermediate_2 = y + 0.25*dt*self.f2(y_intermediate_1)
        self.theta1, self.theta1_dot = y_intermediate_2
        self.theta1 = (self.theta1 + np.pi)%(2*np.pi) - np.pi # keeping theta between -pi to pi
        x_intermediate_3 = x + 0.5*dt*self.f1(x_intermediate_2)
        self.theta0, self.theta0_dot = x_intermediate_3
        self.theta0 = (self.theta0 + np.pi)%(2*np.pi) - np.pi # Keeping theta between -pi to pi
        y_intermediate_3 =y + 0.5*dt*self.f2(y_intermediate_2)
        self.theta1, self.theta1_dot = y_intermediate_3
        self.theta1 = (self.theta1 + np.pi)%(2*np.pi) - np.pi # keeping theta between -pi to pi
        
        #finally updating states.
        x = x + dt*self.f1(x_intermediate_3)
        
        self.theta0, self.theta0_dot = x

        self.theta0 = (self.theta0 + np.pi)%(2*np.pi) - np.pi # Keeping theta between -pi to pi
        
        y = y+ dt*self.f2(y_intermediate_3)
        
        self.theta1, self.theta1_dot = y
        
        self.theta1 = (self.theta1 + np.pi)%(2*np.pi) - np.pi # keeping theta between -pi to pi
        
        self.total_time += dt

        self.visualize_pendulum_1()       
        self.visualize_pendulum_2()
        self.obj_id = self.obj_id + 2
        
        print(self.total_time)
        #print(self.theta1)
        
        return
    def visualize_pendulum_1(self):
        pendulum1_marker = Marker()
        pendulum1_marker.header.frame_id = "map"
        pendulum1_marker.id = self.obj_id
        pendulum1_marker.type = Marker.LINE_STRIP
        pendulum1_marker.action = Marker.ADD
        pendulum1_marker.pose.orientation.w = 1.0
        pendulum1_marker.scale.x = 0.05  # Line width
        
        # Set the points of the line
        point_1 = Point()
        point_1.x = 0.0
        point_1.y = 0.0 - self.amp*cos(self.freq*self.total_time)
        point_1.z = 0.0

        point_2 = Point()
        point_2.x = self.length1 * sin(self.theta0)
        point_2.y = -self.length1 * cos(self.theta0) - self.amp*cos(self.freq*self.total_time)
        point_2.z = 0.0
        pendulum1_marker.points = [point_1,
                            point_2
                        ]
        
        
        pendulum1_marker.color.r = 1.0
        pendulum1_marker.color.a = 1.0  # Alpha value
        Duration_of_pendulum1_marker = Duration()
        Duration_of_pendulum1_marker.sec = 0
        Duration_of_pendulum1_marker.nanosec = int(self.state_update_timeperiod * 1e+9)
        pendulum1_marker.lifetime = Duration_of_pendulum1_marker  # Permanent pendulum_marker
        self.visualizer_1.publish(pendulum1_marker)
    
    
    def visualize_pendulum_2(self):
        pendulum_marker = Marker()
        pendulum_marker.header.frame_id = "map"
        pendulum_marker.id = self.obj_id + 1
        pendulum_marker.type = Marker.LINE_STRIP
        pendulum_marker.action = Marker.ADD
        pendulum_marker.pose.orientation.w = 1.0
        pendulum_marker.scale.x = 0.05  # Line width
        point_3 = Point()
        point_3.x = self.length1 * sin(self.theta0)
        point_3.y = -self.length1 * cos(self.theta0) - self.amp*cos(self.freq*self.total_time)
        point_3.z = 0.0
        point_4 = Point()
        point_4.x = point_3.x + self.length2 * sin(self.theta1)
        point_4.y = point_3.y - self.length2 * cos(self.theta1) - self.amp*cos(self.freq*self.total_time)
        point_4.z = 0.0
        pendulum_marker.points = [point_3,
                            point_4
                        ]
        pendulum_marker.color.r = 1.0
        pendulum_marker.color.a = 1.0  # Alpha value
        Duration_of_pendulum_marker = Duration()
        Duration_of_pendulum_marker.sec = 0
        Duration_of_pendulum_marker.nanosec = int(self.state_update_timeperiod * 1e+9)
        pendulum_marker.lifetime = Duration_of_pendulum_marker  # Permanent pendulum_marker
        self.visualizer_2.publish(pendulum_marker)

        
        
        
def main(args = None):

    rclpy.init(args = args)
    pendulum_ = double_inverted_pendulum()
    rclpy.spin(pendulum_)

    pendulum_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
