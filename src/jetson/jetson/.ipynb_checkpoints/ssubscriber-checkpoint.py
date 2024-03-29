# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import math
from rclpy.node import Node
import filterpy.kalman
import filterpy.common
import numpy as np

from std_msgs.msg import String,Float32
from sensor_msgs.msg import Imu,TimeReference
from geometry_msgs.msg import Vector3,Quaternion


class Subscriber(Node):

    def __init__(self):
        self.x_velocity = 0
        self.y_velocity = 0
        self.z_velocity = 0
        self.total_velocity = 0
        self.x_acceleration = 0
        self.y_acceleration = 0
        self.z_acceleration = 0
        self.ang_x = 0
        self.ang_y = 0
        self.ang_z = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.sum_ang_x = 0
        self.sum_ang_y = 0
        self.x_ang_cor = 0
        self.y_ang_cor = 0
        self.k = True
        self.i = 0
        self.dt = 0.01                      
        self.measurementSigma = 0.5        
        self.processNoise = 1e-4
        self.ac_x = []
        self.ac_y = []
        self.ac_z = []
        self.gy_x = []
        self.gy_y = []
        self.gy_z = []
        self.filterspeed = filterpy.kalman.KalmanFilter(dim_x=6,dim_z=6)
        self.filterspeed.F = np.eye(6) 
        self.filterspeed.H = np.eye(6) 
        self.filterspeed.Q = np.eye(6)*self.processNoise 
        self.filterspeed.R = np.eye(6)*self.measurementSigma**2 
        self.filterspeed.x = np.array([0,0,0,0,0,0]) 
        self.filterspeed.P = np.eye(6)*8 
        filteredx = [] 
        super().__init__('subscriber')
        self.velocity_gor = self.create_publisher(Quaternion, 'velocity', 10)
        self.steering = self.create_publisher(Vector3, 'steering', 10)
        self.pose = self.create_publisher(Vector3, 'pose', 10)
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.acceleration_init,
            1)
        self.subscription  # prevent unused variable warning
    def acceleration_init(self,msg):
        z = [[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z,msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]] 
        self.filterspeed.predict()
        self.filterspeed.update(z)
        self.filterspeed.x
        self.ac_x = self.filterspeed.x[0]
        self.ac_y = self.filterspeed.x[1]
        self.ac_z = self.filterspeed.x[2]
        self.gy_x = self.filterspeed.x[3]
        self.gy_y = self.filterspeed.x[4]
        self.gy_z = self.filterspeed.x[5]
        
        if self.k:
            self.ang_x = math.asin(self.ac_y/9.81)
            self.ang_y = math.asin(self.ac_x/9.81)
            self.k = False
        self.velocity()
        
    def velocity(self):
        self.ang_x += self.gy_x*0.01
        self.ang_y += self.gy_y*0.01
        self.ang_z += self.gy_z*0.01
        
        self.x_velocity += (self.ac_x - 9.81*math.sin(self.ang_y))*math.cos(self.ang_y)*0.01
        self.y_velocity += (self.ac_y - 9.81*math.sin(self.ang_x))*math.cos(self.ang_x)*0.01
        self.z_velocity += (self.ac_z - 9.81 * (math.cos(self.ang_x)* math.cos(self.ang_y)))*0.01
        self.total_velocity = (self.x_velocity**2 + self.y_velocity**2)**(1/2)
        
        self.x += self.x_velocity*0.01
        self.y += self.y_velocity*0.01
        self.z += self.z_velocity*0.01
        
        print("x ang:",self.ang_x,"\ny ang:",self.ang_y)
        print("x velocity:",self.x_velocity,"\ny velocity:",self.y_velocity,"\ntotal horizontal velocity:",self.total_velocity)
        velocity_msg= Quaternion()
        velocity_msg.x = self.x_velocity
        velocity_msg.y = self.y_velocity
        velocity_msg.z = self.z_velocity
        velocity_msg.w = self.total_velocity
        angular_msg = Vector3()
        angular_msg.x = self.ang_x
        angular_msg.y = self.ang_y
        angular_msg.z = self.ang_z
        pose_data = Vector3()
        pose_data.x = self.x
        pose_data.y = self.y
        pose_data.z = self.z

        self.velocity_gor.publish(velocity_msg)
        self.steering.publish(angular_msg)
        self.pose.publish(pose_data)
        

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





