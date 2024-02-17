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

import message_filters
import rclpy
import math
from rclpy.node import Node
from rclpy.clock import Clock
import filterpy.kalman
import filterpy.common
import numpy as np

from cv_bridge import CvBridge
from std_msgs.msg import String,Float32
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import Vector3,Quaternion
import open3d as o3d
from rclpy.qos import qos_profile_sensor_data, QoSProfile
import numpy as np
import time
import cv2



class Subscriber(Node):

    def __init__(self):

        self.prev_t_imu = Clock().now().seconds_nanoseconds()
        self.prev_t_realsens = self.prev_t_imu

        self.filterx = filterpy.kalman.KalmanFilter(dim_x=3,dim_z=1)
        
        self.filterx.x = np.array([0.0, 0.0, 0.0])
        self.filterx.P = np.eye(3)*1 
        
        super().__init__('kalman_filter')
        self.velocity_kalman = self.create_publisher(Vector3, 'kalman', 10)
        self.subscription = self.create_subscription(
            Vector3,
            '/acceleration_IMU_without_G',
            self.kalman_IMU,
            1)
        self.subscription = self.create_subscription(
            Quaternion,
            '/velocit_realsens',
            self.kalman_realsens,
            1)
        
    def kalman_IMU(self,msg):
        time = Clock().now().seconds_nanoseconds()
        dt = time[0] - self.prev_t_imu[0] + time[1]/(10**9) - self.prev_t_imu[1]/(10**9)
        processNoise = 1e-4
        measurementSigma = 3.3
        
        self.filterx.F = np.array([ [1,   dt,     (dt**2)/2],
                                        [0,   1.0,    dt],
                                         [0,   0,      1.0]])
        self.filterx.H = np.array([[0,0,1]])
        self.filterx.Q = filterpy.common.Q_discrete_white_noise(dim=3, dt=dt, var=processNoise)
        self.filterx.R = np.array([[measurementSigma*measurementSigma]])
        self.filterx.predict()
        z =[[msg.x,msg.y,msg.z]] 
        self.filterx.update([[msg.x]])

        x,y,z = self.filterx.x
        velocity = Vector3()
        velocity.x= y
        velocity.y = 0.0
        velocity.z = 0.0

        self.velocity_kalman.publish(velocity)
        self.prev_t_imu = time


    def kalman_realsens(self,msg):
        time = Clock().now().seconds_nanoseconds()
        dt = time[0] - self.prev_t_realsens[0] + time[1]/(10**9) - self.prev_t_realsens[1]/(10**9)
        processNoise = 1e-4
        measurementSigma = 0.05  
        
        self.filterx.F = np.array([ [1,   dt,     (dt**2)/2],
                                    [0,   1.0,           dt],
                                    [0,   0,            1.0]])
        self.filterx.H = np.array([[0,1,0]])
        self.filterx.Q = filterpy.common.Q_discrete_white_noise(dim=3, dt=dt, var=processNoise)
        self.filterx.R = np.array([[measurementSigma*measurementSigma]])
        self.filterx.predict()
        z =[[msg.x,msg.y,msg.z]] 
        self.filterx.update([[msg.x*10]])
        x,y,z = self.filterx.x
        velocity = Vector3()
        velocity.x= y
        velocity.y = 0.0
        velocity.z = 0.0

        self.velocity_kalman.publish(velocity)
        self.prev_t_realsens = time
        
        

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





