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
from rclpy.clock import Clock
import filterpy.kalman
import filterpy.common
from tf_transformations import euler_from_quaternion
import numpy as np
import message_filters
from std_msgs.msg import String,Float32
from sensor_msgs.msg import Imu,TimeReference
from geometry_msgs.msg import Vector3,QuaternionStamped,Quaternion


class Subscriber(Node):

    def __init__(self):

        self.x_velocity = 0
        self.y_velocity = 0
        self.z_velocity = 0
        self.total_velocity = 0
        self.prev_t = Clock().now().seconds_nanoseconds()
        print(self.prev_t)

        super().__init__('filter_x_sens')
        self.acceleration = self.create_publisher(Vector3, 'acceleration_IMU_without_G', 10)
        self.velocity_gor = self.create_publisher(Quaternion, 'velocity', 10)
        self.steering = self.create_publisher(Vector3, 'steering', 10)
        self.pose = self.create_publisher(Vector3, 'pose', 10)
        self.subscription_IMU = message_filters.Subscriber(self,Imu,'/imu/data')
        self.subscription_Quaternion = message_filters.Subscriber(self,QuaternionStamped,'/filter/quaternion')
        ats = message_filters.ApproximateTimeSynchronizer([self.subscription_IMU, self.subscription_Quaternion],queue_size=1,slop=0.1)
        ats.registerCallback(self.velocity)
    def velocity(self,imu,quat):
        time = Clock().now().seconds_nanoseconds()
        dt = time[0] - self.prev_t[0] + time[1]/(10**9) - self.prev_t[1]/(10**9)
        print(dt) 
        ang_x,ang_y,ang_z = euler_from_quaternion([quat.quaternion.x,quat.quaternion.y,quat.quaternion.z,quat.quaternion.w])
        self.x_velocity += (imu.linear_acceleration.x + 9.81*math.sin(ang_y))*math.cos(ang_y)*dt
        self.y_velocity += (imu.linear_acceleration.y + 9.81*math.sin(ang_x))*math.cos(ang_x)*dt
        self.z_velocity += (imu.linear_acceleration.z + 9.81 * (math.cos(ang_x)* math.cos(ang_y)))*dt
        self.total_velocity = (self.x_velocity**2 + self.y_velocity**2)**(1/2)


        velocity_msg= Quaternion()
        velocity_msg.x = self.x_velocity
        velocity_msg.y = self.y_velocity
        velocity_msg.z = self.z_velocity
        velocity_msg.w = self.total_velocity
        

        accel = Vector3()
        accel.x = (imu.linear_acceleration.x + 9.81*math.sin(ang_y))*math.cos(ang_y)
        accel.y = (imu.linear_acceleration.y - 9.81*math.sin(ang_x))*math.cos(ang_x)
        accel.z = (imu.linear_acceleration.z - 9.81 * (math.cos(ang_x)* math.cos(ang_y)))

        self.acceleration.publish(accel)
        self.velocity_gor.publish(velocity_msg)

        self.prev_t = time

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





