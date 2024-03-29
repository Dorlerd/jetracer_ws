import message_filters
import rclpy
import math
from rclpy.node import Node
from rclpy.clock import Clock
import numpy as np
from control.matlab import *

from cv_bridge import CvBridge
from std_msgs.msg import String,Float32
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import Twist
import open3d as o3d
from rclpy.qos import qos_profile_sensor_data, QoSProfile
import numpy as np
import time
import cv2
from jetracer.nvidia_racecar import NvidiaRacecar



class Subscriber(Node):

    def __init__(self):
        print("prev")
        self.car = NvidiaRacecar()
        self.car.steering_offset = 0
        self.car.steering_gain = 1
        self.car.throttle_gain = 1
        self.car.throttle = 1
        self.car.throttle = 0
        self.car.steering = -1
        self.car.steering = 1
        self.car.steering = 0
        
        super().__init__('base_move')
        self.subscription = self.create_subscription(
            Twist,
            '/move_jetracer',
            self.move,
            1)
        
    def move(self,msg):
        self.car.steering = 0
        self.car.steering = msg.angular.z
        speed = msg.linear.x
        prev_speed = self.car.throttle
        dspeed = speed - prev_speed
        f = tf(1, [4*abs(dspeed), 1]);
        y,x=step(dspeed*f)
        for i in range(71):
            self.car.throttle = prev_speed + y[i+1]
            time.sleep(x[i+1]-x[i])

        
        

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