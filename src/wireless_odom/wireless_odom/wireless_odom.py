import rclpy
from rclpy.node import Node
from control.matlab import *
from rclpy.clock import Clock
import math

from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from tf2_ros import TransformBroadcaster
import smbus

class Differ():

    def __init__(self):
        self.prev_pos = 0

    def data(self,new,time):
        data = (new - self.prev_pos)/time
        self.prev_pos = new
        return data


class Subscriber(Node):

    def __init__(self):
        self.bus = smbus.SMBus(0)

        self.prev_t = 0

        self.rul = 0
        
        self.x = 0
        self.y = 0
        self.phi = 0
        self.d_pos_wheel = Differ()

        super().__init__('wireless_odom')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher_ = self.create_publisher(Vector3, 'odom', 10)
        self.subscription = self.create_subscription(Float32,
            '/pos_wheel',
            self.wireless_odom,1)
        self.kalman = self.create_subscription(Vector3,
            '/kalman',
            self.up_date_odom,1)
        
    def wireless_odom(self,msg):
        time = Clock().now().seconds_nanoseconds()
        dt = time[0] - self.prev_t[0] + time[1]/(10**9) - self.prev_t[1]/(10**9)
        
        pos_wheel = msg.data
        v_pos_wheel = 0.0235*self.d_pos_wheel.data(pos_wheel,dt)

        self.x += v_pos_wheel*math.cos(self.phi)
        self.y += v_pos_wheel*math.sin(self.phi)


        self.prev_t = time

    def up_date_odom(self,msg):

        



        
        

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