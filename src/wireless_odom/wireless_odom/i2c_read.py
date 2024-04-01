import rclpy
from rclpy.node import Node
from control.matlab import *

from std_msgs.msg import Float32
import smbus



class Subscriber(Node):

    def __init__(self):
        self.bus = smbus.SMBus(0)
        
        super().__init__('i2c_read')
        self.publisher_ = self.create_publisher(Float32, 'pos_wheel', 10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.pos_wheel)
        
    def pos_wheel(self):
        data1 = self.bus.read_byte_data(0x36,0x0D)
        data2 = self.bus.read_byte_data(0x36,0x0C)
        data = data2 << 8 | data1
        pos_wheel = Float32()
        pos_wheel.data = data*0.087890625
        self.publisher_.publish(pos_wheel)



        
        

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