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


class Subscriber(Node):

    def __init__(self):
        qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth=5)
        super().__init__('odom_cam')
        self.prev_time = 0
        self.time = 0
        self.br = CvBridge()
        self.img_rgb = 0
        self.img_depth = 0
        self.img_rgb_prev = 0
        self.img_depth_prev = 0
        self.tf = 0
        self.first = True
        self.velocity_cam = self.create_publisher(Quaternion, 'velocity', 10)
        self.pose = self.create_publisher(Vector3, 'pose', 10)
        self.image_depth_sub = message_filters.Subscriber(self,Image,'/camera/depth/image_rect_raw')
        self.image_rgb_sub = message_filters.Subscriber(self,Image,'/camera/color/image_raw')
        ats = message_filters.ApproximateTimeSynchronizer([self.image_rgb_sub, self.image_depth_sub],queue_size=1,slop=0.1)
        ats.registerCallback(self.odom_calc)
    def odom_calc(self,rgb_msg,depth_msg):
        self.img_rgb = self.br.imgmsg_to_cv2(rgb_msg,"rgb8")
        self.img_rgb = np.asarray(self.img_rgb )
        self.img_rgb = o3d.geometry.Image(self.img_rgb.astype(np.uint8))
        self.img_depth= self.br.imgmsg_to_cv2(depth_msg,'passthrough')
        self.img_depth = np.asarray(self.img_depth)
        self.img_depth = o3d.geometry.Image(self.img_depth.astype(np.float32))
        self.time = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec/(10**9)
        if self.first:
            self.img_rgb_prev = self.img_rgb
            self.img_depth_prev = self.img_depth
            self.first = False 
            self.prev_time = self.time
            return
        
        source_rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(self.img_rgb_prev, self.img_depth_prev)
        target_rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(self.img_rgb, self.img_depth)
        option = o3d.pipelines.odometry.OdometryOption()
        odo_init = np.identity(4)
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic()
        pinhole_camera_intrinsic.intrinsic_matrix = [[554.254691191187,    0, 320.5],
                                    [  0, 554.254691191187,  240.5],
                                    [  0,   0,   1 ]]

        [success_hybrid_term, trans_hybrid_term,
 info] = o3d.pipelines.odometry.compute_rgbd_odometry(
     source_rgbd_image, target_rgbd_image, pinhole_camera_intrinsic, odo_init,
     o3d.pipelines.odometry.RGBDOdometryJacobianFromColorTerm(), option)
        trans_hybrid_term = np.array(trans_hybrid_term)
        x=trans_hybrid_term[0][3]/(self.time-self.prev_time)
        y=trans_hybrid_term[1][3]/(self.time-self.prev_time)
        z=trans_hybrid_term[2][3]/(self.time-self.prev_time)
        #print((x**2+y**2)**0.5)
        #print(1/(self.time-self.prev_time))
        velocity_msg= Quaternion()
        velocity_msg.x = y
        velocity_msg.y = x
        velocity_msg.z = z
        velocity_msg.w = ((x**2+y**2)**0.5)
        self.velocity_cam.publish(velocity_msg)

        self.img_rgb_prev=self.img_rgb
        self.img_depth_prev=self.img_depth
        self.prev_time = self.time
        
        

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





