class RosOpen3D:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_rgb_sub = message_filters.Subscriber('/camera/color/image_raw',
                                               Image,queue_size=1)
        self.image_depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', 
                                               Image, queue_size=1)
        msg = rospy.wait_for_message('/camera/aligned_depth_to_color/camera_info', CameraInfo)
        # save camera intrinsics
        intrinsic_matrix = np.array(msg.K).reshape(3, 3)
        self.pcd_pub = rospy.Publisher('/pcd_created/real', PointCloud2, queue_size=1)
        ats = message_filters.ApproximateTimeSynchronizer([self.image_rgb_sub, self.image_depth_sub],queue_size=1,slop=0.1)
        ats.registerCallback(self.image_callback)
       
    def image_callback(self, rgb_msg, depth_msg):
        
        # ROS to CV2
        rgb_cv2 = self.bridge.imgmsg_to_cv2(rgb_msg,desired_encoding='rgb8')
        rgb_np = np.asarray(rgb_cv2)
        rgp_np = np.array([rgb_np])  
        print(rgb_np.shape)

        rospy.loginfo('Transforming ROS RGB and Depth into CV2')
        rgb = o3d.geometry.Image(rgb_np.astype(np.uint8))

        depth_cv2 = self.bridge.imgmsg_to_cv2(depth_msg,desired_encoding='passthrough')
        depth_np = np.asarray(depth_cv2)
        depth = o3d.geometry.Image(depth_np.astype(np.float32))
        print(depth_np.shape)
        assert (depth_np.shape == rgb_np.shape[:2])
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth, convert_rgb_to_intensity=False)

        camera_intrinsics = o3d.io.read_pinhole_camera_intrinsic("camera_intrinsics/d435_real.json")
        pcd  = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camera_intrinsics)
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        o3d.visualization.draw_geometries([pcd],window_name="Raw PCD",
                                         width=1920, height=1080, left=0,top=500)
        # Point cloud is really distorted, not sure why. 

        # THIS TRANSFORMS OPEN3D POINTCLOUD TO ROS POINTCLOUD
        ros_pcd = orh.o3dpc_to_rospc(pcd, frame_id='map', stamp=rospy.Time())
        self.pcd_pub.publish(ros_pcd)

if __name__ == '__main__':
    rospy.init_node("create_pcd_node", anonymous=True)
    create_pcd = RosOpen3D()
    rospy.spin()
