import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class SteeringActionClient(Node):

    def __init__(self):
        super().__init__('wheel_steer_actionclient')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_steering/follow_joint_trajectory')
        self.velocity = self.create_publisher(Float64MultiArray, '/joint_wheels/commands', 10)
        self.subscription = self.create_subscription(
            Twist,
            '/demo/cmd_demo',
            self.send_goal,
            10)

    def send_goal(self, msg):
        goal_msg = FollowJointTrajectory.Goal()

        # Fill in data for trajectory
        joint_names = ["steering_left",
                       "steering_right"]

        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0]

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        point2.positions = [msg.angular.z, msg.angular.z]

        points.append(point2)

        goal_msg.goal_time_tolerance = Duration(
            seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points
        
        goal_velocity = Float64MultiArray()
        goal_velocity.data = [msg.linear.x,msg.linear.x,msg.linear.x,msg.linear.x]
        
        self.velocity.publish(goal_velocity)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info('Received feedback:'+str(feedback))


def main(args=None):

    rclpy.init()

    action_client = SteeringActionClient()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()

