import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from navigate_msgs.action import NavPose

class NavPoseClient(Node):

    def __init__(self):
        super().__init__('nav_pose_client')

        # Define quality of service
        qos = QoSProfile(depth=10)

        # Subscribers
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/input/goal_pose', self.goal_pose_callback, 10)

        # ActionClient
        self._action_client = ActionClient(self, PoseStamped, 'nav_pose')

        # Goal handle
        self.goal_handle = None

        self.get_logger().info("NavPose client has been initialised.")

    def goal_pose_callback(self, msg):
        # Cancel previous action
        if self.goal_handle:
            self._action_client.cancel_goal(self.goal_handle)

        goal_msg = msg
        self.goal_handle = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.goal_handle.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Remaining distance: {feedback.distance}")

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('goal rejected')
            return
        
        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()

        self.get_logger().info(f'Result {result.success}')

def main(args=None):
    rclpy.init(args=args)

    nav_pose_action_client = NavPoseClient()

    rclpy.spin(nav_pose_action_client)

    nav_pose_action_client.destroy()
    rclpy.shutdown()

if __name__ == "__main__":
    main()