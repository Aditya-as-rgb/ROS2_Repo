import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

class SimplePathPlanner(Node):
    def __init__(self):
        super().__init__('simple_path_planner')
        
        # Initialize the navigator
        self.navigator = BasicNavigator()
        self.get_logger().info('Waiting for Nav2 to activate...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is ready!')

    def create_pose(self, x, y, yaw=0.0):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.w = 1.0
        
        return pose

    def navigate_to_goal(self, x, y):
        """Navigate to a single goal"""
        goal_pose = self.create_pose(x, y)
        
        self.get_logger().info(f'Navigating to goal: x={x}, y={y}')
        self.navigator.goToPose(goal_pose)
        
        # Monitor progress
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                self.get_logger().info(f'ETA: {eta:.0f} seconds')
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Check result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
            return True
        else:
            self.get_logger().error('Failed to reach goal')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    planner = SimplePathPlanner()
    
    # Define waypoints
    waypoints = [
        (2.0, 2.0),
        (2.0, -2.0),
        (-2.0, -2.0),
        (-2.0, 2.0)
    ]
    
    # Navigate through waypoints
    for x, y in waypoints:
        success = planner.navigate_to_goal(x, y)
        if not success:
            break
    
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
