#!/usr/bin/env python3
import time
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped

STOP_NAVIGATION_NOW = False
NAV_IN_PROGRESS = False
MOVING_FORWARD = True
COSTMAP_CLEARING_PERIOD = 0.5

class GoToGoalPose(Node):
    def __init__(self):
        super().__init__('go_to_goal_pose')
        
        self.publisher_eta = self.create_publisher(String, '/goal_pose/eta', 10)
        self.publisher_status = self.create_publisher(String, '/goal_pose/status', 10)
        self.subscription_go_to_goal_pose = self.create_subscription(
            PoseStamped, '/goal_pose/goal', self.go_to_goal_pose, 10)
        
        self.current_time = self.get_clock().now().nanoseconds
        self.last_time = self.current_time
        self.dt = (self.current_time - self.last_time) * 1e-9
        self.costmap_clearing_period = COSTMAP_CLEARING_PERIOD
        
        self.navigator = BasicNavigator()
        self.get_logger().info('Waiting for Nav2 to activate...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active!')
    
    def go_to_goal_pose(self, msg):
        global NAV_IN_PROGRESS, STOP_NAVIGATION_NOW
        
        self.navigator.clearAllCostmaps()
        self.navigator.goToPose(msg)
        
        while rclpy.ok() and not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            
            if feedback:
                estimated_time = f"{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.0f}"
                msg_eta = String()
                msg_eta.data = str(estimated_time)
                self.publisher_eta.publish(msg_eta)
                
                msg_status = String()
                msg_status.data = "IN_PROGRESS"
                self.publisher_status.publish(msg_status)
                NAV_IN_PROGRESS = True
                
                if STOP_NAVIGATION_NOW:
                    self.navigator.cancelTask()
                    self.get_logger().info('Navigation canceled')
                    STOP_NAVIGATION_NOW = False
                    NAV_IN_PROGRESS = False
        
        result = self.navigator.getResult()
        msg_status = String()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
            msg_status.data = "SUCCEEDED"
        elif result == TaskResult.CANCELED:
            msg_status.data = "CANCELED"
        elif result == TaskResult.FAILED:
            msg_status.data = "FAILED"
        
        self.publisher_status.publish(msg_status)

class GetCurrentVelocity(Node):
    def __init__(self):
        super().__init__('get_current_velocity')
        self.subscription_current_velocity = self.create_subscription(
            Twist, '/cmd_vel', self.get_current_velocity, 1)
    
    def get_current_velocity(self, msg):
        global MOVING_FORWARD
        MOVING_FORWARD = msg.linear.x > 0.0

def main(args=None):
    rclpy.init(args=args)
    
    try:
        go_to_goal_pose = GoToGoalPose()
        get_current_velocity = GetCurrentVelocity()
        
        executor = MultiThreadedExecutor()
        executor.add_node(go_to_goal_pose)
        executor.add_node(get_current_velocity)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            go_to_goal_pose.destroy_node()
            get_current_velocity.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
