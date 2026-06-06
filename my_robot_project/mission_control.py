import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class RoverMissionControl(Node):
    def __init__(self):
        super().__init__('rover_mission_control')
        
        # Waypoints for the Mars Rover mission
        self.waypoints = [
            (12.0, 0.0), (12.0, 12.0), (0.0, 12.0),
            (-12.0, 12.0), (-12.0, 0.0), (0.0, 0.0)
        ]
        self.current_idx = 0
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║  Mars Rover Nav2 Mission Control         ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')
        self.get_logger().info('Waiting for NavigateToPose action server...')
        
    def start_mission(self):
        # Wait for Nav2 to be fully active
        server_ready = False
        while not server_ready and rclpy.ok():
            if self.nav_client.wait_for_server(timeout_sec=1.0):
                server_ready = True
            else:
                self.get_logger().info('Nav2 server not ready yet, retrying...')
        
        self.get_logger().info('Nav2 server connected! Starting mission...')
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('MISSION COMPLETE! All waypoints reached.')
            return

        x, y = self.waypoints[self.current_idx]
        self.get_logger().info(f'🚀 Sending Rover to Waypoint {self.current_idx + 1}/{len(self.waypoints)}: ({x}, {y})')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # Face the direction of travel (approximate)
        goal_msg.pose.pose.orientation.w = 1.0 

        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2! Retrying in 5s...')
            time.sleep(5.0)
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted. Navigating...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4: # GoalStatus.STATUS_SUCCEEDED (imported from action_msgs.msg might be safer but 4 is standard)
            self.get_logger().info(f'✅ Waypoint {self.current_idx + 1} reached successfully!')
            self.current_idx += 1
            # Short pause before next waypoint
            time.sleep(2.0)
            self.send_next_goal()
        else:
            self.get_logger().warn(f'Waypoint navigation failed with status {status}. Retrying...')
            time.sleep(2.0)
            self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = RoverMissionControl()
    
    # Wait for initial systems to settle
    time.sleep(5.0)
    
    node.start_mission()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
