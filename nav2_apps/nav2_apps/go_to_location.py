import rclpy
from rclpy.node import Node
import threading # smachを別スレッドで動かすため
import yaml
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from happy_interfaces.srv import GoToLocation

# from rclpy.executors import MultiThreadedExecutorc
from rclpy.executors import MultiThreadedExecutor



class GoToNode(Node):
    def __init__(self):
        super().__init__('go_to_location')
        
        # --- (ここから) navigation.py の機能をコピー ---
        self.go_srv = self.create_service(GoToLocation, '/go_to_location', self.execute_srv)
        self.navigator = BasicNavigator()
        self.declare_parameter('waypoint_file', '') # Launchファイルから受け取る
        self.waypoints = self.load_waypoints()
        

        if not self.waypoints:
            self.get_logger().error("ウェイポイントが読み込めませんでした。")
            return
            
        self.get_logger().info("Nav2の起動を待機中...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2がアクティブになりました。")
    
    def load_waypoints(self):
        """(navigation.py からコピー) YAMLファイルを読み込む"""
        filepath = self.get_parameter('waypoint_file').get_parameter_value().string_value
        if not filepath:
            self.get_logger().error('Waypoint file path is not set!')
            return {}
        try:
            with open(filepath, 'r') as file:
                waypoint_data = yaml.safe_load(file)
                self.get_logger().info(f"Successfully loaded waypoints from: {filepath}")
                # "locations" (複数形) を使用（前回修正したYAMLに合わせる）
                return waypoint_data.get('locations', {}) 
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoint file: {e}")
            return {}
    
    def go_to_location(self, location_name):
        """指定された場所へ移動"""
        if location_name not in self.waypoints:
            self.get_logger().error(f"❌ Location '{location_name}' is not defined.")
            return False

        self.get_logger().info(f"Navigating to '{location_name}'...")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        pose_data = self.waypoints[location_name]['pose']

        goal_pose.pose.position.x = float(pose_data['position']['x'])
        goal_pose.pose.position.y = float(pose_data['position']['y'])
        goal_pose.pose.position.z = float(pose_data['position'].get('z', 0.0))
        goal_pose.pose.orientation.x = float(pose_data['orientation'].get('x', 0.0))
        goal_pose.pose.orientation.y = float(pose_data['orientation'].get('y', 0.0))
        goal_pose.pose.orientation.z = float(pose_data['orientation']['z'])
        goal_pose.pose.orientation.w = float(pose_data['orientation']['w'])

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"✅ Arrived at '{location_name}'")
            return True
        else:
            self.get_logger().warn(f"⚠️ Failed to reach '{location_name}' (result={result})")
            return False

        
    def execute_srv(self, request, response):
        location_name = request.location
        response.success = self.go_to_location(location_name)
        return response




def main(args=None):
    rclpy.init(args=args)
    
    go_node = GoToNode()

    if not go_node.waypoints:
        go_node.get_logger().fatal("ウェイポイントが読み込めなかったため、プランナーを起動できません。")
        go_node.destroy_node()
        rclpy.shutdown()
        return


    executor = MultiThreadedExecutor()
    executor.add_node(go_node)
    
    # rclpy.spin() の代わりに executor.spin() を使用
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        go_node.navigator.shutdown()
        go_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
