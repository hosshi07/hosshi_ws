#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from happy_interfaces.srv import Location
import yaml
import os

class WaypointSaver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')

        # 現在のPoseを購読
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # ほんとにここなのか
            self.pose_callback,
            10
        )

        # self.save_sub = self.create_subscription(
        #     String,
        #     '/save_pose',
        #     self.save_callback,
        #     10
        # )
        
        self.save_sub = self.create_service(
            Location,
            '/save_pose',
            self.save_location
        )

        self.current_pose = None
        


        self.get_logger().info('📍 WaypointSaver Node Started')
        self.get_logger().info('server name is /save_pose')
        self.get_logger().info('msg is Location.srv')
        self.get_logger().info('request is "map" and "location"')
        

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def save_location(self, requset, response):
        if self.current_pose is None:
            self.get_logger().warn('❌ 現在位置がまだ取得できていません (/amcl_pose を確認)')
            return

        map_name = requset.map
        self.yaml_path = os.path.expanduser(f'~/happy_ws/src/happy_params/location/{map_name}.yaml')
        
        name = requset.location
        pose = {
            'position': {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'z': self.current_pose.position.z
            },
            'orientation': {
                'x': self.current_pose.orientation.x,
                'y': self.current_pose.orientation.y,
                'z': self.current_pose.orientation.z,
                'w': self.current_pose.orientation.w
            }
        }

        data = {}
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as f:
                try:
                    data = yaml.safe_load(f) or {}
                except yaml.YAMLError:
                    data = {}

        data[name] = pose

        with open(self.yaml_path, 'w') as f:
            yaml.dump(data, f)

        self.get_logger().info(f'✅ {name} を保存しました: {self.yaml_path}')
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
