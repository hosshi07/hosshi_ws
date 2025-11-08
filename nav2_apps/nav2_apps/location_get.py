#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from happy_interfaces.srv import Location, GetLocation
import yaml
import os

class WaypointSaver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')

        # 現在のPoseを購読
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  
            self.pose_callback,
            10
        )


        self.save_sub = self.create_service(
            Location,
            '/save_pose',
            self.save_location
        )

        self.remove_sub = self.create_service(
            Location,
            '/remove_pose',
            self.remove_location
        )

        self.list_sub = self.create_service(
            GetLocation, 
            '/location_list',
            self.location_list
        )
        
        self.current_pose = None
        self.map_file_env = os.getenv("HAPPY_MAP", "arc25")
        map_name = self.map_file_env
        self.get_logger().info(f'マップの名称：{map_name}.yaml')
        self.yaml_path = os.path.expanduser(f'~/happy_ws/src/happy_params/location/{map_name}.yaml')
        
        self.get_logger().info('📍 WaypointSaver Node Started')
        self.get_logger().info('server name is /save_pose')
        self.get_logger().info('msg is Location.srv')
        self.get_logger().info('request is "location"')
        

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def location_list(self, request, response):
        data = {}
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as f:
                try:
                    data = yaml.safe_load(f) or {}
                except yaml.YAMLError:
                    data = {}
        
        location_names = list(data)
        self.get_logger().info('Location list-----')
        # for name in location_names:
        #     self.get_logger().info(name)
        if location_names:
            self.get_logger().info(f"登録済みロケーション: {', '.join(location_names)}")
        else:
            self.get_logger().info("登録済みロケーションはありません。")


        response.location_list = location_names
        return response
    
    def remove_location(self, request, response):
        name = request.location
        data = {}
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as f:
                try:
                    data = yaml.safe_load(f) or {}
                except yaml.YAMLError:
                    data = {}

        try:
            del data[name]
        except KeyError:
            self.get_logger().error('ロケーションがありません')
            response.success = False
            return response
        
        
        
        with open(self.yaml_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f'✅ {name} を保存しました: {self.yaml_path}')
        response.success = True
        return response


    def save_location(self, request, response):
        if self.current_pose is None:
            self.get_logger().warn('❌ 現在位置がまだ取得できていません (/amcl_pose を確認)')
            response.success = False
            return response

        map_name = self.map_file_env
        self.yaml_path = os.path.expanduser(f'~/happy_ws/src/happy_params/location/{map_name}.yaml')
        
        name = request.location
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
        os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)
        with open(self.yaml_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

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
