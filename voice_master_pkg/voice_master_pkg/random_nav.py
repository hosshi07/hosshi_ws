import rclpy
from rclpy.node import Node
#使うサーバーのメッセージの型
from happy_interfaces.srv import Whisper, Piper, GoToLocation
import random


"""
このマスターはランダムにナビゲーションを行うモジュールである…
"""

location_points = ["living_room", "entrance_room", "kitchen_room", "chair1", "chair2", "chair3", "exit"]

class SimpleClient(Node):
    def __init__(self):
        super().__init__('voice_master')
        #こちらpiper（合成音声）のサーバーを指定
        self.tts_srv = self.create_client(Piper, '/tts/piper')
        #ウィスパー（音声聞き入れ）のサーバー指定
        self.stt_srv = self.create_client(Whisper, '/stt/whisper_service')
        
        self.nav_srv = self.create_client(GoToLocation, '/go_to_location')
    
    
    def go_nav(self, location):
        request = GoToLocation.Request()
        request.location = location
        self.nav_run = self.nav_srv.call_async(request)
        rclpy.spin_until_future_complete(self, self.nav_fur)
        return self.nav_run.result()

    #音声出力のメゾット　上で指定したサーバーにリクエストを送るもの．（今回はレスポンスを使わない）
    def speak_request(self, text):
        #リクエストをするための型のインスタンス化
        request = Piper.Request()
        #このようにリクエストのないようを入れる
        request.text = text
        #ここでサーバーにリクエストしている．　（非同期処理）
        self.fur = self.tts_srv.call_async(request)
        rclpy.spin_until_future_complete(self, self.fur)
    
    #文字起こしのメゾット　今回はリクエストはないがレスポンスをいただく
    def whisper_srv(self):
        #ウィスパーはリクエストないのでからである．ただしインスタンス化はする．
        re = Whisper.Request()
        #サーバーへリクエスト　（この段階では非同期）
        get_str = self.stt_srv.call_async(re)
        #サーバーの処理が終わるまで待つ（ブロック）　（これがあったら同期的な処理となる）
        rclpy.spin_until_future_complete(self, get_str)
        #返り値がレスポンス
        return get_str.result()


def main(args=None):
    rclpy.init(args=args)
    simple_client = SimpleClient()     #  クライアンとオブジェクトの生成  （pythonのインスタンス化とほぼ同じ）
    simple_client.get_logger().info("マスターをかいしします")
    simple_client.nav_run("entrance_room")
    simple_client.speak_request("hello my name is mimi. start navigation.")
    while True:
        random_location = random.choice(location_points)
        if random_location == "exit":
            break
        simple_client.speak_request(f"I going to go to {random_location}")
        simple_client.nav_run(random_location)
        simple_client.speak_request("Finish")
        
    
    
    simple_client.get_logger().info("finish")
    rclpy.shutdown()


if __name__ == '__main__':
    main()