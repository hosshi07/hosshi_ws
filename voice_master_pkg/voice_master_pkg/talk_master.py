import rclpy
from rclpy.node import Node
from happy_interfaces.srv import Whisper, Piper


class SimpleClient(Node):
    def __init__(self):
        super().__init__('voice_master')
        self.tts_srv = self.create_client(Piper, '/piper')
        self.stt_srv = self.create_client(Whisper, '/whisper_service')

    def send_request(self, text):
        request = Piper.Request()
        request.text = text
        self.fur = self.tts_srv.call_async(request)
    
    def whisper_srv(self):
        re = Whisper.Request()
        get_str = self.stt_srv.call_async(re)
        rclpy.spin_until_future_complete(self, get_str)
        return get_str.result()
        
        



def main(args=None):
    rclpy.init(args=args)
    simple_client = SimpleClient()     #  クライアンとオブジェクトの生成  
    while True:
        re = simple_client.whisper_srv().result
        if re == 'exit':
            break
        simple_client.send_request(re)       # リクエストを送る
        print(simple_client.fur.done())
        if simple_client.fur.done():
            resp = simple_client.fur.result()
            simple_client.get_logger().info(f'結果：{resp.result}')
        
    simple_client.get_logger().info("finish")
    rclpy.shutdown()


if __name__ == '__main__':
    main()