import rclpy
from rclpy.node import Node
#使うサーバーのメッセージの型
from happy_interfaces.srv import Whisper, Piper


class SimpleClient(Node):
    def __init__(self):
        super().__init__('voice_master')
        #こちらpiper（合成音声）のサーバーを指定
        self.tts_srv = self.create_client(Piper, '/tts/piper')
        #ウィスパー（音声聞き入れ）のサーバー指定
        self.stt_srv = self.create_client(Whisper, '/stt/whisper_service')

    #音声出力のメゾット　上で指定したサーバーにリクエストを送るもの．（今回はレスポンスを使わない）
    def speak_request(self, text):
        #リクエストをするための型のインスタンス化
        request = Piper.Request()
        #このようにリクエストのないようを入れる
        request.text = text
        #ここでサーバーにリクエストしている．　（非同期処理）
        self.fur = self.tts_srv.call_async(request)
    
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
    while True:
        #指定したメゾットを呼び出してる．これはウィスパー　このとき文字をreに代入してる
        re = simple_client.whisper_srv().sentences
        #文字起こししたものをターミナルへ表示．
        simple_client.get_logger().info(re)
        if re == 'exit':
            break
        
        #文字起こししたものを合成音声メゾットに送っている
        simple_client.speak_request(re)       # リクエストを送る
    #おしまい
    simple_client.get_logger().info("finish")
    rclpy.shutdown()


if __name__ == '__main__':
    main()