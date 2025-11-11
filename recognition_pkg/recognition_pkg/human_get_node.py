#このパッケージはこのように作った．
#ros2 pkg create recognition_pkg --build-type ament_python --node-name human_get_node --dependencies rclpy std_msgs sensor_msgs happy_interfaces 

from ultralytics import YOLOWorld, YOLO
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from custom_msgs.srv import StrBool
from sensor_msgs.msg import Image

from ultralytics import YOLOWorld
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
#サービス通信のモジュールを作る際にはサービスの型を作っておき，それを指定する．作るにはそれ専用のパッケージがいる．ament_cmake
from custom_msgs.srv import SetBool  #私はSetBoolを作った       request: なし、response:  string型のresult
from sensor_msgs.msg import Image


class HumanNode(Node):
    def __init__(self):
        super().__init__("human_get_node")
        #今回は画像を使うためプログラミングで使える形にする準備を記載
        self.bridge = CvBridge()
        #画像をサブスクする際にエラーが起きないようにするため
        self.current_frame = None
        #サブスクの準備、これがリアルセンスから取れる画像があるトピックサーバー　get_imageメゾットに画像は流れる
        self.img_sub = self.create_subscription(Image, "/camera/color/image_raw", self.get_image, 10)
        #ここでサーバーを建てる…　人間判定
        self.human_srv = self.create_service(SetBool, "/human_get", self.human_get)
        #yoloモデルを使う
        self.model = YOLOWorld("yolov8l-world.pt") 

        
    #imgに画像が入る
    def get_image(self, img):
        try:
            #プログラムでimgを扱える形に変換する
            #プログラムで扱える画像はself.crrent_frameに入ってる
            self.current_frame = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except:
            #トピックに画像がない場合の処理
            self.get_logger().info("Image conversion failed")
    
    #今回はリクエスとがなく、レスポンスがあるがinitメゾットで指定した以上、下のように書く…
    def human_get(self, request, response):
        #サーバーが起動するとyolo の推定をする‥それがresultに入る
        results = self.model(source=self.current_frame, conf=0.4)






def main(args=None):
    print('node_on')
    rclpy.init(args=args)
    get_node = Humanode()

    try:
        rclpy.spin(get_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
