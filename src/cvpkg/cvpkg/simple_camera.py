# カメラ検証用

import rclpy
from rclpy.node import Node
import cv2

class SimpleCamera(Node):
    def __init__(self):
        super().__init__('simple_camera')
        
        # 1. カメラを開く (0 は標準カメラ。USBカメラなら 2 や 4 の場合もあり)
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error("カメラが見つかりません！")
            exit()

        # 2. タイマーを作成 (0.033秒 = 約30FPS で callback を呼ぶ)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        # カメラから1フレーム読み込む
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warning("画像を読み込めませんでした")
            return

        # --- ここに画像処理を書くことができます ---
        # 例: 前回のボール検出のコードをここに書けば、リアルタイム検出になります！
        
        # 3. 画面に表示
        cv2.imshow("Camera View", frame)
        
        # 4. キー入力を少し待つ (これがないとウィンドウが更新されません)
        cv2.waitKey(1)

    def __del__(self):
        # 終了時にカメラを解放
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCamera()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了処理
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()