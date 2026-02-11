import cv2
import numpy as np
import os
import rclpy
from rclpy.node import Node
from pathlib import Path

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("ball_detector")

        # パラメータ宣言
        self.declare_parameter("lower_blue", [100, 100, 50])
        self.declare_parameter("upper_blue", [140, 255, 255])

        self.declare_parameter("lower_red1", [0, 120, 70])
        self.declare_parameter("upper_red1", [10, 255, 255])
        self.declare_parameter("lower_red2", [170, 120, 70])
        self.declare_parameter("upper_red2", [180, 255, 255])

        timer_period = 0.033
        self.timer = self.create_timer(timer_period, self.timer_callback)

        base_dir = Path.home() / "haru26_ws/src/cvpkg/cvpkg"
        img_path = os.path.join(base_dir, "ball1.webp")

        if not os.path.exists(img_path):
            self.get_logger().error(f"ファイルが存在しません: {img_path}")
            exit()

        self.img = cv2.imread(img_path)
        self.hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

    def timer_callback(self):
        img = self.img.copy()
        hsv = self.hsv

        # パラメータ取得
        lower_blue = np.array(self.get_parameter("lower_blue").value, dtype=np.uint8)
        upper_blue = np.array(self.get_parameter("upper_blue").value, dtype=np.uint8)

        lower_red1 = np.array(self.get_parameter("lower_red1").value, dtype=np.uint8)
        upper_red1 = np.array(self.get_parameter("upper_red1").value, dtype=np.uint8)
        lower_red2 = np.array(self.get_parameter("lower_red2").value, dtype=np.uint8)
        upper_red2 = np.array(self.get_parameter("upper_red2").value, dtype=np.uint8)

        # マスク生成
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 | mask_red2

        # --- 青色の輪郭処理 ---
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_blue:
            area = cv2.contourArea(cnt)
            if area > 500:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)

                cv2.circle(img, center, radius, (255, 0, 0), 2)  # 青は青色で囲む
                cv2.putText(img, "Blue", (center[0] - 20, center[1] - radius - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        # --- 赤色の輪郭処理 ---
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_red:
            area = cv2.contourArea(cnt)
            if area > 500:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)

                cv2.circle(img, center, radius, (0, 0, 255), 2)  # 赤は赤色で囲む
                cv2.putText(img, "Red", (center[0] - 20, center[1] - radius - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("Detected Balls", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
