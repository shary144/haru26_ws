import cv2
import numpy as np
import os
import rclpy
from rclpy.node import Node
from pathlib import Path


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("ball_detector")  # ← 必須

# ---------------------------------------------------------
        # 【修正ポイント】
        # 自分のファイル(__file__)の場所を取得して、そこを基準にする
        # ---------------------------------------------------------
        base_dir = Path.home() / "haru26_ws"
        img_path = 
        img_path = "/home/kouki/haru26_ws/src/cvpkg/cvpkg/ball1.webp"
        
        # デバッグ用：実際にどこを探しているか確認する
        print(f'Looking for image at: {img_path}') 
        
        # ファイルが存在するか確認（念の為）
        if not os.path.exists(img_path):
             self.get_logger().error(f"ファイルが存在しません: {img_path}")
             # ここで終了せずにreturnなどで抜けるのが安全ですが、テストならexitでもOK
             exit()

        img = cv2.imread(img_path)
        # （画像処理はそのまま）
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # --- 青色の範囲 ---
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # --- 赤色の範囲（2つ必要） ---
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # 赤のマスクを合成
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
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main(args=None):          # ← クラスの外！
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
