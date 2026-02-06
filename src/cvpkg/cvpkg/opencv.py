# カメラを起動して、画像をリアルタイムで読み取ってもらう。pythonで実行する用（ROS2で実行できるように変更する必要あり）
import cv2
import numpy as np

# カメラを起動
cap = cv2.VideoCapture(0)

# 画像読み込み
if not cap.isOpened():
    print("カメラが開けません")
    exit()

while True:
    # 映像をフレームごとに読み込む
    ret, frame = cap.read()

    if not ret:
        print("フレームが取得できません")
        break

    img = frame

    # HSV変換
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

    # 結果表示
    cv2.imshow("Detected Balls", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()