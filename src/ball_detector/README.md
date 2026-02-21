# ボール検出用のやつ
2025/12/02更新
# 使い方
①カメラをつなぐ

②環境変数の設定(``./opt/ros/jazzy/setup.bash``など)

③```ros2 run ball_detector ball_detector_node```とターミナルに打つと起動

④もしくは```ros2 launch ball_detector debug_launch.py```とするとdebugモードで起動

# git clone した後に変更すべき場所
src/main.cppの様々な部分を変えなければならない
36行目の
```"/home/crs3/camera_ws/camera_ws/fry_ws/src/ball_detector/camera_calib.yml"```
をお手元にあるcamera_calib.ymlファイルのパスに変更

103~125行目のhsv値をdebugモードをつかって上手く設定

202行目のボールの半径を変更

267行目にカメラが下向き何°かを入力

268~269はゴミです。カメラは正面には向けてください。
気が向いたら実装します。

270~272行目に、自己位置制御点からのカメラのオフセット偏差を入力