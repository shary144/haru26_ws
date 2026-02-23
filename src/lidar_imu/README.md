自己位置推定の使い方

必要なもの(packages)(依存関係)
lidar_imu
urg_node2
odometry_plugins

IMUを起動
USBで通信

2DLiDARを起動
LANで通信
12Vの電源を接続


ros2 launch src/odometry_plugins/launch/odometry_bridge_launch.xml
IMUのデータをROSに送るnode
rosとしてIMUのデータを送る
urg_nodeのIMU版
これでIMUを使えるようにする

動かすと、いろいろ出る。

この時にros2 topic listを見ると、
/imu/data
/parameter_events
/rosoutというtopicが表示される

/imu/dataをechoすると、
linear_accelerationで、加速度（慣性（重力を1とする））を見られる
angular_velocityで、角速度を見られる

自己位置推定をアレンジしたかったら、/imu/dataをもとに、アレンジしてみましょう。



ros2 launch urg_node2 urg_node2.launch.py
これで、2DLiDARを使えるようにする。これに関しても、神田先輩は中身を知らないので、知らなくてよい。

これで、topicを見ると、いろいろある
/scanを見ると、いろいろ見られます。



ros2 launch lidar_imu lidar_imu_bringup.launch.py
二つを合体させますよ、というコード
上の２つを実行してから、これを実行しましょう。



この三つの奴でうまくいかなかったら、これをもう２、３回繰り返してみる。



<ソースファイルの説明>
imu_pubsub.cpp
imuの情報を使いやすいように整理する
オフセット誤差、というのがあるので、これをいい感じに調整する。
日によっても変わったりする。大変です。
実際に静止した状態の時に誤差があると思いますので、なんかうまくいかなかったら調整しましょう。
角速度と、加速度の誤差を調整する
imu_array、という場所に、6次元ベクトルとしてpublishしています。
callbackを三回に一回やっています。謎です。

lider_pubsub.cpp
これはよくわかってない。
2DLiDARが反転しているので、シータの符号を調整する
極座標を直交座標にしている。
使いやすいように処理している
2次元を一度1次元に落として送信してから2次元に組み立てる、という方式を取っているので改善の余地あり

imu_gyro.cpp
imuの角速度と加速度を積分して向きと位置を出す。（実際にはz成分が不要（z = 0））
ややこしいので説明なし
パラメーターの調整がややこしい
初期位置を打つところがある(170行目くらい)


icp_localization_node.cpp
robot_pose（7次元（位置とクウォータニオン））と、LiDARの値を用いて、修正した位置をlidar_to_imuで送る
100行くらいのところにmax_dist（ｍ）というのがある。これは除外点の閾値である。壁からの距離を計算する（現在は3cmである）。フィールドがでかくなったり、移動が速くなったりしたら、閾値を大きくする必要がありますね。
初期位置を打つところがある(310行目くらい。単位はm)。

上の２つ、初期位置を打つところがあるので注意しましょう。

mapについての説明
lidar_map.pyで、地図を作る。(実行するとfield.jpegで確認できる)
lidar_map.pyを実行する。その後に出力結果をbox_data.hppに{}にして入れる（ここまででフィールドの準備）

colcon buildをする
ros2 launch lidar_imu lidar_imu_bringup.launch.pyを実行
その後、rviz2を実行すると、フィールド図を見られる。

同じコマンドを二つ以上実行しないように気を付けましょう。同じやつを2つ同時に実行すると、rviz2上で見たときにロボット（直方体）がぴくぴくします。


