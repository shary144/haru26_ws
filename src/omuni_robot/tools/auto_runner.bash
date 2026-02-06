#まだ制作中なので実行しないで
#!/bin/bash
set -euo pipefail

# スクリプトの場所
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ros2_ws のルート（tools → omuni2rin → src → ros2_ws）
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# 初回ビルド
if [ ! -d "$WS_DIR/install" ]; then
    echo "初回ビルドを実行します..."
    colcon build --symlink-install
else
    echo "ビルドしますか？ [y/N]"
    read ans
    if [[ "$ans" == "y" ]]; then
        colcon build --symlink-install
    fi
fi

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

sudo ip link set can0 up type can bitrate 1000000

# ROS2 ノード起動（通常ユーザー）
ros2 launch robomas_package_2 robomas_launch.py &
ros2 run joy joy_node &
ros2 run omuni_robot omuni_3rin_node &

# 一応起動用
# ros2 topic echo /joy