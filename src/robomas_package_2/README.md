ubuntu上でcan0を1000000ビットレートで起動するコマンド
```bash
sudo ip link set can0 up type can bitrate 1000000
```

sender_node起動コマンド
```bash
ros2 run robomas_package_2 sender
```

can_receiver起動コマンド
```bash
ros2 run robomas_package_2 receiver
```

試しのロボマス駆動コマンド （ロボマス１を電流制御で300mAで回す・ロボマス４を速度制御で500rpmで回す）
```bash
ros2 topic pub -1 /motor_cmd_array robomas_package_2/MotorCmdArray "
cmds:
- {id: 1, mode: 0, value: 300}
- {id: 4, mode: 1, value: 500}
"
```