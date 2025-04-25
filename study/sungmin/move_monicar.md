# monicar 움직이기

1. 경로 이동 및 소스

```bash
$ cd robot_ws/
$ source install/local_setup.bash
```

2. bringup

```bash
$ ros2 launch monicar_bringup mcu.launch.py
```

3. 터미널 창 하나 더 열고 cmd_vel 토픽 발행하기

```bash
// 계속 전진
$ ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

```bash
// 한번 살짝 돌기
$ ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.78}}"
```
