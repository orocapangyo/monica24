# 키보드로 monicar 움직이기

1. 터미널 2개를 열고, 각각 디렉토리를 이동하고 소스를 실행

```bash
$ cd robot_ws/
$ source install/local_setup.bash
```

2. 하나의 터미널에서 bringup 실행
```bash
$ ros2 launch monicar2_bringup mcu.launch.py
```

3. 다른 하나의 터미널에서 teleop 실행
```bash
$ ros2 run monicar2_teleop teleop_keyboard
```

4. w a s d x 를 이용하여 움직임을 제어하고, c로 led, z로 사운드, p로 디스플레이 바꾸기(디스플레이 납땜 되어있으면)
- 움직임 제어할때 키보드를 연속으로 누르거나 누르고 있지 말고, 키 하나씩 천천히 누르기
- 좌회전 우회전은 전진 상태에서 가능

  ![Screenshot from 2025-05-06 17-53-39](https://github.com/user-attachments/assets/fcaadbca-73a4-4f40-93de-bda111907890)
