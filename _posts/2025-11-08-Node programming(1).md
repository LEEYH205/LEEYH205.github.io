---
title: "Node Programmming(1)"
date: 2025-11-08 20:00:00 +0900
categories: [ROBOTICS]
tags: [Robotics, Node Programming]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---


# 사용할 로봇 소개

## PAL Robotics

- 스페인 바르셀로나에 본사를 둔 로봇 공학 회사로, 2004년에 설립
- 휴머노이드 로봇과 자율 로봇 시스템을 개발하는 데 주력하며, 특히 서비스 로봇, 물류 자동화, 의료 및 연구 목적으로 사용되는 로봇 솔루션을 제공

PMB2 (TIAGo Base)
![](/assets/img/image.png)
![](/assets/img/image%20(1).png)

모듈형 설계로 다양한 용도로 확장 가능


## 로봇 시뮬레이션 환경 구성
1. 시뮬레이션 환경 구성을 위해 아래 명령어로 패키지를 설치하기.
``` bash
sudo apt install ros-humble-pmb2-simulation
pip install transforms3d
```

2. 의존성 등을 포함한 설치가 완료된 후 시뮬레이션 환경 열기.
``` bash
ros2 launch pmb2_gazebo pmb2_gazebo.launch.py is_public_sim:=True world_name:=simple_office_with_people
```

![](/assets/img/Screenshot%20from%202025-11-08%2020-05-37.png)


---

# 문제점 분석하기

- 노드 역할 설명
    - 로봇은 로봇 앞에 0.5m 가까이 있는 장애물을 감지할 때까지 앞으로 이동
    - 로봇이 장애물과 0.5m 보다 가까워지면 로봇은 정지하고 방향을 바꾸기 위해 회전
    - 3가지 callback 함수 존재
        - odom_callback: 로봇의 현재 회전량
        - scan_callback: 로봇 기준으로 전방 거리를 측정하기 위한 용도
        - timer_callback: 로봇 제어하기 위한 용도


- 새로운 ROS2 패키지를 생성
```bash
ros2 pkg create --build-type ament_python node_exercise --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs
```

problem.py  (문제가 있는 python file)
``` python
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from transforms3d.euler import quat2euler


class RobotControl(Node):
    def __init__(self):
        super().__init__('problem_node')

        self.seconds_sleeping = 10 # 회전할 시간을 10초로 설정

        # Publisher
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_msg = Twist()

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry, 'mobile_base_controller/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan_raw', self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Timer
        self.timer = self.create_timer(0.3, self.timer_callback)

        self.front_laser = 0.0 # 전방 레이저 거리
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

    def odom_callback(self, msg: Odometry):
        self.get_logger().info("Odom CallBack")
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.roll, self.pitch, self.yaw = quat2euler(orientation_list)

    def scan_callback(self, msg: LaserScan):
        self.get_logger().info("Scan CallBack")
        self.front_laser = msg.ranges[359] # 전방 레이저 거리

    def stop_robot(self):
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.0
        self.vel_pub.publish(self.cmd_msg)

    def move_straight(self):
        self.cmd_msg.linear.x = 0.3
        self.cmd_msg.angular.z = 0.0
        self.vel_pub.publish(self.cmd_msg)

    def rotate(self):
        self.cmd_msg.angular.z = -0.4
        self.cmd_msg.linear.x = 0.0

        rotation_start_time = self.get_clock().now() # 회전 시작 시간 기록
        rotation_duration = Duration(seconds=self.seconds_sleeping)

        self.get_logger().info("Starting rotation for " + str(self.seconds_sleeping) + " seconds")

        while self.get_clock().now() - rotation_start_time < rotation_duration:
            self.vel_pub.publish(self.cmd_msg)

        self.get_logger().info("Rotation complete")
        self.stop_robot()

    def timer_callback(self):
        self.get_logger().info("Timer CallBack")
        if self.front_laser == 0.0: return

        try:
            self.get_logger().warning(">>> Front Range Value=" + str(self.front_laser))
            if self.front_laser > 0.5:
                self.get_logger().info("MOVE STRAIGHT")
                self.move_straight()
            else:
                self.get_logger().info("STOP ROTATE")
                self.stop_robot()
                self.rotate()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    control_node = RobotControl()
    try:
        rclpy.spin(control_node)
    finally:
        control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 문제의 동작 화면
![](/assets/img/Peek%202025-11-08%2020-44.gif)

```bash
[WARN]: >>> Front Range Value=0.5393399596214294
[INFO]: MOVE STRAIGHT
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Timer CallBack
[WARN]: >>> Front Range Value=0.4523996412754059
[INFO]: STOP ROTATE
[INFO]: Starting rotation for 10 seconds
[INFO]: Rotation complete
[INFO]: Timer CallBack
[WARN]: >>> Front Range Value=0.4523996412754059
[INFO]: STOP ROTATE
[INFO]: Starting rotation for 10 seconds
[INFO]: Rotation complete
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Timer CallBack
[WARN]: >>> Front Range Value=8.114912986755371
[INFO]: MOVE STRAIGHT
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Timer CallBack
[WARN]: >>> Front Range Value=6.737293720245361

```
거리 측정으로 `MOVE STRAIGHT`하면서,
`Odom CallBack`과 `can CallBack`, `Timer CallBack`이 잘 돌다가,

`if self.front_laser > 0.5:`조건에 따라, 전방 거리가 0.5보다 작아져,
(`>>> Front Range Value=0.4523996412754059`)

`STOP ROTATE`이 되면서,
`self.seconds_sleeping = 10`에 맞춰, `Starting rotation for 10 seconds`하고 있음.

근데, 10초가 지나서 `Rotation complete`가 되었음에도,
전방거리가 업데이트되지않아서, 이전 값을 사용해서 회전을 또 함.

```bash
[WARN]: >>> Front Range Value=0.4523996412754059
[INFO]: STOP ROTATE
[INFO]: Starting rotation for 10 seconds
[INFO]: Rotation complete
[INFO]: Timer CallBack
[WARN]: >>> Front Range Value=0.4523996412754059
[INFO]: STOP ROTATE
[INFO]: Starting rotation for 10 seconds
[INFO]: Rotation complete
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Timer CallBack
```

---

# 문제점 분석

## 상황 분석

- 실행 결과를 보면 아래와 같은 현상이 발생하는 것을 볼 수 있습니다.
    - 로봇이 회전 기능에 들어가자마자 `odom_callback()` 및 `scan_callback()` 실행이 중지됨
    - `odom_callback()` 및 `scan_callback()`이 `timer_callback()`에 의해 차단됨
    - 요약하면, 로봇이 회전 중일 때는 다른 콜백이 실행되지 않음

```python
while self.get_clock().now() - rotation_start_time < rotation_duration:
            self.vel_pub.publish(self.cmd_msg)
```
> python은 single thread로 도는데, 회전을 시작해서 `def rotate(self)`의 `while문`에 묶여있는 것임.


## 원인 분석

- **콜백 블로킹으로 인한 센서 데이터 수신 불가**
    - rotate() 함수의 while 루프가 메인 스레드를 차단
    - scan_callback과 odom_callback이 실행되지 못함
    - 결과적으로 10초 동안 로봇의 센서 데이터가 완전히 손실됨


## 이러한 문제가 미칠 수 있는 영향

- 센서 데이터의 최신화 문제
    - 현업 로봇 시스템에서는 센서 데이터가 너무 오래되면 **안전상의 이유로** 시스템이 **중지**되도록 함
    - 센서 데이터의 실시간성은 시스템 안전성의 핵심 요소
- 메인 스레드가 블로킹되어 다른 중요한 시스템 이벤트 처리 불가
- 긴급 정지 명령 등 중요한 제어 신호 수신 지연 가능성