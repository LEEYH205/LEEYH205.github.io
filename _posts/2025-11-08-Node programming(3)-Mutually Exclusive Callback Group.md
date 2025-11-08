---
title: "2025-11-08-Node programming(3)-Mutually Exclusive Callback Group"
date: 2025-11-08 21:20:00 +0900
categories: [ROBOTICS]
tags: [Robotics, Node Programming, Executor, Callback Group, Mutually Exclusive Callback Group]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# 문제점 요약


- 로봇이 회전 기능에 들어가자마자 `odom_callback()` 및 `scan_callback()` 실행이 중지됨
- `odom_callback()` 및 `scan_callback()`이 **`timer_callback()`**에 의해 차단됨
- 요약하면, 로봇이 회전 중일 때는 다른 콜백이 실행되지 않음


# **Mutually Exclusive Callback Group를 통한 문제 해결**


## **[실험1] *SingleThreadedExecutor()와 MutuallyExclusiveCallbackGroup() 조합***


solution1.py

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from transforms3d.euler import quat2euler

# Import the libraries to use executors and callback groups
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

class RobotControl(Node):
    def __init__(self):
        super().__init__('solution1_node')

        self.seconds_sleeping = 10 # 회전할 시간을 10초로 설정

        # Publisher
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_msg = Twist()

        # Callback group
        self.group = MutuallyExclusiveCallbackGroup()

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry, 'mobile_base_controller/odom', self.odom_callback, 10,
            callback_group=self.group)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan_raw', self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.group)

        # Timer
        self.timer = self.create_timer(0.3, self.timer_callback, callback_group=self.group)

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

    # Create a single-threaded executor
    executor = SingleThreadedExecutor()
    executor.add_node(control_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

setup.py
```python
entry_points={
    'console_scripts': [
        'problem_node = node_exercise.problem:main',
        'solution1_node = node_exercise.solution1:main',
    ],
},
```
![](/assets/img/Peek%202025-11-08%2021-28.gif)

>당연히 작동안함. 콜백 그룹을 써도 다시 반복 되는 것을 볼 수 있음.

## 이유
### 1. 멀티 스레드를 사용하지 않아서.
- `SingleThreadedExecutor()`
- 하나의 스레드가 담당하다보니까 그 하나의 스레드가 타이머 콜백에 묶여있어서, 다른 두 개의 콜백이 작동하지 않음.
```python
...
control_node = RobotControl()

# Create a single-threaded executor
executor = SingleThreadedExecutor()
executor.add_node(control_node)

try:
    executor.spin()
    ...
...
```
### 2. 콜백 그룹을 묶는 방식의 차이.
- `MutuallyExclusiveCallbackGroup()`
- 세 가지의 콜백을 하나의 콜백 그룹으로 묶었음.
- 그룹 내에서는 한 번에 하나의 콜백만 실행이 가능하다.
```python
...
# Callback group
self.group = MutuallyExclusiveCallbackGroup()

# Subscriber
self.odom_sub = self.create_subscription(
    Odometry, 'mobile_base_controller/odom', self.odom_callback, 10,
    callback_group=self.group)
self.scan_sub = self.create_subscription(
    LaserScan, 'scan_raw', self.scan_callback,
    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
    callback_group=self.group)

# Timer
self.timer = self.create_timer(0.3, self.timer_callback, callback_group=self.group)
...
```

## [실험2] MultiThreadedExecutor()와 MutuallyExclusiveCallbackGroup() 조합

solution2.py
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile, HistoryPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from transforms3d.euler import quat2euler

# Import the libraries to use executors and callback groups
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

class RobotControl(Node):
    def __init__(self):
        super().__init__('solution2_node')

        self.seconds_sleeping = 10 # 회전할 시간을 10초로 설정

        # Publisher
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_msg = Twist()

        # Callback group
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry, 'mobile_base_controller/odom', self.odom_callback, 10,
            callback_group=self.group1)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan_raw', self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,),
            callback_group=self.group2)

        # Timer
        self.timer = self.create_timer(0.3, self.timer_callback, callback_group=self.group3)

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

    # Create a multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(control_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

`MutuallyExclusiveCallbackGroup()`를 3개를 한다.

```python
# Callback group
...
self.group1 = MutuallyExclusiveCallbackGroup()
self.group2 = MutuallyExclusiveCallbackGroup()
self.group3 = MutuallyExclusiveCallbackGroup()

# Subscriber
self.odom_sub = self.create_subscription(
    Odometry, 'mobile_base_controller/odom', self.odom_callback, 10,
    callback_group=self.group1)
self.scan_sub = self.create_subscription(
    LaserScan, 'scan_raw', self.scan_callback,
    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,),
    callback_group=self.group2)

# Timer
self.timer = self.create_timer(0.3, self.timer_callback, callback_group=self.group3)
...
```

`MultiThreadedExecutor()`를 사용한다.
```python
control_node = RobotControl()

# Create a multi-threaded executor
executor = MultiThreadedExecutor(num_threads=3)
executor.add_node(control_node)

try:
    executor.spin()
```

![](/assets/img/Peek%202025-11-08%2021-43.gif)

- 이번에는 **odom_callback(), scan_callback(), timer_callback()**에 대해 하나씩 세 개의 서로 다른 **`MutuallyExclusiveCallbackGroup()`**을 생성했습니다.
- 또한 executor를 **`MultiThreadedExecutor()`**로 변경하고 스레드 3개를 할당해줬습니다.
    - 필요에 따라 각 그룹에 콜백을 배치합니다.
    - 그룹이 많을수록 더 많은 스레드가 필요하므로 더 많은 리소스가 필요합니다.

```bash
[WARN]: >>> Front Range Value=0.558488130569458
[INFO]: MOVE STRAIGHT
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
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Timer CallBack
[WARN]: >>> Front Range Value=0.4978385865688324
[INFO]: STOP ROTATE
[INFO]: Starting rotation for 10 seconds
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
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
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
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
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Rotation complete
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Timer CallBack
[WARN]: >>> Front Range Value=8.57592487335205
[INFO]: MOVE STRAIGHT
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Scan CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
[INFO]: Odom CallBack
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
[INFO]: Odom CallBack
[INFO]: Timer CallBack
[WARN]: >>> Front Range Value=9.080524444580078
```

Callback Group에 대한 추가 정보 : https://docs.ros2.org/crystal/api/rclpy/api/execution_and_callbacks.html#module-rclpy.callback_groups


