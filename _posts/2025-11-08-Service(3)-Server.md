---
title: "Service(3)-Server"
date: 2025-11-08 15:00:00 +0900
categories: [ROBOTICS]
tags: [Robotics, Service, Server]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# Service  - Server

- 목표
    - `std_srvs/srv/SetBool` 인터페이스 타입을 활용하여 TIAGo 로봇을 제어하는 서비스 서버 노드 생성
    ```
    ros2 interface show std_srvs/srv/SetBool
    ```
- 시나리오
    - `/tiago_move` 서비스를 생성 (`std_srvs/srv/SetBool` 활용)
    - 서비스 호출을 받으면, 요청된 상태(True/False)에 따라 로봇의 이동을 시작하거나 중지
        - True: 계속 원을 그리며 움직임
        - False: 기존 움직임을 정지
    - 10Hz의 주기로 로봇에 속도 명령을 publish


```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist

class TiagoMoveServer(Node):
    def __init__(self):
        super().__init__('tiago_move_server')
        self.srv = self.create_service(SetBool, 'tiago_move', self.tiago_move_callback)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.is_moving = False # Flag to check if TIAGo is moving
        self.get_logger().info('TIAGo Move Server has been started')

    def tiago_move_callback(self, request, response):
        self.is_moving = request.data

        if self.is_moving:
            response.message = "TIAGo starts moving in a circle"
            self.get_logger().info('TIAGo starts moving in a circle')
        else:
            response.message = "TIAGo stops moving"
            self.get_logger().info('TIAGo stops moving')

        response.success = True # Set response success to True
        return response

    def timer_callback(self):
        msg = Twist()
        if self.is_moving:
            # Move in a circle
            msg.linear.x = 0.5  # 0.5 m/s
            msg.angular.z = 0.7  # 0.7 rad/s
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    tiago_move_server = TiagoMoveServer()
    rclpy.spin(tiago_move_server)
    tiago_move_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


```python
# setup.py에 추가
entry_points={
    'console_scripts': [
        'empty_service_client = tutorial_service.empty_service_client:main',
        'tiago_move_server = tutorial_service.tiago_move_server:main', # 추가!
    ],
},
```

![](/assets/img/Peek_2025-11-08_15-05.gif)