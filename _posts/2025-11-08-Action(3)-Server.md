---
title: "Action(3)-Server"
date: 2025-11-08 17:00:00 +0900
categories: [ROBOTICS]
tags: [Robotics, Action, Server]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# Action Server Node


- 목표
    - `turtlesim/action/RotateAbsolute` 액션 타입을 활용하여 TIAGo 로봇의 회전을 제어하는 액션 서버 노드 생성
    ```bash
    ros2 interface show turtlesim/action/RotateAbsolute
    ```
    ![](/assets/img/Screenshot%20from%202025-11-08%2017-00-40.png)

- 시나리오
    - 위 액션 타입을 활용하여 `/rotate_tiago` 액션을 생성
    - 클라이언트로부터 goal을 받으면, theta 만큼 로봇을 회전시키되 오도메트리 정보를 활용하지 않고 최대한 간단하게 구현
        > Odometry : 기준점부터 로봇이 얼마나 이동하고, 얼마나 회전했는지를 나타내는 것

rotate_server.py
```python
import rclpy
from rclpy.action import ActionServer       # ActionServer 가져옴.
from rclpy.node import Node
from turtlesim.action import RotateAbsolute
from geometry_msgs.msg import Twist
import time

class RotateActionServer(Node):
    def __init__(self):
        super().__init__('rotate_action_server')

        # Action server 초기화
        self.action_server = ActionServer(
            self,
            RotateAbsolute,
            'rotate_tiago',
            self.execute_callback
        )

        # cmd_vel 퍼블리셔 설정
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 현재 회전 각도 변수
        self.current_angle = 0.0

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal to rotate to angle: %f' % goal_handle.request.theta)

        # 목표 각도와 현재 각도 차이 계산
        target_angle = goal_handle.request.theta
        angle_diff = target_angle - self.current_angle

        # 회전 방향 및 속도 설정
        twist_msg = Twist()
        twist_msg.angular.z = 0.5 if angle_diff > 0 else -0.5

        # 피드백 메시지 초기화
        feedback_msg = RotateAbsolute.Feedback()

        # 목표 각도에 도달할 때까지 반복
        while abs(angle_diff) > 0.01:
            self.cmd_pub.publish(twist_msg)

            # 각도 차이 업데이트
            angle_diff -= 0.01 * twist_msg.angular.z  # 간단한 각도 차이 갱신

            # 피드백 메시지 업데이트 및 publish
            feedback_msg.remaining = abs(angle_diff)
            goal_handle.publish_feedback(feedback_msg)

            # 각도 도달 확인
            if abs(angle_diff) <= 0.01:
                break

            time.sleep(0.01)

        # 목표에 도달하면 정지
        twist_msg.angular.z = 0.0
        self.cmd_pub.publish(twist_msg)

        # 현재 각도 초기화
        self.current_angle = 0.0

        # 목표 도달 및 응답
        goal_handle.succeed()
        result = RotateAbsolute.Result()
        result.delta = angle_diff

        self.get_logger().info('Goal achieved with delta: %f' % result.delta)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RotateActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

callback 1개
```python
def execute_callback(self, goal_handle)
```

Goal 들어오면 goal 안에서 값 가져오고, 처리하고, Feedback주고, 결과 반환함


```python
# 현재 포즈에서 반시계 방향으로 약 180도 회전을 위한 명령어
ros2 action send_goal -f /rotate_tiago turtlesim/action/RotateAbsolute "{theta: 3.141592}"

# 현재 포즈에서 시계 방향으로 약 90도 회전을 위한 명령어
ros2 action send_goal -f /rotate_tiago turtlesim/action/RotateAbsolute "{theta: -1.5708}"
```

![](/assets/img/Peek%202025-11-08%2017-26.gif)
> 지금은 odometry 값 바탕으로 회전한게 아니라, 속도값에다가 시간 곱해서 변위를 얻어서 회전한 것이기 때문에, 정확하게 돌지 않았음.

