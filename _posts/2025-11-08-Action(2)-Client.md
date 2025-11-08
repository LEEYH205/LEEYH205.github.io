---
title: "Action(2)-Client"
date: 2025-11-08 16:00:00 +0900
categories: [ROBOTICS]
tags: [Robotics, Action, Client]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---


- **액션을 호출한다는 것은 목표(Goal)를 액션 서버로 보내는 것을 의미함**
- 액션도 토픽 및 서비스와 마찬가지로 모두 메시지를 전달하는 방식으로 작동
    - 토픽이 제공하는 정보인 `메시지`는 단일 부분으로 구성
    - `서비스 타입`은 요청(Request)과 응답(Response)의 두 부분으로 구성
    - **`액션 타입`**은 **목표(Goal), 결과(Result), 피드백(Feedback) 세 부분으로 구성**
- 사용된 모든 액션 메시지는 해당 패키지의 `action` 디렉토리에 정의됨



# Action Client Node


- 목표
    - 앞선 실습에서 실행했던 `play_motion2_msgs/action/PlayMotion2` 타입의 `/play_motion2` 액션을 호출하는 **액션 클라이언트** 노드 생성

```
# goal
string motion_name
bool skip_planning

---

# result
bool success
string error

---

# feedback
builtin_interfaces/Time current_time
	int32 sec
	uint32 nanosec
```


![](/assets/img/Screenshot%20from%202025-11-08%2016-33-28.png)

play_motion_client.py
```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from play_motion2_msgs.action import PlayMotion2
import sys

class PlayMotionClient(Node):
    def __init__(self):
        super().__init__('play_motion_client')
        self.action_client = ActionClient(
            self,
            PlayMotion2,
            '/play_motion2',
        )
        self.get_logger().info('Play Motion Client has been started')

    def send_goal(self, motion_name, skip_planning=False):
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        # PlayMotion2 액션의 goal 메시지 생성
        goal_msg = PlayMotion2.Goal()
        goal_msg.motion_name = motion_name
        goal_msg.skip_planning = skip_planning

        self.get_logger().info(f'Sending goal: {motion_name} (skip_planning: {skip_planning})')

        # 비동기로 goal 전송
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        # 우리가 보낸 요청에 대한 future
        self.send_goal_future.add_done_callback(self.goal_response_callback)


    # add_done_callback(self.goal_response_callback)에서
    # self.goal_response_callback은 Action Server에서 내(Action Client)가 보낸 goal에 대해 승낙을 했는지에 대한 callback임.
    # 이걸 등록해줘야함.
    # (최종 결과에 대한 callback이 아님.)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        # 결과를 기다리기 위한 handler를 비동기로 만들어줌
        self.get_result_future = goal_handle.get_result_async()
        # 그 handler에 결과 수령을 위한 callback(self.get_result_callback)을 등록
        self.get_result_future.add_done_callback(self.get_result_callback)

    # Action Server 쪽에서 동작을 완료했을 때,
    # 그때, 우리는 get_result_callback을 통해서 결과가 들어올 것임.
    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Motion completed successfully!')
        else:
            self.get_logger().error(f'Motion failed with error: {result.error}')

        self.get_logger().info('Action completed')

    # 이것도 Action Server에서 줘야 받을 수 있는 것임.
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 현재 시간 출력 (초와 나노초)
        self.get_logger().info(
            f'Current time - sec: {feedback.current_time.sec}, '
            f'nanosec: {feedback.current_time.nanosec}'
        )

def main(args=None):
    rclpy.init(args=args)
    action_client = PlayMotionClient()

    # 명령줄 인자 처리
    motion_name = 'home'  # 기본값
    skip_planning = False  # 기본값

    if len(sys.argv) > 1:
        motion_name = sys.argv[1]
    if len(sys.argv) > 2:
        skip_planning = sys.argv[2].lower() == 'true'

    try:
        action_client.send_goal(motion_name, skip_planning)
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

callback 3개 씀.
```python
def goal_response_callback(self, future)
def get_result_callback(self, future)
def feedback_callback(self, feedback_msg)
```

액션 클라이언트 노드를 시작하고 결과를 확인
```bash
ros2 run tutorial_action play_motion_client
```
- 아래와 같은 사전 정의된 모션들 사용이 가능합니다.
    - home
    - head_tour
    - wave
    - reach_floor
    - reach_max

![](/assets/img/Peek%202025-11-08%2016-53.gif)
![](/assets/img/Peek%202025-11-08%2016-56.gif)