---
title: "Nav2를 활용한 Path Planning 방법들"
date: 2025-12-18 10:00:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Path Planning]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

# Nav2를 통한 Path Planning 실습
- 로봇에게 목적지를 전달해주는 방법은 정말 다양합니다.
    - [[방법1] Rviz2의 버튼](#방법1-rviz2의-버튼)
    - [[방법2] Action Server](#방법2-action-server)
    - [[방법3] Topic](#방법3-topic)
    - [[방법4] 프로그래밍](#방법4-프로그래밍)




## [방법1] Rviz2의 버튼

1. 시뮬레이터 상의 2D LiDAR 시각화를 끄려면 아래 경로의 파일을 수정해줍니다.
    
    `~/nav2_ws/src/neuronbot2/neuronbot2_gazebo/models/neuronbot2/model.sdf`
    
    ```xml
    ...
    
    <!-- 362번째 줄의 **visualize를 false로 변경** -->
    <sensor name="rp_lidar_a1" type="ray">
        <always_on>true</always_on>
        **<visualize>false</visualize>**
        <pose>0.0 0 0.02 0 0 0</pose>
        <update_rate>5.5</update_rate>
        <ray>
    
    ...
    ```
    
2. 시뮬레이션 환경(bookstore)에 neuronbot2을 spawn해줍니다.
    
    ```bash
    ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py
    ```

    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-18%2010-40-52.png)
3. 사전에 만들어 놓은 지도를 불러오고 localization 활용하기 위해, `neuronbot2_nav` 패키지의 `bringup_launch.py` 파일을 위에서 [언급했던대로](https://www.notion.so/1dbdec25546e4a319c5a6c68610151ef?pvs=21) 수정해줍니다.
4. Path Planning 수행을 위한 아래 통합 런치 파일을 시작해줍니다.
    
    ```python
    ros2 launch neuronbot2_nav bringup_launch.py use_sim_time:=true
    ```
    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-18%2010-42-49.png)
5. 지도에서 로봇의 초기 위치가 올바르지 않을 경우, `2D Pose Estimate` 버튼을 클릭한 후 다음 시뮬레이터 상에서 로봇의 실제 위치에 해당하는 위치와 방향을 클릭해줍니다.

    
6. Rviz2의 `2D Goal Pose` 버튼으로 목적에 해당하는 위치와 방향을 클릭해 자율주행 결과를 확인합니다.
    
![](/assets/img/ros2nav2/gif/Peek%202025-12-18%2010-44.gif)


## [방법2] Action Server

- `/navigate_to_pose` 액션 서버를 활용 (`/clicked_point` 토픽으로 지도 상 위치 정보 획득 및 활용)

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose: {header: {frame_id: map}, pose: {position: {x: 1.52, y: 1.92, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0, w: 1.0000000}}}"
```
![](/assets/img/ros2nav2/gif/Peek%202025-12-18%2010-55.gif)

## [방법3] Topic

- `/goal_pose` 토픽을 활용 (`/clicked_point` 토픽으로 지도 상 위치 정보 획득 및 활용)

```bash
ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 2.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```
![](/assets/img/ros2nav2/gif/Peek%202025-12-18%2010-58.gif)
## [방법4] 프로그래밍

- `/navigate_to_pose` 액션 서버를 활용한 코드

```python
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('Nav_To_Pose_Action_Client')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.subscriber_ = self.create_subscription(PointStamped, 'clicked_point', self.callback, 1)

    def callback(self, msg):
        self.get_logger().info('Recieved Data:\n X : %f \n Y : %f \n Z : %f' % (msg.point.x, msg.point.y, msg.point.z))
        self.send_goal (msg.point.x, msg.point.y, 0.0)

    def send_goal(self, x ,y, theta):
        self.get_logger().info('sending goal to action server')
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = x
        goal_pose.pose.pose.position.y = y
        goal_pose.pose.pose.position.z = theta

        self.get_logger().info('waiting for action server')
        self._action_client.wait_for_server()
        self.get_logger().info('action server detected')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)
        self.get_logger().info('goal sent')

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}' + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('FEEDBACK:' + str(feedback) )

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```