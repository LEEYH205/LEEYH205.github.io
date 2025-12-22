---
title: "Simple Commander API - Follow Waypoints"
date: 2025-12-21 11:10:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Simple Commander API, Follow Waypoints]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# **[Nav2 Action 3] Follow Waypoints**

---

- 각 웨이포인트에서 멈추고 동작을 실행하려는 간단한 작업에 가장 적합합니다.
    - 예: 2초 동안 일시 정지, 사진 촬영, 누군가 상자를 놓을 때까지 기다리기 등
- `FollowWaypoints.action`의 기본 구조는 아래와 같습니다.
    
    ```yaml
    # goal definition
    geometry_msgs/PoseStamped[] poses
    ---
    # result definition
    int32[] missed_waypoints
    ---
    # feedback definition
    uint32 current_waypoint
    ```
    
    - 피드백(current_waypoint)은 실행 중인 현재 웨이포인트 ID이며, 마지막에 탐색할 수 없는 웨이포인트가 있는 경우 누락된 웨이포인트 ID의 벡터를 반환합니다.
- Nav2의 **`waypoint_follower`** 설정에는 각 웨이포인트에서 작업을 실행할 수 있는 `TaskExecutor` 플러그인이 포함되어 있습니다.
    - 본 실습에서는 [`WaitAtWaypoint`](https://docs.nav2.org/configuration/packages/nav2_waypoint_follower-plugins/wait_at_waypoint.html) 플러그인을 활용할 예정입니다.
        - 그 외에도 [`PhotoAtWaypoint`](https://docs.nav2.org/configuration/packages/nav2_waypoint_follower-plugins/photo_at_waypoint.html), [`InputAtWaypoint`](https://docs.nav2.org/configuration/packages/nav2_waypoint_follower-plugins/input_at_waypoint.html) 등이 있습니다.
    - 자세한 파라미터 설명은 [이 링크](https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html)를 참고해주세요.
    
    ```yaml
    waypoint_follower:
      ros__parameters:
        loop_rate: 20
        stop_on_failure: false
        waypoint_task_executor_plugin: "wait_at_waypoint"   
        wait_at_waypoint:
          plugin: "nav2_waypoint_follower::WaitAtWaypoint"
          enabled: True
          waypoint_pause_duration: 200
    ```
    
    - `WaitAtWaypoint` 플러그인을 사용하여, 웨이포인트에 도착할 때마다 200ms 동안 정지했다가 다음 웨이포인트로 이동합니다.

1. 시뮬레이션 환경이 켜져 있지 않다면, 아래 명령어로 켜줍니다.
    
    ```bash
    # terminal 1
    ros2 launch neo_simulation2 simulation.launch.py
    
    # terminal 2
    ros2 launch neo_simulation2 navigation.launch.py use_amcl:=True
    
    # terminal 3
    ros2 launch neo_nav2_bringup rviz_launch.py
    ```
    
    - `nav2_simple_commander` 패키지의 API를 사용하기 위해서는 AMCL을 사용하고 있어야 합니다.
2. **nav2_programming** 패키지의 **nav2_programming** 디렉터리에 `follow_waypoints.py`라는 ****새 파일을 생성합니다.
    
    ```bash
    cd ~/nav2_ws/src/nav2_programming/nav2_programming
    touch follow_waypoints.py
    ```
    
3. 방금 만든 파일에 다음 코드를 복사합니다.
    
    ```python
    #! /usr/bin/env python3
    
    from copy import deepcopy
    
    from geometry_msgs.msg import PoseStamped
    import rclpy
    
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    
    def main():
        rclpy.init()
    
        navigator = BasicNavigator()
    
        # Inspection route, probably read in from a file for a real application
        # from either a map or drive and repeat.
        inspection_route = [
            [11.500, 0.153],
            [7.688, -0.200],
            [5.880, 0.123],
            [2.173, -0.082]]
    
        # Set your demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 3.45
        initial_pose.pose.position.y = -4.0
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        # navigator.setInitialPose(initial_pose)
    
        # Wait for navigation to activate fully
        navigator.waitUntilNav2Active()
    
        # Send your route
        inspection_points = []
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        inspection_pose.pose.orientation.z = 1.0
        inspection_pose.pose.orientation.w = 0.0
        for pt in inspection_route:
            inspection_pose.pose.position.x = pt[0]
            inspection_pose.pose.position.y = pt[1]
            inspection_points.append(deepcopy(inspection_pose))
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(inspection_points)
    
        # Do something during your route (e.x. AI to analyze stock information or upload to the cloud)
        # Print the current waypoint ID for the demonstration
        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                      str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))
    
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Inspection of shelves complete! Returning to start...')
        elif result == TaskResult.CANCELED:
            print('Inspection of shelving was canceled. Returning to start...')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Inspection of shelving failed! Returning to start...')
    
        # go back to start
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)
        while not navigator.isTaskComplete():
            pass
    
        exit(0)
    
    if __name__ == '__main__':
        main()
    ```
    
    - `followWaypoints()` 메서드를 호출하기 위한 입력으로 inspection_points를 사전에 정의했습니다. 이 메서드는 로봇에게 웨이포인트 세트(**PoseStamped** 메시지들)를 따라가도록 요청합니다. 그러면 각 포즈에서 선택한 `TaskExecutor` 플러그인이 실행됩니다.
        
        ```python
        navigator.followWaypoints(inspection_points)
        ```
        
4. `setup.py`를 수정하여 `follow_waypoints.py` 스크립트의 실행 파일을 **entry_points**에 추가합니다.
    
    ```python
    entry_points={
        'console_scripts': [
            'navigate_to_pose = nav2_programming.navigate_to_pose:main',
            'navigate_through_poses = nav2_programming.navigate_through_poses:main',
            **'follow_waypoints = nav2_programming.follow_waypoints:main',**
        ],
    },
    ```
    
5. 패키지를 빌드합니다.
    
    ```bash
    cd ~/nav2_ws
    colcon build --symlink-install --packages-select nav2_programming
    source install/local_setup.bash # 환경에 따라 local_setup.zsh
    ```
    
6. 아래 명령어를 통해 노드를 실행시켜 시나리오 결과를 확인합니다.
    
    ```python
    ros2 run nav2_programming follow_waypoints
    ```

![](/assets/img/Screenshot%20from%202025-12-21%2012-37-31.png)
`waypoint_pause_duration: 5000`로 세팅해서 5초 동안 기다린다.


![](/assets/img/Peek%202025-12-21%2012-36.gif)


https://github.com/ros-navigation/navigation2/blob/main/nav2_waypoint_follower/src/waypoint_follower.cpp
https://github.com/ros-navigation/navigation2/tree/main/nav2_waypoint_follower/plugins

어떻게 구현되어있는지 확인해서, 직접 플러그인 제작해서, follow waitpoint마다 동작 구현할 수 있다.

아니면, behavior tree 이용해도 된다. 특정 목적지에 갈때, 어떤 동작을 수행하겠다.


