---
title: "Simple Commander API - Navigate Through Poses"
date: 2025-12-21 11:10:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Simple Commander API, Navigate Through Poses]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# **[Nav2 Action 2] Navigate Through Poses**

---

- `NavigateThroughPoses.action`의 기본 구조는 아래와 같습니다.
    
    ```yaml
    # goal definition
    geometry_msgs/PoseStamped[] poses
    string behavior_tree
    ---
    # result definition
    std_msgs/Empty result
    ---
    # feedback definition
    geometry_msgs/PoseStamped current_pose
    builtin_interfaces/Duration navigation_time
    builtin_interfaces/Duration estimated_time_remaining
    int16 number_of_recoveries
    float32 distance_remaining
    int16 number_of_poses_remaining
    ```
    
    - 이 동작은 각 waypoint에서 멈추지 않고 waypoint(경유지)를 통과함
    - **NavigateToPose**와 거의 구조가 똑같으며, 경유 지점을 통한 진행 상황을 추적하기 위해 피드백에 `number_of_poses_remaining` 필드가 추가되었습니다.

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
2. **nav2_programming** 패키지의 **nav2_programming** 디렉터리에 `navigate_through_poses.py`라는 ****새 파일을 생성합니다.
    
    ```bash
    cd ~/nav2_ws/src/nav2_programming/nav2_programming
    touch navigate_through_poses.py
    ```
    
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2012-10-53.png)
3. 방금 만든 파일에 다음 코드를 복사합니다.
    
    ```python
    #! /usr/bin/env python3
    
    from copy import deepcopy
    
    from geometry_msgs.msg import PoseStamped
    from rclpy.duration import Duration
    import rclpy
    
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    
    '''
    Basic security route patrol demo. In this demonstration, the expectation
    is that there are security cameras mounted on the robots recording or being
    watched live by security staff.
    '''
    
    def main():
        rclpy.init()
    
        navigator = BasicNavigator()
    
        # Security route, probably read in from a file for a real application
        # from either a map or drive and repeat.
        security_route = [
            [1.560, 2.648],
            [2.162, -1.875],
            [13.041, -1.888],
            [13.109, -7.273],
            [3.082, -7.385],
            [-5.263, -7.288],
            [-5.374, -9.868],
            ]
    
        # Set your demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 3.45
        initial_pose.pose.position.y = 2.15
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        # navigator.setInitialPose(initial_pose)
    
        # Wait for navigation to activate fully
        navigator.waitUntilNav2Active()
    
        # Do security route until dead
        while rclpy.ok():
            # Send your route
            route_poses = []
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = navigator.get_clock().now().to_msg()
            pose.pose.orientation.w = 1.0
            for pt in security_route:
                pose.pose.position.x = pt[0]
                pose.pose.position.y = pt[1]
                route_poses.append(deepcopy(pose))
            navigator.goThroughPoses(route_poses)
    
            # Do something during your route (e.x. AI detection on camera images for anomalies)
            # Print ETA for the demonstration
            i = 0
            while not navigator.isTaskComplete():
                i = i + 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time to complete current route: ' + '{0:.0f}'.format(
                          Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                          + ' seconds.')
    
                    # Some failure mode, must stop since the robot is clearly stuck
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                        print('Navigation has exceeded timeout of 180s, canceling the request.')
                        navigator.cancelTask()
    
            # If at the end of the route, reverse the route to restart
            security_route.reverse()
    
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Route complete! Restarting...')
            elif result == TaskResult.CANCELED:
                print('Security route was canceled, exiting.')
                exit(1)
            elif result == TaskResult.FAILED:
                print('Security route failed! Restarting from the other side...')
    
        exit(0)
    
    if __name__ == '__main__':
        main()
    ```
    
    - 사전 등록해 놓은 포즈들을 `route_poses`에 추가합니다. 각 포즈는 **PoseStamped** 메시지로 정의되어야 한다는 점에 유의하세요.
        
        ```python
        
            **security_route** = [
                [1.560, 2.648],
                [2.162, -1.875],
        				...
        				]
        				
        		...
        				
        		for pt in **security_route**:
                pose.pose.position.x = pt[0]
                pose.pose.position.y = pt[1]
                **route_poses**.append(deepcopy(pose))
        ```
        
    - `goThroughPoses()` 메서드를 호출해 로봇에게 일련의 포즈를 방문하도록 요청합니다.
        
        ```python
        navigator.goThroughPoses(route_poses)
        ```
        
    - 탐색 작업이 180초 이상 걸리는 경우 `cancelTask()` 메서드를 사용하여 현재 작업을 취소하였습니다.
        
        ```python
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                        print('Navigation has exceeded timeout of 180s, canceling the request.')
                        navigator.cancelTask()
        ```
        
4. `setup.py`를 수정하여 `navigate_through_poses.py` 스크립트의 실행 파일을 **entry_points**에 추가합니다.
    
    ```python
    entry_points={
        'console_scripts': [
            'navigate_to_pose = nav2_programming.navigate_to_pose:main',
            **'navigate_through_poses = nav2_programming.navigate_through_poses:main',**
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
    
    ```bash
    ros2 run nav2_programming navigate_through_poses
    ```

![](/assets/img/ros2nav2/gif/Peek%202025-12-21%2012-16.gif)
설정해 놓은 경로를 찍고 다음 경로로 이동하는 것을 볼 수 있다.