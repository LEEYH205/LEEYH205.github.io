---
title: "Costmap Filters - Speed Filter 활용 (속도 제한 구역)"
date: 2025-12-21 13:35:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Costmap Filters, Speed Filter]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# Speed Filter

---

## 지도 수정하기

1. GIMP에서 `~/nav2_ws/src/neobotix/neo_simulation2/maps/aws.pgm` 파일을 열어준 뒤, 지도를 수정해줍니다. 색상 변경 시 아래 도구를 통해 `L 값`을 변경하여 `검정(0)~흰색(100)` 비율을 선택해줍니다.
![](/assets/img/Screenshot%20from%202025-12-21%2013-36-38.png)


2. 수정 완료 후, [File] - [Export As…]를 누른 뒤 파일 이름을 aws_speed.pgm으로 저장합니다.
![](/assets/img/Screenshot%20from%202025-12-21%2013-41-13.png)

3. 아래 코드를 통해 기존 aws.yaml을 aws_speed.yaml으로 복사하고 지도 파일을 aws_speed.pgm으로 수정합니다.
```bash
cp ~/nav2_ws/src/neobotix/neo_simulation2/maps/aws.yaml ~/nav2_ws/src/neobotix/neo_simulation2/maps/aws_speed.yaml
```
```yaml
image: aws_speed.pgm # 수정
mode: scale # 추가
resolution: 0.050000
origin: [-7.000, -10.500000, 0.000000]
negate: 0
occupied_thresh: 1.0 # 수정 (기존: 0.65)
free_thresh: 0.0 # 수정 (기존: 0.196)
```

![](/assets/img/Screenshot%20from%202025-12-21%2013-39-59.png)
![](/assets/img/Screenshot%20from%202025-12-21%2013-40-25.png)

- **mode: trinary (기본)**
    - 맵의 픽셀 값을 세 가지 상태로 변환 (자유 영역, 장애물, 알려지지 않은 영역)
    - 로봇이 주로 자유롭게 이동할 수 있는 영역과 장애물을 명확하게 구분해야 하는 경우 사용
- **mode: scale**
    - 맵의 픽셀 값을 연속적인 확률 값으로 변환 (0: 자유영역 ~ 100: 장애물)
    - 장애물과 자유 영역 사이의 확률적 경계를 고려해야 하거나, 더 정교한 경로 계획이 필요한 경우 사용


4. 수정한 지도를 워크스페이스의 **install** 디렉토리에 추가하기 위해 패키지를 다시 빌드해줍니다.
    
    ```bash
    cd ~/nav2_ws
    colcon build --symlink-install --packages-select neo_simulation2
    source ~/nav2_ws/install/local_setup.bash
    ```


## Speed Filter 적용 실습

1. 위에서 수정한 지도를 이용하여 **Speed Filter**를 적용하기 위해 global_costmap, local_costmap, controller_server 관련 설정을 수정해줍니다. 먼저 `~/nav2_ws/src/neobotix/neo_simulation2/configs/mpo_700/navigation.yaml` 파일을 아래와 같이 수정해줍니다.
    - 이전에 적용했던 **Keepout Filter**는 주석처리해줍니다.
    - NeoLocalPlanner 같은 경우 speed_limit이 적용되지 않으므로 **DWBLocalPlanner**로 교체합니다.
    <details>
    <summary>navigation.yaml</summary>
    <div markdown="1">

    ...

    controller_server:
    ros__parameters:
        use_sim_time: True
        controller_frequency: 20.0
        min_x_velocity_threshold: 0.001
        min_y_velocity_threshold: 0.5
        min_theta_velocity_threshold: 0.001
        failure_tolerance: 0.3
        progress_checker_plugin: "progress_checker"
        goal_checker_plugins: ["general_goal_checker"] # "precise_goal_checker"
        controller_plugins: ["FollowPath"]
        # Progress checker parameters
        progress_checker:
        plugin: "nav2_controller::SimpleProgressChecker"
        required_movement_radius: 0.5
        movement_time_allowance: 10.0
        # Goal checker parameters
        general_goal_checker:
        stateful: True
        plugin: "nav2_controller::SimpleGoalChecker"
        xy_goal_tolerance: 0.25
        yaw_goal_tolerance: 1.309
        # DWB parameters
        FollowPath:
        plugin: "dwb_core::DWBLocalPlanner"
        debug_trajectory_details: True
        min_vel_x: 0.0
        min_vel_y: 0.0
        max_vel_x: 0.8
        max_vel_y: 0.0
        max_vel_theta: 1.0
        min_speed_xy: 0.0
        max_speed_xy: 0.8
        min_speed_theta: 0.0
        acc_lim_x: 0.25
        acc_lim_y: 0.25
        acc_lim_theta: 3.2
        decel_lim_x: -2.5
        decel_lim_y: 0.0
        decel_lim_theta: -3.2
        vx_samples: 20
        vy_samples: 5
        vtheta_samples: 20
        sim_time: 1.7
        linear_granularity: 0.05
        angular_granularity: 0.025
        transform_tolerance: 0.2
        xy_goal_tolerance: 0.25
        trans_stopped_velocity: 0.25
        short_circuit_trajectory_evaluation: True
        stateful: True
        critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
        BaseObstacle.scale: 0.02
        PathAlign.scale: 32.0
        PathAlign.forward_point_distance: 0.1
        GoalAlign.scale: 24.0
        GoalAlign.forward_point_distance: 0.1
        PathDist.scale: 32.0
        GoalDist.scale: 24.0
        RotateToGoal.scale: 32.0
        RotateToGoal.slowing_factor: 5.0
        RotateToGoal.lookahead_time: -1.0

        # UNCOMMENT LINE BELOW FOR SPEED LIMIT ZONES DEMO
        speed_limit_topic: "/speed_limit"
        
    ...

    global_costmap:
    global_costmap:
        ros__parameters:
        ...
        plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
        # filters: ["keepout_filter"]
        # keepout_filter:
        #   plugin: "nav2_costmap_2d::KeepoutFilter"
        #   enabled: True
        #   filter_info_topic: "/costmap_filter_info"
        filters: ["speed_filter"]
        speed_filter:
            plugin: "nav2_costmap_2d::SpeedFilter"
            enabled: True
            filter_info_topic: "/costmap_filter_info"
            speed_limit_topic: "/speed_limit"
                
    ...

    local_costmap:
    local_costmap:
        ros__parameters:
        ...
        plugins: ["voxel_layer", "inflation_layer"]
        # filters: ["keepout_filter"]
        # keepout_filter:
        #   plugin: "nav2_costmap_2d::KeepoutFilter"
        #   enabled: True
        #   filter_info_topic: "/costmap_filter_info"
        filters: ["speed_filter"]
        speed_filter:
            plugin: "nav2_costmap_2d::SpeedFilter"
            enabled: True
            filter_info_topic: "/costmap_filter_info"
            speed_limit_topic: "/speed_limit"

    ...

    </div>
    </details>

2. `~/nav2_ws/src/neobotix/neo_simulation2/configs/mpo_700/filters.yaml` 파일을 아래와 같이 수정해줍니다.
```yaml
costmap_filter_info_server:
  ros__parameters:
    use_sim_time: true
    filter_info_topic: "/costmap_filter_info"
    # For Keepout Zones
    #type: 0
    #mask_topic: "/keepout_filter_mask"
    #base: 0.0
    #multiplier: 1.0
    # For Speed Limits
    type: 1
    mask_topic: "/speed_filter_mask"
    base: 100.0
    multiplier: -1.0

filter_mask_server:
  ros__parameters:
    use_sim_time: true
    frame_id: "map"
    # For Keepout Zones
    #topic_name: "/keepout_filter_mask"
    #yaml_filename: "/home/lyh/nav2_ws/src/neobotix/neo_simulation2/maps/aws_keepout.yaml"
    # For Speed Limits
    topic_name: "/speed_filter_mask"
    yaml_filename: "/home/lyh/nav2_ws/src/neobotix/neo_simulation2/maps/aws_speed.yaml"
```

3. 결과를 확인하기 위해 시뮬레이션을 재실행 해줍니다.
    
    ```bash
    # terminal 1
    ros2 launch neo_simulation2 simulation.launch.py
    
    # terminal 2
    ros2 launch neo_simulation2 navigation.launch.py use_amcl:=True
    
    # terminal 3
    ros2 launch neo_nav2_bringup rviz_launch.py
    ```
![](/assets/img/Screenshot%20from%202025-12-21%2013-51-15.png)
4. Rviz2에서 /speed_filter_mask 토픽을 시각화하고 지도를 확인합니다.
![](/assets/img/Screenshot%20from%202025-12-21%2013-52-22.png)
```bash
ros2 launch neo_simulation2 navigation.launch.py use_amcl:=True
```
를 다시 실행
![](/assets/img/Screenshot%20from%202025-12-21%2013-54-01.png)

![](/assets/img/Peek%202025-12-21%2014-00.gif)
![](/assets/img/Peek%202025-12-21%2014-02.gif)