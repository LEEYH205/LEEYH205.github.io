---
title: "Costmap Filters - Keepout Filter 활용 (접근 금지 구역)"
date: 2025-12-21 12:55:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Costmap Filters, Keepout Filter]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# Keepout Filter

---

## 지도 수정하기

1. 지도 파일을 수정하기 위해 **GIMP**(**G**NU **I**mage **M**anipulation **P**rogram)를 다운로드 합니다.
    
    ```bash
    sudo apt install gimp
    ```
    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2012-57-06.png)

2. GIMP를 실행하고 수정할 지도 파일을 열어줍니다.
    - [File] - [Open] - [지도 파일 선택]
    - **지도 파일 경로**: `~/nav2_ws/src/neobotix/neo_simulation2/maps/aws.pgm`

    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2012-58-02.png)

3. Rectangle Select Tool을 통해 Keepout Filter를 적용할 영역을 선택합니다.

4. Bucket Fill Tool을 통해 선택한 영영의 색상을 검정색으로 채워줍니다.

    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2012-59-11.png)

5. [File] - [Export As…]를 누른 뒤 파일 이름을 aws_keepout.pgm으로 저장하고 아래와 같은 화면에서 Export 버튼을 누릅니다.
    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-00-11.png)


아래 코드를 통해 기존 aws.yaml을 aws_keepout.yaml으로 복사하고 지도 파일을 aws_keepout.pgm으로 수정합니다.
```bash
cp ~/nav2_ws/src/neobotix/neo_simulation2/maps/aws.yaml ~/nav2_ws/src/neobotix/neo_simulation2/maps/aws_keepout.yaml
```
```yaml
image: aws_keepout.pgm # 수정
resolution: 0.050000
origin: [-7.000, -10.500000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-01-17.png)
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-03-22.png)
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-03-43.png)


![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-04-49.png)
- 마스크의 각 픽셀 음영은 사용할 특정 Costmap 필터에 대한 인코딩된 정보를 의미
    - **0**: free cell
    - **100**: occupied cell (로봇이 통과할 수 없음)


7. 수정한 지도를 워크스페이스의 **install** 디렉토리에 추가하기 위해 패키지를 다시 빌드해줍니다.
    
    ```bash
    cd ~/nav2_ws
    colcon build --symlink-install --packages-select neo_simulation2
    source ~/nav2_ws/install/local_setup.bash
    ```

`/home/lyh/nav2_ws/install/neo_simulation2/share/neo_simulation2/maps`에 제대로 빌드되어있는 것을 확인할 수 있다.

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-06-27.png)

## Keepout Filter 적용 실습

1. 위에서 수정한 지도를 이용하여 **Keepout Filter**를 적용하기 위해 global_costmap과 local_costmap 관련 설정을 수정해줍니다. 먼저 `~/nav2_ws/src/neobotix/neo_simulation2/configs/mpo_700/navigation.yaml` 파일을 아래와 같이 수정해줍니다.
    
    ```yaml
    ...
    
    global_costmap:
      global_costmap:
        ros__parameters:
          ...
          plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
          **filters: ["keepout_filter"]
          keepout_filter:
            plugin: "nav2_costmap_2d::KeepoutFilter"
            enabled: True
            filter_info_topic: "/costmap_filter_info"**
                
    ...
    
    local_costmap:
      local_costmap:
        ros__parameters:
          ...
          plugins: ["voxel_layer", "inflation_layer"]
          **filters: ["keepout_filter"]
          keepout_filter:
            plugin: "nav2_costmap_2d::KeepoutFilter"
            enabled: True
            filter_info_topic: "/costmap_filter_info"
    
    ...**
    ```
    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-13-50.png)
    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-14-31.png)
2. `~/nav2_ws/src/neobotix/neo_simulation2/configs/mpo_700/filters.yaml` 파일을 생성해주고 코드를 작성합니다.
    
    ```bash
    touch ~/nav2_ws/src/neobotix/neo_simulation2/configs/mpo_700/filters.yaml
    ```
    
    ```yaml
    costmap_filter_info_server:
      ros__parameters:
        use_sim_time: true
        filter_info_topic: "/costmap_filter_info"
        type: 0
        mask_topic: "/keepout_filter_mask"
        base: 0.0
        multiplier: 1.0
    
    filter_mask_server:
      ros__parameters:
        use_sim_time: true
        frame_id: "map"
        topic_name: "/keepout_filter_mask"
        yaml_filename: "/home/lyh/nav2_ws/src/neobotix/neo_simulation2/maps/aws_keepout.yaml"
    ```

    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-17-09.png)
    - **costmap_filter_info_server**
        - [`nav2_msgs/CostmapFilterInfo`](https://docs.ros.org/en/humble/p/nav2_msgs/interfaces/msg/CostmapFilterInfo.html) 메시지를 Publish
        - 필터 유형 또는 데이터 변환 계수와 같은 메타데이터가 포함
        - **type**
            - 0: keepout/lanes filter
            - 1: speed limit filter in % of maximum speed
            - 2: speed limit filter in absolute values (m/s)
        - **mask_topic**
            - mask를 publish할 때 토픽 이름
        - **base, multiplier**
            - space = filter_mask_data * **multiplier** + **base**
            - OccupancyGrid 데이터 값을 필터 공간 값으로 변환하는 데 사용
    - **filter_mask_server**
        - [`nav2_msgs/OccupancyGrid`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html) 메시지를 Publish
3. `~/nav2_ws/src/neobotix/neo_nav2_bringup/launch/navigation_neo.launch.py` 파일을 아래와 같이 수정해줍니다.
    <details>
    <summary>`navigation_neo.launch.py`</summary>
    <div markdown="1">
        
        ```python
        # Copyright (c) 2022 Neobotix GmbH
        #
        # Licensed under the Apache License, Version 2.0 (the "License");
        # you may not use this file except in compliance with the License.
        # You may obtain a copy of the License at
        #
        #     http://www.apache.org/licenses/LICENSE-2.0
        #
        # Unless required by applicable law or agreed to in writing, software
        # distributed under the License is distributed on an "AS IS" BASIS,
        # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
        # See the License for the specific language governing permissions and
        # limitations under the License.
        
        import os
        
        from ament_index_python.packages import get_package_share_directory
        
        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, GroupAction
        from launch.conditions import IfCondition
        from launch.substitutions import LaunchConfiguration, PythonExpression
        from launch_ros.actions import Node
        from launch_ros.actions import LoadComposableNodes
        from launch_ros.descriptions import ComposableNode
        from nav2_common.launch import RewrittenYaml
        
        def generate_launch_description():
            # Get the launch directory
            bringup_dir = get_package_share_directory('neo_nav2_bringup')
        
            namespace = LaunchConfiguration('namespace')
            use_sim_time = LaunchConfiguration('use_sim_time')
            autostart = LaunchConfiguration('autostart')
            params_file = LaunchConfiguration('params_file')
            **filter_mask_params_file = LaunchConfiguration('filter_mask_params_file')**
            use_multi_robots = LaunchConfiguration('use_multi_robots')
        
            lifecycle_nodes = ['controller_server',
                               'planner_server',
                               'behavior_server',
                               'bt_navigator',
                               'waypoint_follower',
                               **'filter_mask_server',
                               'costmap_filter_info_server']**
        
            remappings = [('/tf', 'tf'),
                          ('/tf_static', 'tf_static')]
        
            param_substitutions = {
                'use_sim_time': use_sim_time,
                'autostart': autostart}
        
            configured_params = RewrittenYaml(
                    source_file=params_file,
                    root_key=namespace,
                    param_rewrites=param_substitutions,
                    convert_types=True)
        
            **filter_mask_params = RewrittenYaml(
                    source_file=filter_mask_params_file,
                    root_key=namespace,
                    param_rewrites=param_substitutions,
                    convert_types=True)**
        
            declare_namespace_cmd = DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Top-level namespace')
        
            declare_use_sim_time_cmd = DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true')
        
            declare_params_file_cmd = DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(bringup_dir, 'config', 'navigation.yaml'),
                description='Full path to the ROS2 parameters file to use for all launched nodes')
        
            declare_autostart_cmd = DeclareLaunchArgument(
                'autostart', default_value='true',
                description='Automatically startup the nav2 stack')
        
            declare_use_multi_robots_cmd =  DeclareLaunchArgument(
                'use_multi_robots', default_value='False',
                description='A flag to remove the remappings')
        
            **declare_filter_mask_params_file_cmd = DeclareLaunchArgument(
                'filter_mask_params_file',
                default_value=os.path.join(
                    get_package_share_directory('neo_simulation2'),
                    'configs/mpo_700/filters.yaml'))**
        
            load_nodes = GroupAction(
                condition=IfCondition(PythonExpression(['not ', use_multi_robots])),
                actions=[
                    **Node(
                        package='nav2_map_server',
                        executable='map_server',
                        name='filter_mask_server',
                        output='screen',
                        parameters=[filter_mask_params],
                        remappings=remappings),
                    Node(
                        package='nav2_map_server',
                        executable='costmap_filter_info_server',
                        name='costmap_filter_info_server',
                        output='screen',
                        parameters=[filter_mask_params],
                        remappings=remappings),**
                    Node(
                        package='nav2_controller',
                        executable='controller_server',
                        output='screen',
                        parameters=[configured_params],
                        remappings=remappings),
                    Node(
                        package='nav2_planner',
                        executable='planner_server',
                        name='planner_server',
                        output='screen',
                        parameters=[configured_params],
                        remappings=remappings),
                    Node(
                        package='nav2_behaviors',
                        executable='behavior_server',
                        name='behavior_server',
                        output='screen',
                        parameters=[configured_params],
                        remappings=remappings),
                    Node(
                        package='nav2_bt_navigator',
                        executable='bt_navigator',
                        name='bt_navigator',
                        output='screen',
                        parameters=[configured_params],
                        remappings=remappings),
                    Node(
                        package='nav2_waypoint_follower',
                        executable='waypoint_follower',
                        name='waypoint_follower',
                        output='screen',
                        parameters=[configured_params],
                        remappings=remappings),
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_navigation',
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time},
                                    {'autostart': autostart},
                                    {'node_names': lifecycle_nodes}]),
                ]
            )
        
            load_nodes_multi_robot = GroupAction(
                condition=IfCondition(use_multi_robots),
                actions=[
                    Node(
                        package='nav2_controller',
                        executable='controller_server',
                        output='screen',
                        parameters=[configured_params]),
                    Node(
                        package='nav2_planner',
                        executable='planner_server',
                        name='planner_server',
                        output='screen',
                        parameters=[configured_params]),
                    Node(
                        package='nav2_behaviors',
                        executable='behavior_server',
                        name='behavior_server',
                        output='screen',
                        parameters=[configured_params]),
                    Node(
                        package='nav2_bt_navigator',
                        executable='bt_navigator',
                        name='bt_navigator',
                        output='screen',
                        parameters=[configured_params]),
                    Node(
                        package='nav2_waypoint_follower',
                        executable='waypoint_follower',
                        name='waypoint_follower',
                        output='screen',
                        parameters=[configured_params]),
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_navigation',
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time},
                                    {'autostart': autostart},
                                    {'node_names': lifecycle_nodes}]),
                ]
            )
        
            # Create the launch description and populate
            ld = LaunchDescription()
        
            # Declare the launch options
            ld.add_action(declare_namespace_cmd)
            ld.add_action(declare_use_sim_time_cmd)
            ld.add_action(declare_params_file_cmd)
            ld.add_action(declare_autostart_cmd)
            ld.add_action(declare_use_multi_robots_cmd)
            **ld.add_action(declare_filter_mask_params_file_cmd)**
        
            # Add the actions to launch all of the navigation nodes
            ld.add_action(load_nodes)
            ld.add_action(load_nodes_multi_robot)
        
            return ld
        
        ```
    </div>
    </details>

    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-20-38.png)
    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-20-58.png)
    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-21-08.png)
    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-21-19.png)
    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-21-29.png)
    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-21-42.png)
4. yaml 파일이 추가되었으므로 패키지를 다시 빌드해줍니다.
    
    ```bash
    cd ~/nav2_ws
    colcon build --symlink-install --packages-select neo_simulation2
    source install/local_setup.bash # 환경에 따라 local_setup.zsh
    ```
    
5. 결과를 확인하기 위해 시뮬레이션을 재실행 해줍니다.
    
    ```bash
    # terminal 1 (이미 켜져있을 경우 생략)
    ros2 launch neo_simulation2 simulation.launch.py
    
    # terminal 2
    ros2 launch neo_simulation2 navigation.launch.py use_amcl:=True
    
    # terminal 3 (이미 켜져있을 경우 생략)
    ros2 launch neo_nav2_bringup rviz_launch.py
    ```
    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2013-25-38.png)

    ![](/assets/img/ros2nav2/gif/Peek%202025-12-21%2013-31.gif)

6. 아래와 같이 Mask를 생성하여 특정 경로로만 로봇이 주행하도록 할 수도 있습니다.
    ![](/assets/img/ros2nav2/img/keepoutfilter_1.png)