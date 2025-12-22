---
title: "Behavior Tree - Nav2 연동 테스트"
date: 2025-12-20 15:50:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Behavior Tree]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---
# 실습 환경 구축 및 시뮬레이션 환경 열기

---

1. 워크스페이스 내에 필요한 리소스들을 가져옵니다.
    
    ```bash
    cd ~/nav2_ws/src
    git clone https://github.com/Adlink-ROS/ros2_object_msgs.git
    git clone https://github.com/Adlink-ROS/BT_ros2.git
    ```
    
2. 워크스페이스 내 패키지들을 빌드합니다.
    
    ```bash
    cd ~/nav2_ws
    colcon build --symlink-install --packages-select object_msgs
    source install/local_setup.bash # 환경에 따라 local_setup.zsh
    
    rosdep install --from-paths src/BT_ros2 --ignore-src -r -y
    colcon build --symlink-install --packages-select bt_ros2
    source install/local_setup.bash # 환경에 따라 local_setup.zsh
    ```

3. `mememan_world.model` 환경을 사용하기 위해 파일들을 수정해줍니다.
    - `~/nav2_ws/src/neuronbot2/neuronbot2_gazebo/launch/**neuronbot2_world.launch.py**`
    ```PYTHON
    ...
        world = os.path.join(
        get_package_share_directory('neuronbot2_gazebo'),
        'worlds',
        'mememan_world.model'
    )
    ...
    ```
    - `~/nav2_ws/src/neuronbot2/neuronbot2_nav/launch/bringup_launch.py`
    ```PYTHON
    ...

    my_map_file = 'mememan.yaml'

    ...
    ```

4. 시뮬레이션 환경을 열어줍니다.
```DASH
# terminal 1
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py

# terminal 2
ros2 launch neuronbot2_nav bringup_launch.py use_sim_time:=true
```

![](/assets/img/Screenshot%20from%202025-12-20%2015-57-14.png)


# [실습 1] 정해진 경유지 반복하여 방문하기
```dash
ros2 launch bt_ros2 bt_ros2.launch.py
```
![](/assets/img/Screenshot%20from%202025-12-20%2015-57-41.png)
![](/assets/img/bt_nav2_1.png)
- **메인 트리 구조 (MainTree)**
    - "MainTree"라는 ID를 가진 주 행동 트리가 정의되어 있음
    - 이 트리는 전체 시퀀스를 3번 반복 (Repeat 노드)
- **목표 설정**
    - 세 개의 목표 위치(Goal_a, Goal_b, Goal_c)가 SetBlackboard 노드를 통해 정의됨
    - 각 목표는 x, y 위치와 z, w 방향을 포함
- **이동 시퀀스**
    - 정의된 세 목표 지점으로 순차적으로 이동
    - 각 이동은 "MoveRobot" 서브트리를 호출하여 실행
![](/assets/img/Peek%202025-12-20%2016-05.gif)



# [실습 2] InterruptEvent 활용
```DASH
# terminal 1
ros2 run bt_ros2 bt_ros2 --ros-args -p bt_xml:=$HOME/nav2_ws/src/BT_ros2/bt_xml/bt_nav_mememan_interrupt.xml

# terminal 2
ros2 topic pub -1 /interrupt_event std_msgs/msg/String data:\ \'gohome\'
```

```xml
<root main_tree_to_execute = "MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <SetBlackboard output_key="Goal_a" value="-0.579;-1.341;0.0;1.0" />
            <SetBlackboard output_key="Goal_b" value="5.214;-1.533;0.0;1.0" />
            <SetBlackboard output_key="Goal_c" value="-1.588;1.253;0.0;1.0" />
            <Fallback>
                <ReactiveSequence>
                    <InterruptEvent event="gohome"/>
                    <Repeat num_cycles="10">
                        <Sequence>
                            <SubTree ID="MoveRobot" target="Goal_a" />
                            <SubTree ID="MoveRobot" target="Goal_b" />
                        </Sequence>
                    </Repeat>
                </ReactiveSequence>
                <Sequence>
                    <SubTree ID="MoveRobot" target="Goal_c" />
                </Sequence>
            </Fallback>
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="MoveRobot">
        <Sequence name="SetGoal">
            <Nav2Client goal="{target}" />
        </Sequence>
    </BehaviorTree>
</root>

```
![](/assets/img/bt_nav2_2.png)
- 목표 설정
    - 세 개의 목표 위치(Goal_a, Goal_b, Goal_c)가 SetBlackboard 노드를 통해 정의됨
- 주요 행동 구조 (Fallback 노드 내)
    - 두 가지 주요 행동 중 하나가 실행
    - 첫 번째 행동 (**ReactiveSequence**)
        - "gohome" 이벤트를 감지하는 `InterruptEvent` 노드가 있음
        - Goal_a와 Goal_b 사이를 10회 왕복
            
            → MoveRobot 서브트리를 사용하여 Goal_a로 이동
            
            → 그 다음 Goal_b로 이동
            
        - 이 과정 중 언제든 "gohome" 이벤트가 발생하면 중단
    - 두 번째 행동 (**Sequence**)
        - 첫 번째 행동이 중단되거나 완료되면 실행
        - MoveRobot 서브트리를 사용하여 Goal_c로 이동

![](/assets/img/Screenshot%20from%202025-12-20%2016-13-27.png)
![](/assets/img/Peek%202025-12-20%2016-12.gif)