---
title: "IMU 및 Odometry 데이터 분석"
date: 2025-12-23 16:50:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, IMU, Odometry]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

# IMU 및 Odometry 데이터 시각화해보기

---

## plotjuggler
![](/assets/img/HiWonderPi/img/plotjuggler.png)
![](/assets/img/HiWonderPi/gif/plotjuggler.gif)

```bash
# 설치
sudo apt install ros-humble-plotjuggler ros-humble-plotjuggler-ros ros-humble-plotjuggler-msgs

# 실행
ros2 run plotjuggler plotjuggler
```

![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-23%2016-53-32.png)
![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-23%2016-54-18.png)

![](/assets/img/HiWonderPi/gif/IMG_4425.gif)
## Rviz2 활용

- Odometry 시각화
- IMU 시각화


![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-23%2017-02-30.png)
IMU 세팅을 제대로 안하면 이렇게 이상한 값이 나온다.

![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-23%2017-03-13.png)
IMU 세팅을 다시해서 제대로 나온다.

![](/assets/img/HiWonderPi/gif/IMG_4426.gif)
![](/assets/img/HiWonderPi/gif/IMG_4428.gif)


# 핵심 코드 분석하기

---

## Odometry 정보 Publish를 위한 절차

- **`driver/controller/launch/controller.launch.py`** 시작
    1. `odom_publisher_launch` (IncludeLaunchDescription)
        
        ```python
        odom_publisher_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(controller_package_path, 'launch/odom_publisher.launch.py')
            ]),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': use_namespace,
                'imu_frame': imu_frame,
                'frame_prefix': frame_prefix,
                'base_frame': base_frame,
                'odom_frame': odom_frame
            }.items()
        )
        ```
        
    2. `imu_filter_launch` (IncludeLaunchDescription)
        
        ```python
        imu_filter_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(peripherals_package_path, 'launch/imu_filter.launch.py')
            ])
        )
        ```
        
    3. `ekf_filter_node` (Node)
        
        ```python
        ekf_filter_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_param, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('odometry/filtered', 'odom'),
                ('cmd_vel', 'controller/cmd_vel')
            ],
            condition=IfCondition(enable_odom),
        )
        ```

## **1단계: odom_publisher_launch**

### **1-1. robot_description_launch**

```python
robot_description_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(rosmentor_description_package_path, 'launch/robot_description.launch.py')
    ]),
    launch_arguments={
        'frame_prefix': frame_prefix,
        'use_gui': 'false',
        'use_rviz': 'false',
        'use_sim_time': 'false',
        'use_namespace': use_namespace,
        'namespace': namespace,
    }.items()
)
```

- 역할: URDF 로딩, robot_state_publisher 실행
- 출력: TF 트리 (base_footprint, imu_link 등)

### **1-2. robot_controller_launch**

```python
robot_controller_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(robot_controller_package_path, 'launch/ros_robot_controller.launch.py')
    ]),
    launch_arguments={
        'imu_frame': imu_frame,
    }.items()
)
```

```python
ros_robot_controller_node = Node(
    package='ros_robot_controller',
    executable='ros_robot_controller',
    output='screen',
    parameters=[{'imu_frame': imu_frame}]
)
```

- 역할: 하드웨어 제어 노드 실행
- 주요 기능:
    - IMU Raw 데이터 발행: `/ros_robot_controller/imu_raw`
    - 모터 제어 구독: `/ros_robot_controller/set_motor`
    - 배터리, 버튼, 조이스틱 등 센서 토픽 발행
- 초기화:
    - 하드웨어 보드 연결 (`/dev/rrc`)
    - 모터 속도 초기화 (0으로 설정)
    - 50Hz로 센서 데이터 발행 시작

### **1-3. odom_publisher_node**

```python
odom_publisher_node = Node(
    package='controller',
    executable='odom_publisher',
    name='odom_publisher',
    output='screen',
    parameters=[os.path.join(controller_package_path, 'config/calibrate_params.yaml'), {
        'base_frame_id': base_frame, 
        'odom_frame_id': odom_frame,
        'pub_odom_topic': True,
        }],  
)
```

- 역할: Wheel encoder 기반 odometry 계산 및 발행
    
    ![](https://docs.hiwonder.com/projects/MentorPi/en/latest/_static/media/4.MotionControlLesson/4.6/media/image16.png)
    
- 주요 기능:
    - cmd_vel 구독: `/controller/cmd_vel`
    - 속도 적분으로 위치 계산
    - `odom_raw` 토픽 발행
- 파라미터:
    - calibrate_params.yaml: 보정 계수 (linear_correction_factor, angular_correction_factor)
    - base_frame_id: base_footprint
    - odom_frame_id: odom
- 연산 흐름
    1. `cmd_vel` (Twist) 입력
    2. 속도 분해 및 변환
        - `set_velocity()`에서 필요한 각각의 모터회전 속도 계산
    3. 실제 모터 제어 → MotorsState 발행
    4.  `cal_odom_fun()`에서 오도메트리 연산 (Scale factor 적용 포함)
        
        ```python
        def cal_odom_fun(self):
            while True:
                self.current_time = time.time()
                if self.last_time is None:
                    self.dt = 0.0
                else:
                    self.dt = self.current_time - self.last_time
        
                self.odom.header.stamp = self.clock.now().to_msg()
                
                delta_x = self.linear_x * self.dt * math.cos(self.pose_yaw)
                delta_y = self.linear_x * self.dt * math.sin(self.pose_yaw)
                delta_yaw = self.angular_z * self.dt
        
                self.x += delta_x
                self.y += delta_y
                self.pose_yaw += delta_yaw
        
                self.odom.pose.pose.position.x = self.linear_factor * self.x
                self.odom.pose.pose.position.y = self.linear_factor * self.y
                self.odom.pose.pose.orientation = rpy2qua(0.0, 0.0, self.pose_yaw)
                self.odom.twist.twist.linear.x = self.linear_x
                self.odom.twist.twist.linear.y = self.linear_y
                self.odom.twist.twist.angular.z = self.angular_z
        
                if self.linear_x == 0 and self.linear_y == 0 and self.angular_z == 0:
                    self.odom.pose.covariance = ODOM_POSE_COVARIANCE_STOP
                    self.odom.twist.covariance = ODOM_TWIST_COVARIANCE_STOP
                else:
                    self.odom.pose.covariance = ODOM_POSE_COVARIANCE
                    self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
        
                self.odom_pub.publish(self.odom)
                self.last_time = self.current_time
                time.sleep(0.02)
        ```
        
        - 고정 공분산(covariance) 적용
            
            ```python
            ODOM_POSE_COVARIANCE = list(map(float,
                                    [1e-3, 0, 0, 0, 0, 0,
                                    0, 1e-3, 0, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e3]))
            ODOM_POSE_COVARIANCE_STOP = list(map(float,
                                        [1e-9, 0, 0, 0, 0, 0,
                                         0, 1e-3, 1e-9, 0, 0, 0,
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0,
                                         0, 0, 0, 0, 1e6, 0,
                                         0, 0, 0, 0, 0, 1e-9]))
            ODOM_TWIST_COVARIANCE = list(map(float,
                                    [1e-3, 0, 0, 0, 0, 0,
                                     0, 1e-3, 0, 0, 0, 0,
                                     0, 0, 1e6, 0, 0, 0,
                                     0, 0, 0, 1e6, 0, 0,
                                     0, 0, 0, 0, 1e6, 0,
                                     0, 0, 0, 0, 0, 1e3]))
            ODOM_TWIST_COVARIANCE_STOP = list(map(float,
                                        [1e-9, 0, 0, 0, 0, 0,
                                          0, 1e-3, 1e-9, 0, 0, 0,
                                          0, 0, 1e6, 0, 0, 0,
                                          0, 0, 0, 1e6, 0, 0,
                                          0, 0, 0, 0, 1e6, 0,
                                          0, 0, 0, 0, 0, 1e-9]))
            ```
            
            | 매개변수 이름 | 설명 |
            | --- | --- |
            | ODOM_POSE_COVARIANCE | Odometry Pose covariance |
            | ODOM_POSE_COVARIANCE_STOP | 속도가 0일 때, Odometry Pose covariance |
            | ODOM_TWIST_COVARIANCE | Odometry Twist covariance |
            | ODOM_TWIST_COVARIANCE_STOP | 속도가 0일 때, Odometry Twist covariance |

## **2단계: imu_filter_launch**

### **2-1. imu_calib_node**

```python
imu_calib_node = Node(
    package='imu_calib',
    executable='apply_calib',
    name='imu_calib',
    output='screen',
    parameters=[{"calib_file": calib_file_path}],
    remappings=[
        ('raw', '/ros_robot_controller/imu_raw'),
        ('corrected', 'imu_corrected')
    ]
)
```

- 역할: IMU Raw 데이터 보정
- 입력: `/ros_robot_controller/imu_raw`
- 출력: `imu_corrected`
- 보정 관련 파일: `calibration/config/imu_calib.yaml`

### **2-2. imu_filter_node**

```python
imu_filter_node = Node(
    package='imu_complementary_filter',
    executable='complementary_filter_node',
    name='imu_filter',
    output='screen',
    parameters=[
        {
            'use_mag': False,
            'do_bias_estimation': True,
            'do_adaptive_gain': True,
            'publish_debug_topics': True
        }
    ],
    remappings=[
        ('/tf', 'tf'),
        ('/imu/data_raw', 'imu_corrected'),
        ('imu/data', 'imu')
    ]
)
```

- 역할: Complementary filter로 IMU orientation 계산
- 입력: `imu_corrected`
- 출력: `imu` (필터링된 IMU 데이터)
- 기능:
    - Bias 추정
    - Adaptive gain
    - Orientation 계산 (quaternion)

## **3단계: ekf_filter_node**

- 역할: Extended Kalman Filter로 센서 퓨전
- 입력:
    - `odom0`: **odom_raw** (wheel encoder odometry)
    - `odom1`: **odom_rf2o** (LiDAR odometry, 사용은 안됨)
    - `imu0`: **imu** (필터링된 IMU)
- 출력:
    - `odom` (퓨전된 odometry)
    - TF: `odom → base_footprint`
- 설정 관련 파일: `driver/controller/config/ekf.yaml`