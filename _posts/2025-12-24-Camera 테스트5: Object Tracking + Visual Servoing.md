---
title: "Camera 테스트5: Object Tracking + Visual Servoing"
date: 2025-12-24 13:30:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, Camera, Object Tracking, Visual Servoing]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

# 개요

---

> ROS2를 활용하여 카메라로 객체를 실시간으로 추적하고, PID 제어를 통해 로봇을 움직여 객체를 화면의 목표 위치에 유지하는 프로젝트 입니다. `object_tracking` 노드가 카메라 이미지를 직접 처리하여 색상 기반 객체를 검출하고, Visual Servoing 기법을 사용하여 로봇의 선속도 및 각속도를 제어합니다.
> 

**주요 기능**

- **직접 이미지 처리**: 카메라 이미지를 직접 수신하여 LAB 색상 공간에서 객체 검출
- **Visual Servoing**: 이미지 좌표를 이용한 로봇 이동 제어
- **이중 축 PID 제어**: 수평(yaw) 및 수직(거리) 방향으로 독립적인 PID 제어
- **로봇 이동 제어**: 선속도 및 각속도를 통한 로봇 제어
- **색상 선택**: ColorPicker를 통한 마우스 클릭 색상 선택 또는 사전 정의된 색상 사용
- **객체 추적**: 이전 위치 기반 객체 추적 알고리즘


### (Project 4) Color Tracking과의 차이점

| 항목 | Color Tracking | Object Tracking |
| --- | --- | --- |
| **입력** | color_detect 노드로부터 색상 정보 수신 | 카메라 이미지를 직접 처리 |
| **제어 대상** | 카메라 서보 (팬/틸트) | 로봇 이동 (선속도/각속도) |
| **색상 공간** | RGB (color_detect 노드에서 처리) | LAB (직접 처리) |
| **PID 제어** | 서보 각도 제어 | 로봇 속도 제어 |

동작 확인 (디버그 모드)
1. MentorPi를 시작하고 VNC Viewer에 연결합니다.
2. **Terminator**에서 아래 명령어를 입력해 ROS 2관련 자동 실행 서비스를 중지합니다.
    
    ```bash
    ~/.stop_ros.sh
    ```
    
3. Object Tracking을 활성화하기 위해 명령어를 입력합니다.
    
    ```bash
    ros2 launch app object_tracking_node.launch.py debug:=true
    ```
    
4. 새 터미널에서 아래 서비스를 호출하고, 추적할 Object를 클릭하고 영역을 확인합니다.
    
    ```bash
    ros2 service call /object_tracking/enter std_srvs/srv/Trigger {}
    ```
    
5. 아래 명령으로 결과를 확인합니다.
    
    ```bash
    ros2 service call /object_tracking/set_running std_srvs/srv/SetBool "{data: true}"
    ```
    
6. 실습을 종료하려면 터미널에서 'Ctrl+C'를 누르거나 아래 명령어를 입력합니다.
    
    ```bash
    ros2 service call /object_tracking/exit std_srvs/srv/Trigger {}
    ```
    
7. **LXTerminal**에서 아래 명령어로 로봇 기능들을 다시 활성화하거나, 로봇을 재시작할 수 있습니다.
    
    ```bash
    sudo systemctl restart start_node.service
    ```

![](/assets/img/HiWonderPi/gif/IMG_4442_1.gif)

## Launch 파일 구조

```
**object_tracking_node.launch.py**
    ├─ **depth_camera.launch.py** 포함 (debug=true일 때만)
    │   └─ 카메라 드라이버 노드 시작
    │   └─ 이미지 발행 노드 시작
    │
    ├─ **controller.launch.py** 포함 (debug=true일 때만)
    │   └─ 로봇 제어기 노드 시작
    │
    └─ **object_tracking** 노드 시작
        └─ debug 파라미터 전달
```

1. **Controller 노드**: 로봇 제어기 (debug=true일 때만)
2. **Depth Camera 노드**: 카메라 드라이버 및 이미지 발행 (debug=true일 때만)
3. **Object Tracking 노드**: 객체 추적 및 Visual Servoing 제어

**Launch 파일 간의 관계:**
- `object_tracking_node.launch.py`는 최상위 launch 파일로, 다른 launch 파일들을 조건부로 포함합니다.
- `depth_camera.launch.py`는 카메라 하드웨어 드라이버를 시작합니다.
- `controller.launch.py`는 로봇 제어기를 담당합니다.

## 동작 원리

1. **이미지 수신**: 카메라로부터 RGB 이미지를 수신
2. **색상 공간 변환**: RGB 이미지를 LAB 색상 공간으로 변환
3. **객체 검출**: LAB 색상 범위를 이용하여 객체를 검출
    - 마스크 생성 및 형태학적 연산 (침식/팽창)
    - 컨투어 검출 및 필터링
    - 최소 외접원 계산
4. **객체 추적**: 이전 프레임의 객체 위치를 기반으로 가장 가까운 객체를 선택
5. **오차 계산**: 객체의 현재 위치와 목표 위치의 차이를 계산
    - 수평 방향 오차: `x - x_stop` (기본값: 320)
    - 수직 방향 오차: `y - y_stop` (기본값: 300 또는 400)
6. **PID 제어**: 각 방향의 오차를 PID 제어기로 처리하여 로봇 속도 조정량을 계산
    - 수평 방향: 각속도 제어 (yaw)
    - 수직 방향: 선속도 제어 (거리)
7. **로봇 제어**: 계산된 속도를 Twist 메시지로 발행하여 로봇을 제어
8. **피드백 루프**: 새로운 이미지에서 객체 위치를 다시 확인하고 반복

## 프로그램 구조

### Launch 파일 구조

```
[Launch 파일 실행]
    ↓
[환경 변수 확인] (need_compile)
    ↓
[Debug 모드 확인]
    ↓
[Depth Camera Launch 포함] (debug=true일 때만)
    ㄴ [카메라 드라이버 노드]
    ㄴ [이미지 발행 노드]
    ↓
[Controller Launch 포함] (debug=true일 때만)
    ㄴ [로봇 제어기 노드]
    ↓
[Object Tracking 노드]
    ├─ [이미지 구독]
    ├─ [객체 검출 및 추적]
    ├─ [PID 제어]
    └─ [로봇 제어]
```
### Object Tracking 노드 구조

```
[프로그램 시작]
    ↓
[ROS2 노드 초기화]
    ├─ 서비스 생성
    ├─ 토픽 구독/발행 설정
    ├─ PID 제어기 초기화
    └─ 상태 변수 초기화
    ↓
[이미지 수신] (image_callback)
    ↓
[색상 선택 모드 확인]
    ├─ ColorPicker가 있으면 색상 선택
    └─ Tracker가 있으면 객체 추적
    ↓
[객체 추적] (ObjectTracker.__call__)
    ├─ [LAB 색상 공간 변환]
    ├─ [마스크 생성 및 형태학적 연산]
    ├─ [컨투어 검출 및 필터링]
    ├─ [객체 위치 계산]
    ├─ [오차 계산]
    ├─ [PID 제어]
    └─ [Twist 메시지 생성]
    ↓
[로봇 제어] (is_running=true일 때만)
    └─ [Twist 메시지 발행]
```

# 코드 분석

---

## 1. 프로그램 시작 및 초기화

### **노드 초기화**

```python
class OjbectTrackingNode(Node):
    """
    객체 추적 ROS2 노드
    카메라 이미지를 받아 객체를 추적하고 Visual Servoing을 통해 로봇을 제어합니다.
    """
    def __init__(self, name):
        """
        노드 초기화
        
        Args:
            name: 노드 이름
        """
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.name = name
        
        # 상태 변수 초기화
        self.set_callback = False  # 마우스 콜백 설정 여부
        self.color_picker = None  # 색상 선택기
        self.tracker = None  # 객체 추적기
        self.is_running = False  # 추적 실행 여부
        self.threshold = 0.1  # 색상 검출 임계값
        self.dist_threshold = 0.3  # 거리 임계값
        self.lock = threading.RLock()  # 스레드 동기화를 위한 락
        
        # 이미지 관련 변수
        self.image_sub = None  # 이미지 구독자
        self.result_image = None  # 결과 이미지
        self.image_height = None  # 이미지 높이
        self.image_width = None  # 이미지 너비
        self.bridge = CvBridge()  # ROS 이미지와 OpenCV 이미지 변환용
        self.image_queue = queue.Queue(2)  # 디버그용 이미지 큐
        
        # 발행자 생성
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 로봇 속도 제어
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 결과 이미지 발행
        
        # 서비스 생성
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)  # 추적 모드 진입
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)  # 추적 모드 종료
        self.set_running_srv = self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)  # 추적 시작/중지
        self.set_target_color_srv = self.create_service(SetPoint, '~/set_target_color',
                                                          self.set_target_color_srv_callback)  # 마우스 클릭 색상 설정
        self.get_target_color_srv = self.create_service(Trigger, '~/get_target_color',
                                                          self.get_target_color_srv_callback)  # 현재 추적 색상 조회
        self.set_threshold_srv = self.create_service(SetFloat64, '~/set_threshold',
                                                        self.set_threshold_srv_callback)  # 임계값 설정
        self.set_large_model_target_color_srv = self.create_service(SetString, '~/set_large_model_target_color',
                                                                      self.set_large_model_target_color_srv_callback)  # 대형 모델용 색상 설정 서비스 생성

        # 서보 제어 발행자 (호환성을 위해 유지)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        
        # 하트비트 서비스: 5초 동안 하트비트가 없으면 자동으로 종료
        Heart(self, self.name + '/heartbeat', 5,
              lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))
        
        # 디버그 모드 설정 확인
        self.debug = self.get_parameter('debug').value
        if self.debug:
            # 디버그 모드일 경우 화면 표시를 위한 별도 스레드 시작
            threading.Thread(target=self.main, daemon=True).start()
        
        # 초기화 완료 서비스 생성
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        # 음성 인식 웨이크업 구독 (대형 모델 추적 중단용)
        self.wakeup_sub = self.create_subscription(Bool, '/vocal_detect/wakeup', self.wakeup_callback, 1)

        # 목표 추적 실패 관련 변수 초기화
        self.get_logger().info(f"LAB 색상 데이터 파일 경로: {yaml_handle.lab_file_path}")
        self.target_lost = False  # 목표 추적 실패 여부 플래그
        self.large_model_tracking = False  # LLM에 의해 시작된 추적인지 여부
        self.target_lost_timer = None  # 목표 추적 실패 후 자동 정지 타이머

        # 카메라 타입 가져오기
        self.camera_type = os.environ['DEPTH_CAMERA_TYPE']

        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
```

- **초기화 과정**
    1. ROS2 초기화 및 노드 생성
    2. 상태 변수 초기화 (tracker, color_picker, is_running 등)
    3. 발행자 생성 (로봇 제어, 결과 이미지)
    4. 서비스 생성 (enter, exit, set_running, set_target_color 등)
    5. 하트비트 서비스 생성 (5초 타임아웃)
    6. 디버그 모드 확인 및 화면 표시 스레드 시작
    7. 환경 변수 확인 (카메라 타입)


## 2. Enter/Exit 서비스

### **Enter 서비스**

```python
def enter_srv_callback(self, request, response):
    """
    추적 모드 진입 서비스 콜백
    객체 추적 모드로 진입하고 이미지 구독을 시작합니다.
    """
    self.get_logger().info('\033[1;32m%s\033[0m' % 'object tracking enter')
    self.get_logger().info(f"LAB 색상 데이터 파일 경로: {yaml_handle.lab_file_path}")
    with self.lock:
        # 상태 초기화
        self.is_running = False
        self.threshold = 0.5
        self.tracker = None
        self.color_picker = None
        self.dist_threshold = 0.3
        self.large_model_tracking = False  # 진입 시 대형 모델 추적 플래그 초기화
        
        # 이미지 구독 시작 (아직 구독하지 않은 경우에만)
        if self.image_sub is None:
            self.image_sub = self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image',
                                                        self.image_callback, 1)  # 카메라 이미지 구독
        
        # 로봇 정지 명령 발행
        self.mecanum_pub.publish(Twist())
    response.success = True
    response.message = "enter"
    return response
```

### **Exit 서비스**

```python
def exit_srv_callback(self, request, response):
    """
    추적 모드 종료 서비스 콜백
    객체 추적 모드를 종료하고 이미지 구독을 중지합니다.
    """
    self.get_logger().info('\033[1;32m%s\033[0m' % 'object tracking exit')
    try:
        # 이미지 구독 중지
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
    except Exception as e:
        self.get_logger().error(str(e))
    with self.lock:
        # 상태 초기화
        self.is_running = False
        self.color_picker = None
        self.tracker = None
        self.threshold = 0.5
        self.dist_threshold = 0.3
        
        # 로봇 정지 명령 발행
        self.mecanum_pub.publish(Twist())
        
        # 대형 모델 추적 관련 정리
        self.large_model_tracking = False  # 대형 모델 추적 플래그 해제
        if self.target_lost_timer is not None:
            self.target_lost_timer.cancel()  # 타이머 취소
            self.target_lost_timer = None
        self.target_lost = False  # 종료 시 목표 추적 실패 플래그 초기화
    response.success = True
    response.message = "exit"
    return response
```

## 3.  색상 설정

### **마우스 클릭 색상 설정**

```python
def set_target_color_srv_callback(self, request, response):
    """
    추적 대상 색상 설정 서비스 콜백
    마우스 클릭 위치의 색상을 추적 대상으로 설정합니다.
    
    Args:
        request.data.x, request.data.y: 이미지 상대 좌표 (0.0~1.0)
                                      -1, -1이면 색상 선택 취소
    """
    self.get_logger().info('\033[1;32m%s\033[0m' % 'set_target_color')
    with self.lock:
        x, y = request.data.x, request.data.y
        if x == -1 and y == -1:
            # 색상 선택 취소
            self.color_picker = None
            self.tracker = None
        else:
            # 색상 선택기 생성 (10프레임 동안 샘플링)
            self.color_picker = ColorPicker(request.data, 10)
        
        # 로봇 정지 명령 발행
        self.mecanum_pub.publish(Twist())
    response.success = True
    response.message = "set_target_color"
    return response
```

## 4.  이미지 처리 및 객체 추적

### **이미지 콜백**

```python
def image_callback(self, ros_image):
    """
    이미지 콜백 함수
    카메라로부터 받은 이미지를 처리하여 객체를 추적하고 로봇을 제어합니다.
    
    처리 순서:
    1. ROS 이미지를 OpenCV 형식으로 변환
    2. ColorPicker 모드 또는 ObjectTracker 모드 확인
    3. 객체 추적 및 로봇 제어 명령 생성
    4. 결과 이미지 발행 또는 화면 표시
    """
    # ROS 이미지 형식을 OpenCV RGB 형식으로 변환
    cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
    rgb_image = np.array(cv_image, dtype=np.uint8)
    self.image_height, self.image_width = rgb_image.shape[:2]

    # 결과 표시용 이미지 복사
    result_image = np.copy(rgb_image)
    with self.lock:
        # 색상 선택기와 객체 추적은 상호 배타적
        # 색상 선택기가 있으면 색상 선택 모드, 없으면 객체 추적 모드
        if self.color_picker is not None:  # 색상 선택 모드
            # 색상 선택기로부터 색상 정보 획득
            target_color, result_image = self.color_picker(rgb_image, result_image)
            if target_color is not None:
                # 색상 선택 완료: ObjectTracker 생성
                self.color_picker = None
                self.tracker = ObjectTracker(target_color, self)
                self.get_logger().info("추적 대상 색상 설정 완료: {}".format(target_color))
        else:
            # 객체 추적 모드
            if self.tracker is not None:
                try:
                    # 객체 추적 수행 및 로봇 제어 명령 생성
                    result_image, twist = self.tracker(rgb_image, result_image, self.threshold)

                    if self.is_running:
                        # 추적 실행 중이면 로봇 제어 명령 발행
                        self.mecanum_pub.publish(twist)

                        # 목표를 다시 찾았을 때 타이머 취소
                        # lost_target_count가 threshold 이하이면 추적 성공으로 간주
                        if self.large_model_tracking and self.target_lost and self.target_lost_timer is not None and self.tracker.lost_target_count <= self.tracker.lost_threshold:
                            self.target_lost_timer.cancel()
                            self.target_lost_timer = None
                            self.target_lost = False
                            self.get_logger().info("목표 재발견, 타이머 중지 (Target reacquired, stopping timer)")

                    else:
                        # 추적이 중지된 상태면 PID 제어기 초기화
                        self.tracker.pid_dist.clear()
                        self.tracker.pid_yaw.clear()
                except Exception as e:
                    self.get_logger().error(str(e))

                # 목표 추적 실패 감지 및 자동 정지 타이머 시작
                # 연속으로 목표를 잃었고, 아직 타이머가 시작되지 않았을 때만 시작
                if self.large_model_tracking and self.is_running and self.tracker.lost_target_count > self.tracker.lost_threshold and not self.target_lost:
                    self.start_stop_timer()  # image_callback 내에서 호출
                    self.get_logger().warn("Track Lost 감지, 자동 정지 타이머 시작... (Target lost, starting stop timer...)")

    # 디버그 모드 또는 일반 모드에 따라 결과 처리
    if self.debug:
        # 디버그 모드: 화면 표시를 위해 이미지 큐에 추가
        if self.image_queue.full():
            # 큐가 가득 차면 가장 오래된 이미지 제거
            self.image_queue.get()
        # 이미지를 큐에 추가
        self.image_queue.put(result_image)
    else:
        # 일반 모드: 결과 이미지를 ROS 토픽으로 발행
        # OpenCV는 BGR 형식을 사용하지만 ROS는 RGB 형식을 사용하므로 변환
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))
```

## **5. ObjectTracker 클래스**

### 초기화 (**`__init__`)**

```python
class ObjectTracker:
    """
    객체 추적 클래스
    LAB 색상 공간을 이용하여 객체를 검출하고, PID 제어를 통해 로봇을 제어합니다.
    """
    def __init__(self, color, node, set_color=None, set_status=False):
        """
        ObjectTracker 초기화
        
        Args:
            color: 마우스 클릭으로 선택한 색상 (LAB, RGB 튜플) 또는 None
            node: ROS2 노드 객체
            set_color: 사전 정의된 색상 이름 (예: 'red', 'blue')
            set_status: True면 사전 정의된 색상 사용, False면 마우스 선택 색상 사용
        """
        self.node = node
        self.machine_type = os.environ['MACHINE_TYPE']
        self.camera_type = node.camera_type

        # PID 제어기 초기화 (수평 방향: yaw, 수직 방향: 거리)
        self.pid_yaw = pid.PID(0.006, 0.0, 0.0)
        self.pid_dist = pid.PID(0.002, 0.0, 0.00)
        
        # 객체 추적 관련 변수
        self.last_color_circle = None  # 이전 프레임의 객체 위치 (중심 좌표, 반지름)
        self.lost_target_count = 0  # 목표를 연속으로 잃은 프레임 수
        self.lost_threshold = 5  # 목표 추적 실패로 판단할 연속 프레임 수
        self.x_stop = 320  # 수평 방향 목표 위치 (픽셀)

        # LAB 색상 데이터 YAML 파일 로드
        try:
            self.lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
            # ... (에러 처리 생략)
        except Exception as e:
            self.node.get_logger().error(f"YAML 파일 로드 또는 파싱 실패: {e}")
            raise

        # 색상 설정 관련 변수
        self.set_status = set_status
        self.set_color = set_color
        if color is not None:
            self.target_lab, self.target_rgb = color  # 마우스 선택 색상 (LAB, RGB)

        # 카메라 타입에 따른 목표 위치 및 처리 크기 설정
        if self.camera_type == 'ascamera':
            self.y_stop = 300  # 수직 방향 목표 위치
            self.pro_size = (320, 180)  # 이미지 처리 크기
        else:
            self.y_stop = 400
            self.pro_size = (320, 240)
```

### 메인 Tracking 함수 (`__call__`)

> 메인 Tracking 함수는 이미지 처리부터 로봇 제어까지의 전체 과정을 담당합니다. 주요 단계별로 나누어 설명합니다.

일반적인 `__init__`﻿ 메서드가 객체 생성 시 초기화를 담당하는 반면, `__call__`﻿은 이미 생성된 인스턴스가 호출될 때 동작합니다.
> 

**이미지 전처리 및 색상 범위 설정**

```python
def __call__(self, image, result_image, threshold):
    """
    객체 추적 및 로봇 제어 메인 함수
    
    Args:
        image: 입력 RGB 이미지
        result_image: 결과 표시용 이미지
        threshold: 색상 검출 임계값
        
    Returns:
        result_image: 결과 이미지
        twist: 로봇 제어 명령 (Twist 메시지)
    """
    twist = Twist()
    h, w = image.shape[:2]
    
    # 이미지 전처리: 크기 조정, LAB 색상 공간 변환, 가우시안 블러
    image = cv2.resize(image, self.pro_size)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # RGB를 LAB 색상 공간으로 변환
    image = cv2.GaussianBlur(image, (5, 5), 5)  # 노이즈 제거를 위한 가우시안 블러

    # 색상 범위 설정: 마우스 선택 색상 또는 사전 정의된 색상 사용
    if self.set_status == False:
        # 마우스로 선택한 색상 기준으로 범위 설정
        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color
    else:
        # 사전 정의된 색상 사용 (YAML 파일에서 읽기)
        camera_type = 'Stereo' if self.camera_type == 'ascamera' else 'Mono'
        # ... (에러 처리 생략)
        color_data = self.lab_data['lab'][camera_type][self.set_color]
        min_color = color_data['min']
        max_color = color_data['max']
        target_color = 0, min_color, max_color

```

### **객체 검출 및 추적**

```bash
# 객체 검출: 마스크 생성 및 형태학적 연산
mask = cv2.inRange(image, tuple(target_color[1]), tuple(target_color[2]))  # LAB 색상 범위로 이진화

# 형태학적 연산: 노이즈 제거를 위한 침식 및 팽창
eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 침식: 작은 노이즈 제거
dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 팽창: 객체 크기 복원

# 컨투어 검출 및 필터링
contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 외곽선 검출
contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)  # 각 컨투어의 면적 계산
contour_area = list(filter(lambda c: c[1] > 40, contour_area))  # 면적이 40보다 작은 컨투어 제거 (노이즈 필터링)

# 객체 추적: 이전 위치 기반으로 가장 가까운 객체 선택
circle = None
if len(contour_area) > 0:
    if self.last_color_circle is None:
        # 첫 프레임이거나 이전 위치가 없으면 가장 큰 컨투어 선택
        contour, area = max(contour_area, key=lambda c_a: c_a[1])
        circle = cv2.minEnclosingCircle(contour)
    else:
        # 이전 위치가 있으면 가장 가까운 객체 선택
        (last_x, last_y), last_r = self.last_color_circle
        circles = map(lambda c: cv2.minEnclosingCircle(c[0]), contour_area)
        # 각 객체와 이전 위치 간의 거리 계산
        circle_dist = list(map(lambda c: (c, math.sqrt(((c[0][0] - last_x) ** 2) + ((c[0][1] - last_y) ** 2))),
                               circles))
        circle, dist = min(circle_dist, key=lambda c: c[1])  # 가장 가까운 객체 선택
        if dist < 100:  # 100 픽셀 이내의 객체만 유효한 것으로 간주
            circle = circle
```

### **PID 제어 및 로봇 제어**

```python
# 객체를 찾은 경우: PID 제어를 통한 로봇 제어
if circle is not None:
    self.lost_target_count = 0  # 목표를 찾았으므로 카운터 초기화
    (x, y), r = circle
    
    # 처리된 이미지 좌표를 원본 이미지 좌표로 변환
    x = x / self.pro_size[0] * w
    y = y / self.pro_size[1] * h
    r = r / self.pro_size[0] * w

    # 결과 이미지에 목표 위치 및 검출된 객체 표시
    cv2.circle(result_image, (self.x_stop, self.y_stop), 5, (255, 255, 0), -1)
    # ... (객체 표시 코드 생략)

    # 수직 방향 제어 (거리/선속도): y 좌표가 목표 위치와 20픽셀 이상 차이날 때만 제어
    if abs(y - self.y_stop) > 20:
        self.pid_dist.update(y - self.y_stop)  # 오차 업데이트
        twist.linear.x = common.set_range(self.pid_dist.output, -0.35, 0.35)  # 선속도 제한
    else:
        self.pid_dist.clear()  # 목표 위치에 도달했으면 PID 초기화
    
    # 수평 방향 제어 (각속도): x 좌표가 목표 위치와 20픽셀 이상 차이날 때만 제어
    if abs(x - self.x_stop) > 20:
        self.pid_yaw.update(x - self.x_stop)  # 오차 업데이트
        
        # 로봇 타입별 제어 방식
        if self.machine_type == 'MentorPi_Acker':
            # 조향각 기반 제어 (Ackermann 스티어링)
            steering_angle = common.set_range(-self.pid_yaw.output, -math.radians(322 / 2000 * 180),
                                               math.radians(322 / 2000 * 180))
            if steering_angle != 0:
                R = 0.145 / math.tan(steering_angle)  # 회전 반경 계산
                twist.angular.z = -twist.linear.x / R  # 각속도 계산
        else:
            # 일반적인 각속도 제어
            twist.angular.z = common.set_range(self.pid_yaw.output, -2, 2)
    else:
        self.pid_yaw.clear()  # 목표 위치에 도달했으면 PID 초기화
    
    # 이전 위치 업데이트
    self.last_color_circle = ((x, y), r)
    
    return result_image, twist  # 객체를 찾았으므로 제어 명령 반환

else:  # 객체를 찾지 못한 경우
    self.lost_target_count += 1
    return result_image, Twist()  # 목표를 잃었으므로 정지 명령 반환
```

## 중요 서비스 정리

### Set Target Color 서비스

- 마우스 클릭 위치의 색상을 추적 대상으로 설정
- **서비스 이름**: `~/set_target_color`
- **서비스 타입**: `interfaces/srv/SetPoint`
    - **요청**
        - `data.x`: 이미지 상대 x 좌표 (0.0~1.0) 또는 -1 (색상 선택 취소)
        - `data.y`: 이미지 상대 y 좌표 (0.0~1.0) 또는 -1 (색상 선택 취소)
    - **응답**
        - `success`: 성공 여부
        - `message`: “set_target_color”

### Set Threshold 서비스

- 색상 검출 임계값을 설정
- **서비스 이름**: `~/set_threshold`
- **서비스 타입**: `interfaces/srv/SetFloat64`
    - **요청**
        - `data`: 임계값 (기본값: 0.1)
    - **응답**
        - `success`: 성공 여부
        - `message`: “set_threshold”


## 코드 흐름도

### Launch 파일 실행 흐름

```
[Launch 파일 실행]
    ↓
[환경 변수 확인] (need_compile)
    ↓
[Debug 모드 확인]
    ↓
[Depth Camera Launch 포함] (debug=true일 때만)
    ㄴ [카메라 드라이버 노드 시작]
    ㄴ [이미지 발행 노드 시작]
    ↓
[Controller Launch 포함] (debug=true일 때만)
    ㄴ [로봇 제어기 노드 시작]
    ↓
[Object Tracking 노드 시작]
    ㄴ [debug 파라미터 전달]
```
### Object Tracking 노드 실행 흐름

```
[Object Tracking 노드 시작]
    ↓
[ROS2 노드 초기화]
    ├─ PID 제어기 초기화
    ├─ 발행자/구독자 생성
    ├─ 서비스 생성
    └─ 상태 변수 초기화
    ↓
[Enter 서비스 호출]
    ├─ 이미지 구독 시작
    └─ 상태 초기화
    ↓
[색상 설정]
    ├─ [마우스 클릭] → ColorPicker 생성
    │   └─ [10프레임 샘플링] → ObjectTracker 생성
    ↓
[Set Running 서비스 호출] (is_running = true)
    ↓
[이미지 콜백 루프]
    ├─ [이미지 수신]
    ├─ [ColorPicker 모드]
    │   ├─ [색상 샘플링] → ObjectTracker 생성
    └─ [ObjectTracker 모드]
        ├─ [LAB 색상 공간 변환]
        ├─ [마스크 생성 및 형태학적 연산]
        ├─ [컨투어 검출 및 필터링]
        ├─ [객체 추적] (이전 위치 기반)
        ├─ [위치 계산] (원본 좌표로 변환)
        ├─ [오차 계산]
        │   ├─ 수평 방향: x - x_stop
        │   └─ 수직 방향: y - y_stop
        ├─ [PID 제어]
        │   ├─ 수평 방향: 각속도 제어
        │   └─ 수직 방향: 선속도 제어
        ├─ [Twist 메시지 생성]
        └─ [로봇 제어] (is_running=true일 때만)
    ↓
[Exit 서비스 호출]
    ├─ 이미지 구독 중지
    └─ 로봇 정지
```
