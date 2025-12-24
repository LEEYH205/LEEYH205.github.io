---
title: "Camera 테스트3: Line Follower"
date: 2025-12-24 09:40:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, Camera, Line Follower]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

# Line Follower

---

## 개요

> OpenCV와 ROS2를 활용하여 카메라 이미지에서 선을 실시간으로 감지하고, PID 제어를 통해 로봇이 선을 따라 이동하도록 하는 프로젝트 입니다. LAB 색공간을 사용하여 조명 변화에 강건한 선 인식을 구현하며, 여러 ROI(Region of Interest) 영역에서 선을 검출하여 가중 평균으로 중심 위치를 계산합니다. 또한 라이다를 사용하여 장애물을 감지하고 자동으로 정지하는 기능을 포함합니다.
> 

**주요 기능**

- **선 인식**: LAB 색공간을 사용한 조명 변화에 강건한 선 검출
- **다중 ROI 검출**: 여러 영역에서 선을 검출하여 안정적인 중심 위치 계산
- **PID 제어**: 선의 편차 각도를 PID 제어로 로봇 각속도 조정
- **장애물 회피**: 라이다를 사용한 전방 장애물 감지 및 자동 정지
- **색상 선택**: ColorPicker를 통한 동적 색상 선택 또는 YAML 파일의 사전 정의된 색상 사용

## 동작 확인 (디버그 모드)

1. MentorPi를 시작하고 VNC Viewer에 연결합니다.
2. **Terminator**에서 아래 명령어를 입력해 ROS 2관련 자동 실행 서비스를 중지합니다.
    
    ```bash
    ~/.stop_ros.sh
    ```
    
3. 라인 추종 기능을 활성화하기 위해 명령어를 입력합니다.
    
    ```bash
    ros2 launch app line_following_node.launch.py debug:=true
    ```
    
4. 새 터미널에서 카메라를 실행하고 목표 라인을 선택하는 명령을 입력합니다.
    
    ```bash
    ros2 service call /line_following/enter std_srvs/srv/Trigger {}
    ```
    
5. 아래 명령으로 라인 추종을 시작합니다.
    
    ```bash
    ros2 service call /line_following/set_running std_srvs/srv/SetBool "{data: True}"
    ```
    
6. 실습을 종료하려면 터미널에서 'Ctrl+C'를 누르거나 아래 명령어를 입력합니다.
    
    ```bash
    ros2 service call /line_following/exit std_srvs/srv/Trigger {}
    ```
    
7. **LXTerminal**에서 아래 명령어로 로봇 기능들을 다시 활성화하거나, 로봇을 재시작할 수 있습니다.
    
    ```bash
    sudo systemctl restart start_node.service
    ```
    

## 동작 원리

1. **이미지 수신**: ROS2를 통해 카메라 이미지를 구독
2. **색상 설정**: ColorPicker를 사용하여 선의 색상을 선택하거나, YAML 파일의 사전 정의된 색상을 사용
3. **다중 ROI 검출**: 이미지 하단부터 상단까지 여러 영역(ROI)에서 선을 검출
4. **선 인식 처리**: 각 ROI에서 LAB 색공간 변환, 마스킹, 모폴로지 연산, 컨투어 검출을 수행
5. **중심 위치 계산**: 각 ROI에서 검출된 선의 중심 위치를 가중 평균하여 전체 중심 위치를 계산
6. **편차 각도 계산**: 중심 위치와 이미지 중심의 차이를 이용하여 편차 각도를 계산
7. **PID 제어**: 편차 각도를 PID 제어기로 처리하여 각속도를 계산
8. **로봇 제어**: 계산된 선속도와 각속도를 Twist 메시지로 발행하여 로봇을 제어
9. **장애물 감지**: 라이다 데이터를 분석하여 전방 장애물을 감지하고, 일정 거리 이내에 장애물이 있으면 정지

![](/assets/img/HiWonderPi/gif/IMG_4437.gif)
![](/assets/img/HiWonderPi/gif/Peek%202025-12-24%2009-56.gif)


라인이 안보일 땐
 - 후진하기
 - 카메라를 움직여서 라인을 찾기
등의 로직을 추가하면 될 듯하다.

## 프로그램 구조

![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-24%2010-02-29.png)
```
[프로그램 시작]
    ↓
[ROS2 노드 초기화 및 서비스/토픽 설정]
    ↓
[Enter 서비스 호출]
    ├─ 이미지 구독 시작
    ├─ 라이다 구독 시작
    └─ 초기화
    ↓
[색상 설정]
    ├─ ColorPicker 사용 (동적 색상 선택)
    └─ 또는 YAML 파일 색상 사용 (사전 정의)
    ↓
[이미지 처리 루프]
    ├─ [이미지 수신]
    ├─ [다중 ROI 선 검출]
    ├─ [중심 위치 계산]
    ├─ [편차 각도 계산]
    ├─ [PID 제어]
    ├─ [로봇 제어 명령 발행]
    └─ [장애물 감지 및 정지]
```

# 주요 사용 기술

---

## ROI (Region of Interest) 기반 선 검출
### **ROI란?**

> ROI는 이미지에서 관심 있는 영역을 의미합니다. Line Follower에서는 이미지의 여러 영역에서 선을 검출하여 더 안정적인 결과를 얻습니다.
> 

### **ROI 설정**

```python
# ascamera의 경우
self.rois = ((0.9, 0.95, 0, 1, 0.7),   # 하단 영역 (가중치 0.7)
             (0.8, 0.85, 0, 1, 0.2),   # 중간 영역 (가중치 0.2)
             (0.7, 0.75, 0, 1, 0.1))   # 상단 영역 (가중치 0.1)
```

- 각 튜플: (y_min 비율, y_max 비율, x_min 비율, x_max 비율, 가중치)
- 하단 영역에 높은 가중치를 부여하여 로봇에 가까운 선을 우선적으로 추적

### **가중 평균 계산**

```python
centroid_sum += line_center_x * roi[-1]  # 각 ROI의 중심 x 좌표 × 가중치
center_pos = centroid_sum / self.weight_sum  # 전체 가중 평균
```

## 편차 각도 계산

### **편차 각도란?**

> 로봇의 현재 위치에서 선의 중심까지의 각도 차이를 의미합니다. 이 각도는 로봇이 선을 따라가기 위해 회전해야 할 각도입니다.
> 

### **계산 방법**

```python
deflection_angle = -math.atan((center_pos - (w / 2.0)) / (h / 2.0))
```

- **수식 설명**
    - `center_pos`: 선의 중심 x 좌표 (가중 평균)
    - `w / 2.0`: 이미지 중심 x 좌표
    - `h / 2.0`: 이미지 높이의 절반 (각도 계산을 위한 기준 거리)
    - `atan`: 아크탄젠트 함수로 각도 계산
    - 음수 부호: 좌표계 변환 (이미지 좌표계 → 로봇 좌표계)
- **각도 의미**
    - `deflection_angle > 0`: 선이 오른쪽에 있음 → 오른쪽으로 회전 필요
    - `deflection_angle < 0`: 선이 왼쪽에 있음 → 왼쪽으로 회전 필요
    - `deflection_angle = 0`: 선이 정중앙에 있음 → 직진

## PID 제어

`LiDAR 테스트2: Follower` 참고

## 라이다 기반 장애물 감지

### **프로그램 상에서 장애물 감지 원리**

> 라이다는 360도 방향으로 거리 데이터를 제공합니다. Line Follower는 전방 좌우 각 45도 범위에서 가장 가까운 거리를 측정하여 장애물을 감지합니다.
> 
- **처리 과정**
    1. 라이다 데이터에서 좌우 각 45도 범위의 거리 데이터 추출
    2. 유효한 거리 값만 필터링 (0이 아닌 값, 유한한 값)
    3. 각 방향에서 최소 거리 계산
    4. 최소 거리가 `stop_threshold` (기본 0.4m) 미만이면 정지 플래그 설정
    5. 일정 횟수 이상 안전하면 정지 플래그 해제 (노이즈 필터링)
- **스캔 각도**
    
    ```python
    MAX_SCAN_ANGLE = 240  # 전체 240도 스캔 (항상 가려지는 부분 제외)
    self.scan_angle = math.radians(45)  # 좌우 각 45도 범위 검사
    ```

## ColorPicker

### **ColorPicker란?**

> 이미지의 특정 위치를 클릭하여 해당 위치의 색상을 추출하는 도구입니다. 여러 프레임에 걸쳐 색상을 수집하고 평균을 계산하여 안정적인 색상 값을 얻습니다.
> 

**동작 과정**

1. 사용자가 이미지의 특정 위치를 클릭
2. 해당 위치 주변 2×2 픽셀 영역의 색상 추출
3. 여러 프레임에 걸쳐 색상 수집 (기본 5프레임)
4. 수집된 색상의 평균 계산 (LAB 및 RGB)
5. 평균 색상 값을 반환하여 LineFollower에 전달

```python
# 마우스 클릭으로 색상 선택
self.color_picker = ColorPicker(request.data, 5)  # 5프레임 수집
```

# 코드 분석

---

## 1. 프로그램 시작 및 초기화

### **노드 초기화**

```python
class LineFollowingNode(Node):
    def __init__(self, name):
        rclpy.init()  # ROS2 초기화
        super().__init__(name, allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.name = name
        self.set_callback = False
        self.is_running = False  # 실행 상태 플래그
        self.color_picker = None  # 색상 선택기
        self.follower = None  # LineFollower 인스턴스
        self.scan_angle = math.radians(45)  # 라이다 스캔 각도
        self.pid = pid.PID(0.005, 0.001, 0.0)  # PID 제어기 초기화
        self.stop = False  # 정지 플래그
        self.threshold = 0.5  # 색상 임계값
        self.stop_threshold = 0.4  # 장애물 정지 거리 (미터)
        self.lock = threading.RLock()  # 스레드 동기화 락
        
        # 발행자 및 구독자 초기화
        self.pwm_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 10)
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 로봇 제어
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 결과 이미지
        
        # 서비스 생성
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)
        self.create_service(SetPoint, '~/set_target_color', self.set_target_color_srv_callback)
        self.create_service(Trigger, '~/get_target_color', self.get_target_color_srv_callback)
        self.create_service(SetFloat64, '~/set_threshold', self.set_threshold_srv_callback)
        self.create_service(SetString, '~/set_large_model_target_color',
                            self.set_large_model_target_color_srv_callback)
```

- **초기화 과정**
    1. ROS2 초기화 및 노드 생성
    2. 상태 변수 초기화 (실행 상태, 색상 선택기, LineFollower 등)
    3. PID 제어기 초기화
    4. 발행자 및 구독자 생성 (초기에는 None)
    5. 서비스 생성 (Enter, Exit, 색상 설정 등)
- **주요 파라미터**
    - 색상 처리 비율(`self.threshold`)
        - 색상 처리와 이진 이미지 처리의 가중치(기본값 0.5)를 의미하며, 라인 검출에 쓰이는 최소·최대 색상 범위를 설정
    - 정지 임계값(`self.stop_threshold`)
        - 로봇이 멈추는 조건을 설정하는 값(기본값 0.4)
        - 로봇과 장애물 좌우 거리(`min_dist_left`, `min_dist_right`)가 해당 값보다 작은 경우 정지 동작을 수행
    - 각도 스윙 범위(`self.scan_angle`)
        - 라인 추종 동작 중 로봇이 움직이는 각도의 범위를 설정(기본값 45도)
        - 값이 커지면 로봇의 좌우 움직임(스윙)이 커지고, 값이 작으면 스윙 폭이 줄어들지만 인식 성능에 영향이 있음

## 2. Enter 및 Exit 서비스

### **Enter 서비스 (모드 진입)**

```python
def enter_srv_callback(self, request, response):
    self.get_logger().info('\033[1;32m%s\033[0m' % "line following enter")
    if os.environ['DEPTH_CAMERA_TYPE'] != 'ascamera':
        self.pwm_controller([1850, 1500])  # 카메라 각도 조정
    with self.lock:
        self.stop = False
        self.is_running = False
        self.color_picker = None
        self.pid = pid.PID(1.1, 0.0, 0.0)  # PID 파라미터 재설정
        self.follower = None
        self.threshold = 0.5
        self.empty = 0
        self.large_model_tracking = False
        # 이미지 구독 시작
        if self.image_sub is None:
            self.image_sub = self.create_subscription(Image, 'ascamera/camera_publisher/rgb0/image',
                                                        self.image_callback, 1)
        # 라이다 구독 시작
        if self.lidar_sub is None:
            qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, qos)
        self.mecanum_pub.publish(Twist())  # 정지 명령
    response.success = True
    response.message = "enter"
    return response
```

### **Exit 서비스 (모드 종료)**

```python
def exit_srv_callback(self, request, response):
    self.get_logger().info('\033[1;32m%s\033[0m' % "line following exit")
    try:
        # 구독 해제
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        if self.lidar_sub is not None:
            self.destroy_subscription(self.lidar_sub)
            self.lidar_sub = None
    except Exception as e:
        self.get_logger().error(str(e))
    with self.lock:
        self.is_running = False
        self.color_picker = None
        self.pid = pid.PID(0.00, 0.001, 0.0)  # PID 비활성화
        self.follower = None
        self.threshold = 0.5
        self.mecanum_pub.publish(Twist())  # 정지 명령
        self.large_model_tracking = False
        # 타이머 정리
        if self.target_lost_timer is not None:
            self.target_lost_timer.cancel()
            self.target_lost_timer = None
    response.success = True
    response.message = "exit"
    return response
```

## 3. 색상 설정

### **색상 설정 서비스**

```python
def set_target_color_srv_callback(self, request, response):
    self.get_logger().info('\033[1;32m%s\033[0m' % "set_target_color")
    with self.lock:
        x, y = request.data.x, request.data.y
        self.follower = None
        if x == -1 and y == -1:
            self.color_picker = None  # 색상 선택 취소
        else:
            # ColorPicker 생성 (5프레임 수집)
            self.color_picker = ColorPicker(request.data, 5)
            self.mecanum_pub.publish(Twist())  # 정지 명령
    response.success = True
    response.message = "set_target_color"
    return response
```

### **사전 정의된 색상 설정**

```python
def set_large_model_target_color_srv_callback(self, request, response):
    self.get_logger().info('\033[1;32m%s\033[0m' % "set_large_model_target_color")
    with self.lock:
        color_name = request.data  # 색상 이름 (예: 'red', 'green', 'blue')
        # YAML 파일의 색상 사용
        self.follower = LineFollower(None, self, color_name)
        self.mecanum_pub.publish(Twist())
        self.large_model_tracking = True
        self.get_logger().info("由大模型启动巡线 (Line following started by large model)")
    response.success = True
    response.message = "set_large_model_target_color"
    return response
```

### 4. LineFollower 클래스

### **초기화**

```python
class LineFollower:
    def __init__(self, color, node, set_color=None):
        self.node = node
        self.set_color = set_color  # 색상 이름 (YAML 파일 사용 시)
        self.target_lab = None  # 목표 LAB 색상
        self.target_rgb = None  # 목표 RGB 색상
        self.camera_type = os.environ['DEPTH_CAMERA_TYPE']

        if color is not None:
            self.target_lab, self.target_rgb = color  # ColorPicker로부터 받은 색상

        self.lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
        # ROI 설정 (카메라 타입에 따라 다름)
        if os.environ['DEPTH_CAMERA_TYPE'] == 'ascamera':
            self.rois = ((0.9, 0.95, 0, 1, 0.7), (0.8, 0.85, 0, 1, 0.2), (0.7, 0.75, 0, 1, 0.1))
        else:
            self.rois = ((0.81, 0.83, 0, 1, 0.7), (0.69, 0.71, 0, 1, 0.2), (0.57, 0.59, 0, 1, 0.1))
        self.weight_sum = 1.0  # 가중치 합
        self.lost_target_count = 0  # 목표를 잃은 횟수 카운팅
```

### **선 검출 및 편차 각도 계산**

```python
def __call__(self, image, result_image, threshold):
    centroid_sum = 0
    h, w = image.shape[:2]
    if os.environ['DEPTH_CAMERA_TYPE'] == 'ascamera':
        w = w + 200  # ascamera의 경우 가상 너비 확장

    # 색상 임계값 설정
    if self.set_color is None:  # ColorPicker 사용
        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
    else:  # YAML 파일 색상 사용
        camera_type = 'Stereo' if self.camera_type == 'ascamera' else 'Mono'
        if 'lab' in self.lab_data and camera_type in self.lab_data['lab']:
            if self.set_color in self.lab_data['lab'][camera_type]:
                color_data = self.lab_data['lab'][camera_type][self.set_color]
                min_color = color_data['min']
                max_color = color_data['max']
            else:
                self.node.get_logger().error(f"Color '{self.set_color}' not found")
                return result_image, None
        else:
            self.node.get_logger().error("Invalid lab_data.yaml structure")
            return result_image, None

    target_color = 0, min_color, max_color

    # 각 ROI에서 선 검출
    for roi in self.rois:
        # ROI 영역 추출
        blob = image[int(roi[0] * h):int(roi[1] * h), int(roi[2] * w):int(roi[3] * w)]
        img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB)  # LAB 색공간 변환
        img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 가우시안 블러
        
        # 색상 마스킹
        mask = cv2.inRange(img_blur, tuple(target_color[1]), tuple(target_color[2]))
        # 모폴로지 연산
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        
        # 컨투어 검출
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
        max_contour_area = self.get_area_max_contour(contours, 30)  # 최대 면적 컨투어
        
        if max_contour_area is not None:
            # 최소 외접 사각형 계산
            rect = cv2.minAreaRect(max_contour_area[0])
            box = np.intp(cv2.boxPoints(rect))  # 4개 꼭짓점
            for j in range(4):
                box[j, 1] = box[j, 1] + int(roi[0] * h)  # ROI 좌표를 전체 이미지 좌표로 변환
            
            # 결과 이미지에 사각형 그리기
            cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)

            # 사각형의 대각선 점 계산
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            # 선의 중심점
            line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

            # 중심점 표시
            cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)
            # 가중 평균에 추가
            centroid_sum += line_center_x * roi[-1]  # roi[-1]은 가중치

    # 목표 추종 실패 확인
    if centroid_sum == 0:
        self.lost_target_count += 1
        return result_image, None
    else:
        self.lost_target_count = 0
    
    # 가중 평균 중심 위치 계산
    center_pos = centroid_sum / self.weight_sum
    # 편차 각도 계산
    deflection_angle = -math.atan((center_pos - (w / 2.0)) / (h / 2.0))
    return result_image, deflection_angle
```

1. **초기화**: `centroid_sum` 초기화, 이미지 크기 저장
2. **색상 임계값 설정**: ColorPicker 사용 시 동적 계산, YAML 파일 사용 시 파일에서 읽기
3. **다중 ROI 처리**: 각 ROI에 대해 반복
    - ROI 영역 추출 → LAB 변환 → 가우시안 블러
    - 색상 마스킹 → 모폴로지 연산 (침식 → 팽창)
    - 컨투어 검출 → 최대 면적 컨투어 선택
    - 최소 외접 사각형 계산 → 중심점 계산 → 가중 평균에 추가
4. **결과 계산**: 가중 평균 중심 위치 계산 → 편차 각도 계산 → 반환

## 5. 이미지 처리 및 로봇 제어

### **이미지 콜백 함수**

```python
def image_callback(self, ros_image):
    cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
    rgb_image = np.array(cv_image, dtype=np.uint8)
    self.image_height, self.image_width = rgb_image.shape[:2]
    result_image = np.copy(rgb_image)
    
    with self.lock:
        # ColorPicker가 있으면 색상 선택 모드
        if self.color_picker is not None:
            try:
                target_color, result_image = self.color_picker(rgb_image, result_image)
                if target_color is not None:
                    self.color_picker = None
                    self.follower = LineFollower(target_color, self)
                    self.get_logger().info("target color: {}".format(target_color))
            except Exception as e:
                self.get_logger().error(str(e))
        else:
            # LineFollower가 있으면 선 추적 모드
            twist = Twist()
            twist.linear.x = 0.15  # 기본 선속도 설정
            
            if self.follower is not None:
                try:
                    result_image, deflection_angle = self.follower(rgb_image, result_image, self.threshold)

                    # 타켓이 없을 경우
                    if deflection_angle is None:
                        self.follower.lost_target_count += 1
                        if self.large_model_tracking and self.follower.lost_target_count > 5:
                            if not self.target_lost:
                                self.get_logger().warn("目标丢失，启动停止计时器...")
                                self.start_stop_timer()
                    else:
                        self.follower.lost_target_count = 0
                        if self.large_model_tracking and self.target_lost_timer is not None:
                            if self.target_lost_timer.is_alive():
                                self.target_lost_timer.cancel()
                                self.target_lost_timer = None
                                self.target_lost = False

                    # PID 제어 및 로봇 제어
                    if deflection_angle is not None and self.is_running and not self.stop:
                        self.pid.update(deflection_angle)
                        
                        # 로봇 타입에 따른 각속도 계산
                        if self.machine_type == 'MentorPi_Acker':
                            # 아커만 스티어링 모델
                            steering_angle = common.set_range(-self.pid.output, -math.radians(45), math.radians(45))
                            if steering_angle != 0:
                                R = 0.145 / math.tan(steering_angle)  # 회전 반경 계산
                                twist.angular.z = twist.linear.x / R  # 각속도 계산
                        else:
                            # 일반 차동 구동 모델
                            twist.angular.z = common.set_range(-self.pid.output, -1.0, 1.0)
                        
                        self.mecanum_pub.publish(twist)
                    elif self.stop:
                        self.mecanum_pub.publish(Twist())  # 정지
                    else:
                        self.pid.clear()  # PID 초기화
                except Exception as e:
                    self.get_logger().error(str(e))
    
    # 결과 이미지 발행 또는 디버그 모드
    if self.debug:
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(result_image)
    else:
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))
```

1. **이미지 변환**: ROS 이미지 → OpenCV RGB 이미지
2. **색상 선택 모드**: ColorPicker가 있으면 색상 수집 후 LineFollower 생성
3. **선 추적 모드**: LineFollower가 있으면
    - 선 검출 및 편차 각도 계산
    - 목표 추종 실패 확인 및 타이머 관리
    - PID 제어로 각속도 계산
    - 로봇 타입에 따른 각속도 변환 (아커만/차동 구동)
    - Twist 메시지 발행 또는 정지
4. **결과 발행**: 디버그 모드면 큐에 저장, 아니면 ROS 토픽으로 발행


## 6. 라이다 콜백 (장애물 감지)

### **라이다 콜백 함수**

```python
def lidar_callback(self, lidar_data):
    # 라이다 타입에 따른 데이터 처리
    if self.lidar_type != 'G4':
        min_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
        max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
        left_ranges = lidar_data.ranges[:max_index]  # 왼쪽 데이터
        right_ranges = lidar_data.ranges[::-1][:max_index]  # 오른쪽 데이터
    elif self.lidar_type == 'G4':
        min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
        max_index = int(math.radians(180) / lidar_data.angle_increment)
        left_ranges = lidar_data.ranges[min_index:max_index][::-1]
        right_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1]

    # 스캔 각도 범위 내 데이터 추출
    angle = self.scan_angle / 2
    angle_index = int(angle / lidar_data.angle_increment + 0.50)
    left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])

    # 유효한 거리 값 필터링
    left_nonzero = left_range.nonzero()
    right_nonzero = right_range.nonzero()
    left_nonan = np.isfinite(left_range[left_nonzero])
    right_nonan = np.isfinite(right_range[right_nonzero])
    
    # 좌우 최소 거리 계산
    min_dist_left_ = left_range[left_nonzero][left_nonan]
    min_dist_right_ = right_range[right_nonzero][right_nonan]
    
    if len(min_dist_left_) > 1 and len(min_dist_right_) > 1:
        min_dist_left = min_dist_left_.min()
        min_dist_right = min_dist_right_.min()
        
        # 장애물 감지 및 정지 플래그 설정
        if min_dist_left < self.stop_threshold or min_dist_right < self.stop_threshold:
            self.stop = True  # 정지 플래그 설정
        else:
            self.count += 1
            if self.count > 5:  # 5회 이상 안전하면 정지 해제 (노이즈 필터링)
                self.count = 0
                self.stop = False
```

1. **라이다 타입별 데이터 처리**: G4 타입과 일반 타입에 따라 좌우 데이터 추출 방식 다름
2. **스캔 각도 범위 추출**: 좌우 각 45도 범위 내 데이터만 추출
3. **유효 데이터 필터링**: 0이 아닌 값, 유한한 값만 필터링
4. **최소 거리 계산**: 좌우 각 방향에서 최소 거리 계산
5. **장애물 감지**: 최소 거리가 `stop_threshold` 미만이면 정지 플래그 설정
6. **노이즈 필터링**: 일정 횟수 이상 안전하면 정지 플래그 해제


## 코드 흐름도

### 전체 프로그램 흐름

```
[프로그램 시작]
    ↓
[ROS2 노드 초기화]
    ├─ 서비스 생성
    ├─ 발행자/구독자 초기화 (None)
    └─ 상태 변수 초기화
    ↓
[Enter 서비스 호출]
    ├─ 이미지 구독 시작
    ├─ 라이다 구독 시작
    └─ 초기화
    ↓
[색상 설정]
    ├─ [ColorPicker 모드]
    │   ├─ [이미지 클릭] → [색상 수집] → [LineFollower 생성]
    │
    └─ [YAML 색상 모드]
        └─ [색상 이름] → [LineFollower 생성]
    ↓
[이미지 처리 루프]
    ├─ [이미지 수신]
    ├─ [다중 ROI 선 검출]
    │   ├─ [ROI 추출]
    │   ├─ [LAB 변환]
    │   ├─ [가우시안 블러]
    │   ├─ [색상 마스킹]
    │   ├─ [모폴로지 연산]
    │   ├─ [컨투어 검출]
    │   └─ [최소 외접 사각형]
    ├─ [가중 평균 중심 계산]
    ├─ [편차 각도 계산]
    ├─ [PID 제어]
    ├─ [각속도 계산]
    └─ [로봇 제어 명령 발행]
    ↓
[라이다 처리 루프]
    ├─ [거리 데이터 수신]
    ├─ [좌우 각 45도 범위 추출]
    ├─ [최소 거리 계산]
    └─ [장애물 감지 시 정지]
```
