---
title: "Camera 테스트4: Color Tracking + Visual Servoing"
date: 2025-12-24 10:20:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, Camera, Color Tracking, Visual Servoing]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

# Color Tracking + Visual Servoing

---

## 개요

> ROS2를 활용하여 카메라로 색상 객체를 실시간으로 추적하고, PID 제어를 통해 카메라 서보를 움직여 객체를 화면 중앙에 유지하는 프로젝트 입니다. `color_detect` 노드로부터 색상 객체의 위치 정보를 받아서, Visual Servoing 기법을 사용하여 카메라의 팬(pan) 및 틸트(tilt) 각도를 제어합니다.
> 

**주요 기능**

- **색상 객체 추적**: `color_detect` 노드로부터 색상 객체의 위치 정보 수신
- **Visual Servoing**: 이미지 좌표를 이용한 카메라 서보 제어
- **PID 제어**: 객체 위치 오차를 PID 제어로 카메라 각도 조정
- **이중 축 제어**: 수평(y축) 및 수직(z축) 방향으로 독립적인 PID 제어
- **서보 제어**: PWM 신호를 통한 카메라 서보 모터 제어

## 동작 확인

1. MentorPi를 시작하고 VNC Viewer에 연결합니다.
2. **Terminator**에서 아래 명령어를 입력해 ROS 2관련 자동 실행 서비스를 중지합니다.
    
    ```bash
    ~/.stop_ros.sh
    ```
    
3. 라인 추종 기능을 활성화하기 위해 명령어를 입력합니다.
    
    ```bash
    ros2 launch example color_track_node.launch.py
    ```
    
4. Tracking할 색상을 변경하려면 아래와 같이 명령어를 입력합니다.
    
    ```bash
    # 예시) 빨간색
    ros2 service call /color_track/set_color interfaces/srv/SetString "{data: 'red'}"
    ```
    
5. 실습을 종료하려면 터미널에서 'Ctrl+C'를 누릅니다.
6. **LXTerminal**에서 아래 명령어로 로봇 기능들을 다시 활성화하거나, 로봇을 재시작할 수 있습니다.
    
    ```bash
    sudo systemctl restart start_node.service
    ```

![](/assets/img/HiWonderPi/gif/Peek%202025-12-24%2013-20.gif)
![](/assets/img/HiWonderPi/gif/IMG_4439_1.gif)

## Launch 파일 구조

- `example/example/color_track/color_track_node.launch.py`
    
    ```
    **color_track_node.launch.py**
        ├─ **controller.launch.py** 포함
        │   └─ 로봇 제어기 및 서보 제어 노드 시작
        │
        ├─ **color_detect_node.launch.py** 포함
        │   ├─ depth_camera.launch.py 포함
        │   │   └─ 카메라 드라이버 노드 시작
        │   │   └─ 이미지 발행 노드 시작
        │   └─ color_detect 노드 시작
        │       ├─ roi.yaml 설정 파일 로드
        │       ├─ enable_display 파라미터 전달
        │       └─ enable_roi_display 파라미터 전달
        │
        └─ **color_track_node** 시작
            └─ start 파라미터 전달
    ```
    
    1. **Controller 노드**: 로봇 제어기 및 PWM 서보 제어
    2. **Camera 노드**: 카메라 드라이버 및 이미지 발행
    3. **Color Detect 노드**: 색상 검출 및 위치 정보 발행 (`/color_detect/color_info` 토픽)
    4. **Color Track 노드**: 색상 추적 및 Visual Servoing 제어

## 동작 원리

1. **색상 정보 수신**: `color_detect` 노드로부터 색상 객체의 위치 정보(`ColorsInfo`)를 구독
2. **위치 정보 추출**: 수신된 메시지에서 객체의 중심 좌표(x, y), 반지름, 이미지 크기 등을 추출
3. **오차 계산**: 객체의 현재 위치와 이미지 중심의 차이를 계산
    - 수평 방향 오차: `width/2 - x`
    - 수직 방향 오차: `height/2 - y`
4. **PID 제어**: 각 방향의 오차를 PID 제어기로 처리하여 서보 각도 조정량을 계산
5. **서보 제어**: 계산된 각도를 PWM 신호로 변환하여 카메라 서보를 제어
6. **피드백 루프**: 새로운 이미지에서 객체 위치를 다시 확인하고 반복

## 프로그램 구조

### Launch 파일 구조

```
[Launch 파일 실행]
    ↓
[Controller Launch]
    ㄴ [로봇제어기 노드]
    ㄴ [PWM 서보 제어 노드]
    ↓
[Color Detect Launch]
    ├─ [Depth Camera Launch]
    │   ㄴ [카메라 드라이버 노드]
    │   ㄴ [이미지 발행 노드]
    └─ [Color Detect 노드]
        ㄴ [색상 검출 및 위치 정보 발행]
    ↓
[Color Track 노드]
    ├─ [색상 정보 구독]
    ├─ [PID 제어]
    └─ [서보 제어]
```

### Color Track 노드 구조

```
[프로그램 시작]
    ↓
[ROS2 노드 초기화]
    ├─ 서비스 생성
    ├─ 토픽 구독 설정
    ├─ 클라이언트 생성
    └─ PID 제어기 초기화
    ↓
[초기화 프로세스]
    ├─ 카메라 서보 초기 위치 설정
    └─ 자동 시작 (선택사항)
    ↓
[색상 정보 수신] (color_detect 노드로부터)
    ↓
[메인 제어 루프]
    ├─ [위치 정보 확인]
    ├─ [오차 계산]
    ├─ [PID 제어]
    ├─ [서보 각도 업데이트]
    └─ [PWM 신호 발행]
```
# 주요 사용 기술

---

## Visual Servoing
![](/assets/img/HiWonderPi/gif/download/eye2hand.gif)

### **Visual Servoing이란?**

> 카메라로부터 얻은 이미지 정보를 직접 사용하여 로봇을 제어하는 기법입니다. 객체의 이미지 좌표를 이용하여 로봇의 관절이나 카메라 자체를 제어합니다.
> 
- **Image-based Visual Servoing**
    - 객체의 이미지 좌표를 직접 사용
    - 목표: 객체를 이미지 중앙에 위치시키기
    - 장점: 간단하고 빠른 응답
    - 단점: 깊이 정보가 없어 정확한 위치 제어는 어려움
- **Position-based Visual Servoing**
    - 3D 좌표를 추정하여 사용
    - 목표: 객체를 특정 3D 위치에 위치시키기
    - 장점: 정확한 위치 제어 가능
    - 단점: 복잡하고 계산 비용이 큼
- **Color Tracking에서의 사용**
    - Image-based Visual Servoing 방식 사용
    - 객체의 이미지 좌표(x, y)를 이용하여 카메라 각도 제어
    - 목표: 객체를 이미지 중앙(width/2, height/2)에 위치시키기

### 이중 축 PID 제어

- 각 축에 대해 독립적인 PID 제어기를 사용하여 동시에 제어
    - **팬(Pan) 축**: 수평 방향 회전 (좌우)
    - **틸트(Tilt) 축**: 수직 방향 회전 (상하)

**수평 방향 제어 (Y축)**

```python
self.pid_y.SetPoint = self.center.width / 2  # 목표: 이미지 중앙 x 좌표
self.pid_y.update(self.center.width - self.center.x)  # 오차: 중앙 - 현재 x
self.y_dis -= self.pid_y.output  # 서보 각도 업데이트
```

**수직 방향 제어 (Z축)**

```python
self.pid_z.SetPoint = self.center.height / 2  # 목표: 이미지 중앙 y 좌표
self.pid_z.update(self.center.y)  # 오차: 현재 y - 중앙
self.z_dis -= self.pid_z.output  # 서보 각도 업데이트
```


### ColorsInfo 메시지 구조

- **ColorsInfo 메시지**
    
    ```
    interfaces/ColorInfo[] data
    	string color      # 색상 이름 (예: 'red', 'green', 'blue')
    	int32 width       # 이미지 너비
    	int32 height      # 이미지 높이
    	int32 x           # 객체 중심 x 좌표
    	int32 y           # 객체 중심 y 좌표
    	int32 radius      # 객체 반지름 (원형 검출 시)
    	int32 angle       # 객체 각도 (사각형 검출 시)
    ```
    
    - `color_detect` 노드가 색상 객체를 검출하면 이 메시지를 발행
    - `color_track` 노드는 이 메시지를 구독하여 객체 위치 정보 획득
    - `radius > 10` 조건으로 작은 노이즈 필터링


# 코드 분석

---

## 1. 프로그램 시작 및 초기화

### **노드 초기화**

```python
class ColorTrackNode(Node):
    def __init__(self, name):
        rclpy.init()  # ROS2 초기화
        super().__init__(name, allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        
        # 초기 서보 위치 설정
        self.z_dis = 0.36  # z축 초기값 (틸트)
        self.y_dis = 500   # y축 초기값 (팬)
        self.center = None  # 현재 추적 중인 객체 위치 정보
        self.running = True  # 프로그램 실행 플래그
        self.start = False   # 추적 시작 플래그
        
        # PID 제어기 초기화 (수직, 수평)
        self.pid_z = pid.PID(0.1, 0.0002, 0.0001)  # z축 PID
        self.pid_y = pid.PID(0.1, 0.0002, 0.0001)  # y축 PID
        
        # 시그널 핸들러 설정 (Ctrl+C 처리)
        signal.signal(signal.SIGINT, self.shutdown)
        
        # 발행자 생성
        self.pwm_pub = self.create_publisher(SetPWMServoState,
                                              'ros_robot_controller/pwm_servo/set_state', 10)
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        
        # 색상 정보 구독
        self.create_subscription(ColorsInfo, '/color_detect/color_info',
                                  self.get_color_callback, 1)
        
        # 서비스 및 클라이언트 생성
        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback,
                            callback_group=timer_cb_group)
        self.create_service(SetString, '~/set_color', self.set_color_srv_callback,
                            callback_group=timer_cb_group)
        
        # color_detect 노드의 색상 설정 서비스 클라이언트
        self.set_color_client = self.create_client(SetColorDetectParam,
                                                    '/color_detect/set_param',
                                                    callback_group=timer_cb_group)
        self.set_color_client.wait_for_service()  # 서비스 대기
        
        # 초기화 타이머 (한 번만 실행)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
```

- **초기화 과정**
    1. ROS2 초기화 및 노드 생성
    2. 상태 변수 초기화 (서보 위치, 실행 플래그 등)
    3. PID 제어기 초기화 (z축, y축)
    4. 발행자 생성 (PWM 서보 제어, 로봇 제어)
    5. 색상 정보 토픽 구독 설정
    6. 서비스 생성 (start, stop, set_color)
    7. color_detect 노드의 서비스 클라이언트 생성 및 대기
    8. 초기화 프로세스 타이머 시작

### **초기화 프로세스**

```python
def init_process(self):
    self.timer.cancel()  # 타이머 취소 (한 번만 실행)
    self.init_action()  # 초기 동작 수행
    
    # 파라미터에 따라 자동 시작
    if self.get_parameter('start').value:
        self.start_srv_callback(Trigger.Request(), Trigger.Response())
        request = SetString.Request()
        request.data = 'red'  # 기본 색상: 빨강
        self.set_color_srv_callback(request, SetString.Response())
    
    # 메인 제어 루프를 별도 스레드에서 시작
    threading.Thread(target=self.main, daemon=True).start()
    self.create_service(Trigger, '~/init_finish', self.get_node_state)
    self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
```


### **초기 동작**

```python
def init_action(self):
    self.pwm_controller([1500, 1500])  # 서보 중립 위치 설정
    self.mecanum_pub.publish(Twist())  # 로봇 정지 명령
```

## 2. 색상 설정

### **색상 설정 서비스**

```python
def set_color_srv_callback(self, request, response):
    self.get_logger().info('\033[1;32m%s\033[0m' % "set_color")
    # color_detect 노드에 색상 설정 요청
    msg = SetColorDetectParam.Request()
    msg_red = ColorDetect()
    msg_red.color_name = request.data  # 색상 이름 (예: 'red', 'green', 'blue')
    msg_red.detect_type = 'circle'  # 원형 검출 타입
    msg.data = [msg_red]
    
    # 서비스 호출 (동기 방식)
    res = self.send_request(self.set_color_client, msg)
    if res.success:
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start_track_%s' % msg_red.color_name)
    else:
        self.get_logger().info('\033[1;32m%s\033[0m' % 'track_fail')
    
    response.success = True
    response.message = "set_color"
    return response
```
### **서비스 요청 함수**

```python
def send_request(self, client, msg):
    future = client.call_async(msg)  # 비동기 호출
    while rclpy.ok():
        if future.done() and future.result():
            return future.result()  # 결과 반환
```

1. **색상 이름 수신**: 서비스 요청에서 색상 이름 추출
2. **메시지 생성**: ColorDetect 메시지 생성 (색상 이름, 검출 타입)
3. **서비스 호출**: color_detect 노드의 set_param 서비스 호출
4. **결과 확인**: 성공/실패 로그 출력

## 3. 색상 정보 수신

### **색상 정보 콜백 함수**

```python
def get_color_callback(self, msg):
    if msg.data != []:  # 색상 정보가 있으면
        if msg.data[0].radius > 10:  # 반지름이 10보다 크면 (노이즈 필터링)
            self.center = msg.data[0]  # 객체 위치 정보 저장
        else:
            self.center = None  # 너무 작으면 무시
    else:
        self.center = None  # 색상 정보가 없으면 None
```

1. **메시지 확인**: ColorsInfo 메시지의 data 배열이 비어있지 않은지 확인
2. **크기 필터링**: 반지름이 10보다 큰 객체만 유효한 것으로 간주
3. **위치 정보 저장**: 유효한 객체의 위치 정보를 `self.center`에 저장


## 4. 메인 제어 루프

### **메인 제어 루프**

```python
def main(self):
    while self.running:
        if self.center is not None and self.start:  # 객체가 있고 추적이 시작되었으면
            t1 = time.time()  # 시작 시간 기록
            
            # 수평 방향 제어 (Y축 - 팬)
            self.pid_y.SetPoint = self.center.width / 2  # 목표: 이미지 중앙 x 좌표
            self.pid_y.update(self.center.width - self.center.x)  # 오차 업데이트
            self.y_dis -= self.pid_y.output  # 서보 각도 업데이트
            # 범위 제한
            if self.y_dis < 800:
                self.y_dis = 800
            if self.y_dis > 1900:
                self.y_dis = 1900
            
            # 수직 방향 제어 (Z축 - 틸트)
            self.pid_z.SetPoint = self.center.height / 2  # 목표: 이미지 중앙 y 좌표
            self.pid_z.update(self.center.y)  # 오차 업데이트
            self.z_dis -= self.pid_z.output  # 서보 각도 업데이트
            # 범위 제한
            if self.z_dis > 1900:
                self.z_dis = 1900
            if self.z_dis < 800:
                self.z_dis = 800
            
            # 서보 제어 명령 발행
            self.pwm_controller([self.z_dis, self.y_dis])
            
            # 루프 시간 제어 (약 50Hz)
            t2 = time.time()
            t = t2 - t1
            if t < 0.02:  # 0.02초(50Hz)보다 빠르면 대기
                time.sleep(0.02 - t)
        else:
            time.sleep(0.01)  # 객체가 없거나 추적이 중지되었으면 대기
    
    # 종료 시 초기 동작 수행
    self.init_action()
    rclpy.shutdown()
```

1. **조건 확인**: 객체 위치 정보가 있고 추적이 시작되었는지 확인
2. **수평 방향 제어**:
    - 목표 위치 설정 (이미지 중앙 x 좌표)
    - 오차 계산 및 PID 업데이트
    - 서보 각도 업데이트 및 범위 제한
3. **수직 방향 제어**
    - 목표 위치 설정 (이미지 중앙 y 좌표)
    - 오차 계산 및 PID 업데이트
    - 서보 각도 업데이트 및 범위 제한
4. **서보 제어**: PWM 신호 발행
5. **루프 시간 제어**: 약 50Hz 주기 유지

## 5. 서보 제어 함수

### **PWM 서보 제어 함수**

```python
def pwm_controller(self, position_data):
    pwm_list = []
    msg = SetPWMServoState()
    msg.duration = 0.2  # 서보 이동 시간 (초)
    
    for i in range(len(position_data)):
        pos = PWMServoState()
        pos.id = [i + 1]  # 서보 ID (1번: z축, 2번: y축)
        pos.position = [int(position_data[i])]  # 서보 위치 (PWM 값)
        pwm_list.append(pos)
    
    msg.state = pwm_list
    self.pwm_pub.publish(msg)  # PWM 명령 발행
```

1. **메시지 생성**: SetPWMServoState 메시지 생성
2. **서보 설정**: 각 서보에 대해 ID와 위치 설정
3. **명령 발행**: PWM 제어 명령을 토픽으로 발행

## 6. Start/Stop 서비스

### **Start 서비스**

```python
def start_srv_callback(self, request, response):
    self.get_logger().info('\033[1;32m%s\033[0m' % "start color track")
    self.start = True  # 추적 시작 플래그 설정
    response.success = True
    response.message = "start"
    return response
```

### **Stop 서비스**

```python
def stop_srv_callback(self, request, response):
    self.get_logger().info('\033[1;32m%s\033[0m' % "stop color track")
    self.start = False  # 추적 중지 플래그 설정
    # color_detect 노드에 빈 요청 전송 (색상 설정 해제)
    res = self.send_request(ColorDetect.Request())
    if res.success:
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set color success')
    else:
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set color fail')
    response.success = True
    response.message = "stop"
    return response
```

## 코드 흐름도

### Launch 파일 실행 흐름

```
[Launch 파일 실행]
    ↓
[환경 변수 확인] (need_compile)
    ↓
[Launch 인자 선언]
    ├─ start (기본값: true)
    └─ enable_display (기본값: true)
    ↓
[Controller Launch 포함]
    ├─ [로봇 제어기 노드 시작]
    └─ [PWM 서보 제어 노드 시작]
    ↓
[Color Detect Launch 포함]
    ├─ [Depth Camera Launch 포함]
    │   ├─ [카메라 드라이버 노드 시작]
    │   └─ [이미지 발행 노드 시작]
    └─ [Color Detect 노드 시작]
        └─ [색상 검출 및 위치 정보 발행]
    ↓
[Color Track 노드 시작]
    └─ [start 파라미터 전달]
```
## Color Track 노드 실행 흐름

```
[Color Track 노드 시작]
    ↓
[ROS2 노드 초기화]
    ├─ PID 제어기 초기화
    ├─ 발행자/구독자 생성
    ├─ 서비스/클라이언트 생성
    └─ 초기화 타이머 시작
    ↓
[초기화 프로세스]
    ├─ 서보 중립 위치 설정 (1500, 1500)
    └─ 자동 시작 (start=true인 경우)
        ├─ start 서비스 자동 호출
        └─ 색상 'red'로 자동 설정
    ↓
[색상 설정] (set_color 서비스 또는 자동)
    ├─ color_detect 노드에 색상 설정 요청
    └─ 검출 타입: 'circle'
    ↓
[추적 시작] (start 서비스)
    ↓
[메인 제어 루프]
    ├─ [색상 정보 수신] (get_color_callback)
    │   ├─ 메시지 확인
    │   ├─ 크기 필터링 (radius > 10)
    │   └─ 위치 정보 저장
    ├─ [수평 방향 제어]
    │   ├─ 목표 위치 설정 (width/2)
    │   ├─ 오차 계산 (width - x)
    │   ├─ PID 제어
    │   └─ 서보 각도 업데이트
    ├─ [수직 방향 제어]
    │   ├─ 목표 위치 설정 (height/2)
    │   ├─ 오차 계산 (y)
    │   ├─ PID 제어
    │   └─ 서보 각도 업데이트
    ├─ [범위 제한] (800 ~ 1900)
    ├─ [PWM 명령 발행]
    └─ [루프 시간 제어] (50Hz)
```
