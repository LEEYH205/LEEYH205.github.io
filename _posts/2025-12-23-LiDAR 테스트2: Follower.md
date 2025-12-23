---
title: "LiDAR 테스트2: Follower"
date: 2025-12-23 18:00:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, LiDAR, PID]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

# **Mode 2)** LiDAR 기반 Folllower

---

## 동작 원리

- 라이다 추적 모드는 라이다 센서로부터 받은 거리 데이터를 분석하여 가장 가까운 물체를 찾고, PID 제어를 통해 물체를 향해 이동하며 일정 거리를 유지
- 각도 제어와 거리 제어를 동시에 수행하여 부드럽고 정확한 추적을 구현

## 동작 확인

```python
# 새 터미널에서 라이다 기능 진입
ros2 service call /lidar_app/enter std_srvs/srv/Trigger {}

# Mode 2 시작
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 2}"

# 기능 종료
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 0}"

# 완전 종료
ros2 service call /lidar_app/exit std_srvs/srv/Trigger {}
```

![](/assets/img/HiWonderPi/gif/IMG_4430.gif)

## 코드 분석

### 일반 로봇 타입

```python
def lidar_callback(self, lidar_data):
    ...
    with self.lock:  # 멀티스레드 환경에서 안전한 데이터 접근을 위한 락
        ...
        if self.machine_type != 'MentorPi_Acker':
            ...
            elif self.running_mode == 2:  # 추적 모드
                # 거리 데이터 병합: 우측 반구를 역순으로 + 좌측 반구
                # 결과: 우측(역순) → 좌측 순서로 정렬된 배열 (반시계 방향)
                ranges = np.append(right_range[::-1], left_range)
                self.get_logger().info(str(ranges))  # 디버깅용 로그
                
                # 유효한 거리 데이터만 추출
                nonzero = ranges.nonzero()  # 0이 아닌 값들의 인덱스
                nonan = np.isfinite(ranges[nonzero])  # 유한한 값만 선택 (inf, nan 제외)
                dist_ = ranges[nonzero][nonan]  # 최종 유효 거리 배열
                
                if len(dist_) > 0:
                    dist = dist_.min()  # 가장 가까운 거리
                    min_index = list(ranges).index(dist)  # 최소 거리의 인덱스
                    # 최소 거리에 해당하는 각도 계산
                    # -angle: 초기 각도, angle_increment * min_index: 인덱스에 따른 각도 변화
                    angle = -angle + lidar_data.angle_increment * min_index
                    
                    # 각도 제어: 물체가 5도 이상 벗어나면 PID로 각도 조정
                    if dist < self.threshold and abs(math.degrees(angle)) > 5:
                        if self.lidar_type != 'G4':
                            # 일반 라이다: 각도가 양수면 우측, 음수면 좌측
                            self.pid_yaw.update(-angle)  # 각도 오차를 PID에 입력
                            # PID 출력을 각속도로 변환 (범위: -speed*6 ~ +speed*6)
                            twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
                        elif self.lidar_type == 'G4':
                            # G4 라이다: 각도 방향이 반대
                            self.pid_yaw.update(angle)
                            twist.angular.z = -common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
                    else:
                        self.pid_yaw.clear()  # 각도가 충분히 작으면 PID 초기화

                    # 거리 제어: 목표 거리(threshold/2)와 현재 거리의 차이를 PID로 제어
                    if dist < self.threshold and abs(0.2 - dist) > 0.02:  # 0.02m 이상 차이날 때만 제어
                        # 목표 거리: threshold / 2, 현재 거리: dist
                        # 오차 = 목표 - 현재 (양수면 접근, 음수면 후퇴)
                        self.pid_dist.update(self.threshold / 2 - dist)
                        # PID 출력을 선속도로 변환 (범위: -speed ~ +speed)
                        twist.linear.x = common.set_range(self.pid_dist.output, -self.speed, self.speed)
                    else:
                        self.pid_dist.clear()  # 거리가 목표에 가까우면 PID 초기화
                    
                    # 작은 값 필터링: 노이즈로 인한 미세한 진동 방지
                    if abs(twist.angular.z) < 0.008:  # 0.008 rad/s 미만이면 0으로 설정
                        twist.angular.z = 0.0
                    if abs(twist.linear.x) < 0.05:  # 0.05 m/s 미만이면 0으로 설정
                        twist.linear.x = 0.0
                    
                    self.mecanum_pub.publish(twist)  # 최종 제어 명령 발행
```

- **동작 로직**
    1. **가장 가까운 물체 찾기**: 좌우 데이터를 합쳐 가장 가까운 거리와 각도를 계산
    2. **각도 제어 (PID)**: 물체가 5도 이상 벗어나면 PID로 각도를 조정
    3. **거리 제어 (PID)**: 목표 거리(`threshold / 2`)를 유지하도록 PID로 전후 이동을 제어
    4. **작은 값 필터링**: 매우 작은 각속도나 선속도는 0으로 설정하여 진동을 방지

### MentorPi A1 타입 로봇

```python
def lidar_callback(self, lidar_data):
    ...
    with self.lock:  # 멀티스레드 환경에서 안전한 데이터 접근을 위한 락
        ...
        else:  # MentorPi_Acker 타입 (아커만 스티어링 방식)
            ...
            elif self.running_mode == 2:  # 추적 모드
                # 거리 데이터 병합 (일반 타입과 동일)
                ranges = np.append(right_range[::-1], left_range)
                nonzero = ranges.nonzero()
                nonan = np.isfinite(ranges[nonzero])
                dist_ = ranges[nonzero][nonan]
                
                if len(dist_) > 1:
                    dist = dist_.min()
                    min_index = list(ranges).index(dist)
                    angle = -angle + lidar_data.angle_increment * min_index
                    
                    # 각도 데이터 필터링: 노이즈 제거를 위한 통계적 필터링
                    self.angle_data.append(angle)  # 최근 각도 데이터 저장
                    data = pd.DataFrame(self.angle_data)  # DataFrame으로 변환
                    data_ = data.copy()
                    u = data_.mean()  # 평균 계산
                    std = data_.std()  # 표준편차 계산
                    
                    # 평균 ± 표준편차 범위 내의 데이터만 선택 (이상치 제거)
                    data_c = data[np.abs(data - u) <= std]
                    data_c = data_c.dropna(axis=0, how='any')  # NaN 제거
                    # 최신 각도 데이터 사용 (필터링된 데이터가 아닌 원본의 마지막 값)
                    angle = data.tail(1).iloc[0, 0]
                    # 최근 15개 데이터만 유지 (슬라이딩 윈도우)
                    if len(self.angle_data) == 15:
                        self.angle_data.remove(self.angle_data[0])
                    
                    # 거리 제어: 목표 거리 0.5m 유지
                    if dist < 2 and abs(0.5 - dist) > 0.02:  # 2m 이내이고 0.02m 이상 차이날 때
                        self.pid_dist.SetPoint = 0.5  # 목표 거리 설정
                        self.pid_dist.update(dist)  # 현재 거리를 PID에 입력
                        # PID 출력을 선속도로 변환 (음수 부호: 거리 감소 시 후퇴)
                        twist.linear.x = common.set_range(-self.pid_dist.output, -0.35, 0.35)
                    else:
                        self.pid_dist.clear()
                    
                    # 각도 제어: 아커만 스티어링 기하학 적용
                    if dist < 2 and abs(math.degrees(angle)) > 5:
                        if self.lidar_type != 'G4':
                            self.pid_yaw.update(-angle)
                            # 아커만 스티어링: 각속도 = 선속도 * tan(스티어링각) / 휠베이스
                            # 스티어링각 범위: -0.316 ~ +0.316 rad (약 ±18도)
                            # 휠베이스: 0.145m (앞뒤 바퀴 사이 거리)
                            twist.angular.z = twist.linear.x * math.tan(common.set_range(self.pid_yaw.output, -0.316, 0.316)) / 0.145
                        elif self.lidar_type == 'G4':
                            self.pid_yaw.update(angle)
                            # G4 라이다는 각도 방향이 반대
                            twist.angular.z = -twist.linear.x * math.tan(common.set_range(self.pid_yaw.output, -0.316, 0.316)) / 0.145
                    else:
                        self.pid_yaw.clear()
                    
                    self.mecanum_pub.publish(twist)
```

- **A1 타입용 로직 특징**
    1. **각도 데이터 필터링**: 최근 15개의 각도 데이터를 저장하고, 평균과 표준편차를 계산하여 노이즈를 필터링
    2. **아커만 스티어링 기하학**: 아커만 스티어링의 기하학적 특성을 고려하여 각속도를 계산
        - `twist.angular.z = twist.linear.x * tan(steering_angle) / wheelbase`
        - 여기서 `wheelbase`는 0.145m로 설정되어 있음
    3. **목표 거리**: 0.5m를 목표 거리로 설정


> 라이다 추적 모드는 PID 제어를 핵심으로 사용합니다. 각도 제어와 거리 제어를 동시에 수행하여 부드럽고 정확한 추적을 구현합니다.
> 

## PID 제어 개요

![](/assets/img/HiWonderPi/gif/download/PID_Compensation_Animated.gif)

### **PID 제어란?**

PID 제어는 Proportional(비례), Integral(적분), Derivative(미분)의 세 가지 요소를 조합한 제어 방식입니다. 목표값과 현재값의 차이(오차)를 기반으로 제어 출력을 계산하여 시스템을 원하는 상태로 유도합니다.

### **PID 제어의 구성 요소**

1. **`P` (Proportional) - 비례 제어**
    - 현재 오차에 비례하여 출력을 생성
    - 오차가 크면 큰 제어 출력, 오차가 작으면 작은 제어 출력
    - 빠른 응답을 제공하지만, 목표값에 도달하지 못하는 정상 상태 오차(Steady-state error)가 발생할 수 있음
2. **`I` (Integral) - 적분 제어**
    - 과거 오차의 누적(적분)을 고려
    - 정상 상태 오차를 제거하는 역할
    - 오차가 지속되면 제어 출력이 점진적으로 증가하여 오차를 줄임
    - 너무 크면 과도한 응답(Overshoot)이나 진동이 발생할 수 있음
3. **`D` (Derivative) - 미분 제어**
    - 오차의 변화율(미분)을 고려
    - 시스템의 응답 속도를 예측하여 진동을 억제
    - 빠른 변화에 대해 제동 효과를 제공하여 안정성을 향상시킴
    - 노이즈에 민감할 수 있음

https://www.luisllamas.es/en/pid-controller-simulator/

### **PID 제어 수식**

$$
u(t) = K_p \times e(t) + K_i \int e(t) \, dt + K_d \frac{de(t)}{dt}
$$

- $u(t)$: 제어 출력
- $e(t)$: 오차 (목표값 - 현재값)
- $K_p$: 비례 게인
- $K_i$: 적분 게인
- $K_d$: 미분 게인

### **PID 제어의 장점**

- **부드러운 제어**: 급격한 변화 없이 목표값에 접근
- **정확성**: 정상 상태 오차를 최소화
- **안정성**: 진동을 억제하여 안정적인 제어
- **적응성**: 다양한 시스템에 적용 가능

### LiDAR 기반 Folllower**서의 PID 활용**

- **각도 제어 (`pid_yaw`)**: 물체의 각도 오차를 PID로 계산하여 부드럽게 회전
- **거리 제어 (`pid_dist`)**: 목표 거리와 현재 거리의 차이를 PID로 계산하여 부드럽게 접근/후퇴

## MentorPi 내 관련 SDK

- `lidar_controller.py`는 PID 제어를 위해 `driver/sdk/sdk/pid.py`의 `PID` 클래스를 사용
    
    ```python
    class PID:
        """PID Controller
        """
        def __init__(self, P=0.2, I=0.0, D=0.0):
    
            self.Kp = P
            self.Ki = I
            self.Kd = D
    
            self.sample_time = 0.00
            self.current_time = time.time()
            self.last_time = self.current_time
    
            self.clear()
    
        def clear(self):
            """Clears PID computations and coefficients"""
            self.SetPoint = 0.0
    
            self.PTerm = 0.0
            self.ITerm = 0.0
            self.DTerm = 0.0
            self.last_error = 0.0
    
            # Windup Guard
            self.int_error = 0.0
            self.windup_guard = 20.0
    
            self.output = 0.0
    ```
    
- **PID 업데이트 메서드**
    
    ```python
    def update(self, feedback_value):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = self.SetPoint - feedback_value
    
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
    
        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time
    
            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard
    
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
    
            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error
    
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
    ```
    
- **PID 제어기 초기화**
    
    ```python
    # PID 파라미터
    self.pid_yaw = pid.PID(1.6, 0, 0.16)
    self.pid_dist = pid.PID(1.7, 0, 0.16)
    ```
    
    - **pid_yaw**: 각도 제어용 (Kp=1.6, Ki=0, Kd=0.16)
    - **pid_dist**: 거리 제어용 (Kp=1.7, Ki=0, Kd=0.16)
- **PID 사용 예시**
    - **각도 제어**
        
        ```python
        # 목표 각도 설정 (0도 = 정면)
        self.pid_yaw.SetPoint = 0.0
        
        # 현재 각도 오차를 PID에 입력
        self.pid_yaw.update(-angle)
        
        # PID 출력을 각속도로 변환 (범위 제한)
        twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
        ```
        
    - **거리 제어**
        
        ```python
        # 목표 거리와 현재 거리의 차이를 PID에 입력
        # 목표 거리는 threshold / 2로 설정
        self.pid_dist.update(self.threshold / 2 - dist)
        
        # PID 출력을 선속도로 변환 (범위 제한)
        twist.linear.x = common.set_range(self.pid_dist.output, -self.speed, self.speed)
        ```