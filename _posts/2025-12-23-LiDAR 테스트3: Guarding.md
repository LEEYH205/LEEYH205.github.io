---
title: "LiDAR 테스트2: Guarding"
date: 2025-12-23 18:00:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, LiDAR, PID]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---
# Mode 3) LiDAR Guarding

---

## 동작 원리

- 라이다 센서로부터 받은 거리 데이터를 분석하여 가장 가까운 물체를 감지하고, 물체를 향해 회전하여 감시
- Mode 2와 달리 전후 이동 없이 회전만 수행하여 제자리에서 물체를 감시하는 것이 특징
    
    
    | 항목 | Mode 2 | Mode 3 |
    | --- | --- | --- |
    | 전후 이동 | 있음 (PID 제어) | 없음 |
    | 회전 | 있음 (PID 제어) | 있음 (PID 제어) |
    | 목표 거리 | `threshold / 2` | 없음 |
    | 동작 방식 | 물체를 따라가며 거리 유지 | 제자리에서 물체를 향해 회전 |
    | 사용 사례 | 물체 추적, 따라가기 | 감시, 모니터링 |

## 동작 확인
```python
# 새 터미널에서 라이다 기능 진입
ros2 service call /lidar_app/enter std_srvs/srv/Trigger {}

# Mode 3 시작
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 3}"

# 기능 종료
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 0}"

# 완전 종료
ros2 service call /lidar_app/exit std_srvs/srv/Trigger {}
```

![](/assets/img/HiWonderPi/gif/IMG_4431.gif)

## 코드 분석

```python
def lidar_callback(self, lidar_data):
    ...
    with self.lock:  # 멀티스레드 환경에서 안전한 데이터 접근을 위한 락
        ...
        if self.machine_type != 'MentorPi_Acker':
            ...
            elif self.running_mode == 3:  # 가드 모드
                # 거리 데이터 병합: 우측 반구를 역순으로 + 좌측 반구
                # 결과: 우측(역순) → 좌측 순서로 정렬된 배열 (반시계 방향)
                ranges = np.append(right_range[::-1], left_range)
                
                # 유효한 거리 데이터만 추출
                nonzero = ranges.nonzero()  # 0이 아닌 값들의 인덱스
                nonan = np.isfinite(ranges[nonzero])  # 유한한 값만 선택 (inf, nan 제외)
                dist_ = ranges[nonzero][nonan]  # 최종 유효 거리 배열
                
                if len(dist_) > 1:
                    dist = dist_.min()  # 가장 가까운 거리
                    min_index = list(ranges).index(dist)  # 최소 거리의 인덱스
                    # 최소 거리에 해당하는 각도 계산
                    angle = -angle + lidar_data.angle_increment * min_index
                    
                    # 각도 제어만 수행: 거리가 임계값 이내이고 각도가 5도 이상 벗어나면 회전
                    # 가드 모드는 전후 이동 없이 회전만 수행하여 제자리에서 감시
                    if dist < self.threshold and abs(math.degrees(angle)) > 5:
                        if self.lidar_type != 'G4':
                            # 일반 라이다: 각도 오차를 PID에 입력
                            self.pid_yaw.update(-angle)
                            # PID 출력을 각속도로 변환 (범위: -speed*6 ~ +speed*6)
                            twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
                        elif self.lidar_type == 'G4':
                            # G4 라이다: 각도 방향이 반대
                            self.pid_yaw.update(angle)
                            twist.angular.z = -common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
                    else:
                        # 각도가 충분히 작거나 거리가 임계값 밖이면 PID 초기화
                        self.pid_yaw.clear()
                    
                    # 작은 값 필터링: 노이즈로 인한 미세한 진동 방지
                    if abs(twist.angular.z) < 0.008:  # 0.008 rad/s 미만이면 0으로 설정
                        twist.angular.z = 0.0
                    
                    # 선속도는 항상 0 (가드 모드는 회전만 수행)
                    # twist.linear.x는 기본값 0으로 유지됨
                    
                    self.mecanum_pub.publish(twist)  # 최종 제어 명령 발행
```

- **동작 로직**
    1. **가장 가까운 물체 감지**: 추적 모드와 동일하게 좌우 데이터를 합쳐 가장 가까운 거리와 각도를 계산
    2. **회전 제어만 수행**: 전후 이동 없이 각도만 PID로 제어하여 물체를 향해 회전
    3. **임계값 내에서만 동작**: 거리가 임계값보다 작고 각도가 5도 이상일 때만 회전
    4. **작은 값 필터링**: 매우 작은 각속도는 0으로 설정하여 진동을 방지

### PID 제어기 활용

- 이 모드는 각도 제어만 PID로 수행합니다. 전후 이동 없이 회전만 제어하므로 `pid_yaw`만 사용하고 `pid_dist`는 사용하지 않습니다.
