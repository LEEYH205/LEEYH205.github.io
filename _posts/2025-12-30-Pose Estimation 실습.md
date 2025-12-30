---
title: "Pose Estimation 실습"
date: 2025-12-30 15:10:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, Ultralytics, YOLO, Pose Estimation]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---
# Pose Estimation

---

- **Keypoint**는 관절, 랜드마크 또는 기타 식별 가능한 특징과 같은 객체의 다양한 부분을 나타낼 수 있으며, 키포인트의 위치는 일반적으로 2D 또는 3D 좌표 집합으로 표현됨
    - 이미지 내 특정 지점의 위치를 식별하는 작업으로, 일반적으로 키포인트라고 함
- 포즈 추정 모델의 출력은 이미지 내 객체의 키포인트를 나타내는 점들의 집합이며, 일반적으로 각 점에 대한 신뢰도 점수와 함께 제공
- 장면에서 객체의 특정 부분을 식별하고 서로에 대한 위치를 파악해야 할 때 좋음

## 사용 가능 모델

| **모델** | **크기(픽셀)** | **mAP 50-95** | **mAP 50** | **속도 CPU ONNX(ms)** | **속도 T4 TensorRT10(ms)** | **파라미터(M)** | **FLOPs(B)** |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [YOLO11n-pose](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.pt) | 640 | 50.0 | 81.0 | 52.4 ± 0.5 | 1.7 ± 0.0 | 2.9 | 7.4 |
| [YOLO11s-pose](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s-pose.pt) | 640 | 58.9 | 86.3 | 90.5 ± 0.6 | 2.6 ± 0.0 | 9.9 | 23.1 |
| [YOLO11m-pose](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11m-pose.pt) | 640 | 64.9 | 89.4 | 187.3 ± 0.8 | 4.9 ± 0.1 | 20.9 | 71.4 |
| [YOLO11l-pose](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11l-pose.pt) | 640 | 66.1 | 89.9 | 247.7 ± 1.1 | 6.4 ± 0.1 | 26.1 | 90.3 |
| [YOLO11x-pose](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x-pose.pt) | 640 | 69.5 | 91.1 | 488.0 ± 13.9 | 12.1 ± 0.2 | 58.8 | 202.8 |
- 사전 학습에 사용된 데이터셋: [COCO-pose](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco-pose.yaml)
![](/assets/img/HiWonderPi/img/download/COCO-pose.png)

## YOLO11n 모델 기반의 Pose Estimation 실습

1. `/home/ubuntu/ros2_ws/src/yolo_ros/yolo_bringup/launch/yolo.launch.py`를 열고 아래와 같이 수정해줍니다.
    
    ```bash
    ...
    
    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument(
        "model",
        **default_value="yolo11n-pose.pt", # 수정**
        description="Model name or path",
    )
    
    ...
    
    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        **default_value="cpu",**
        description="Device to use (GPU/CPU)",
    )
    
    ...
    
    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        **default_value="/ascamera/camera_publisher/rgb0/image",**
        description="Name of the input image topic",
    )
    
    image_reliability = LaunchConfiguration("image_reliability")
    image_reliability_cmd = DeclareLaunchArgument(
        "image_reliability",
        **default_value="2",**
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)",
    )
    ```

2. 아래 명령어로 런치파일을 실행합니다.
    
    ```bash
    ros2 launch yolo_bringup yolo.launch.py
    ```
    
3. 아래 명령어로 YOLO 관련 토픽들을 확인할 수 있습니다.
    
    ```bash
    ros2 topic list
    ```

![](/assets/img/HiWonderPi/gif/Peek%202025-12-30%2015-14.gif)

MediaPipe꺼를 사용하는게 낫지 않을까..

## 활용 프로젝트

### **Fitness/운동 자세 평가 로봇**

- 스쿼트, 런지, 팔굽혀펴기 등 각도 기반 자세 평가
- 특정 관절의 angle trajectory → ROS 2로 퍼블리시
- 스마트 헬스 또는 홈 트레이닝 로봇 프로젝트로 확장

### **산업 안전 모니터링 로봇 (작업자 자세 분석)**

- 특정 위험 동작 탐지
    - 몸을 과하게 숙임
    - 기계 쪽으로 손/팔이 특정 각도 이상 접근

[Ultralytics YOLO11을 사용한 분석](https://docs.ultralytics.com/ko/guides/analytics/)