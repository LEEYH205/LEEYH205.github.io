---
title: "Segmentation 실습"
date: 2025-12-30 15:00:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, Ultralytics, YOLO, Segmentation]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---
# Segmentation

---

- [Segmentation](https://www.ultralytics.com/glossary/instance-segmentation)는 객체 감지보다 한 단계 더 나아가 이미지에서 개별 객체를 식별하고 이미지의 나머지 부분에서 분리하는 것을 포함함
- 모델의 출력은 이미지의 각 객체 윤곽을 나타내는 마스크 또는 윤곽선 세트와 각 객체에 대한 클래스 레이블 및 신뢰도 점수
- 이미지에서 객체의 위치뿐만 아니라 정확한 모양도 알아야 할 때 유용함

## 사용 가능 모델

| **모델** | **크기(픽셀)** | **mAP 50-95** | **mAP 50-95** | **속도 CPU ONNX(ms)** | **속도 T4 TensorRT10(ms)** | **파라미터(M)** | **FLOPs(B)** |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [YOLO11n-seg](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-seg.pt) | 640 | 38.9 | 32.0 | 65.9 ± 1.1 | 1.8 ± 0.0 | 2.9 | 9.7 |
| [YOLO11s-seg](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s-seg.pt) | 640 | 46.6 | 37.8 | 117.6 ± 4.9 | 2.9 ± 0.0 | 10.1 | 33.0 |
| [YOLO11m-seg](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11m-seg.pt) | 640 | 51.5 | 41.5 | 281.6 ± 1.2 | 6.3 ± 0.1 | 22.4 | 113.2 |
| [YOLO11l-seg](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11l-seg.pt) | 640 | 53.4 | 42.9 | 344.2 ± 3.2 | 7.8 ± 0.2 | 27.6 | 132.2 |
| [YOLO11x-seg](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x-seg.pt) | 640 | 54.7 | 43.8 | 664.5 ± 3.2 | 15.8 ± 0.7 | 62.1 | 296.4 |
- 사전 학습에 사용된 데이터셋: [COCO](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco128-seg.yaml)

## YOLO11n 모델 기반의 Segmentation 실습

1. `/home/ubuntu/ros2_ws/src/yolo_ros/yolo_bringup/launch/yolo.launch.py`를 열고 아래와 같이 수정해줍니다.
    
    ```bash
    ...
    
    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument(
        "model",
        **default_value="yolo11n-seg.pt", # 수정**
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

![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2015-05-38.png)

## 활용 프로젝트

### **실내 바닥 물체(Obstacle) Segmentation & Nav2 연동**

- Depth Camera 활용 가능 시
- segmentation으로 작은 장애물, 케이블, 가방 등을 분리
- 마스크를 Costmap Layer로 변환해 Nav2에 반영

### Mono Depth Estimation + Segmentation
![](/assets/img/HiWonderPi/img/download/Mono_Depth_Estimation_+_Segmentation.png)