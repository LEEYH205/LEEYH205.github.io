---
title: "Object Detection 실습"
date: 2025-12-30 13:25:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, Ultralytics, YOLO, Object Detection]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

# Object Detection

---

- [Object Detection](https://www.ultralytics.com/glossary/object-detection)는 이미지 또는 비디오 스트림에서 객체의 위치와 클래스를 식별하는 작업
- 모델의 출력은 이미지 내 객체를 둘러싸는 경계 상자 집합과 각 상자에 대한 클래스 레이블 및 신뢰도 점수
- 객체가 어디에 있는지 또는 정확한 모양을 알 필요 없이 장면에서 관심 있는 객체를 식별해야 하는 경우 객체 감지가 좋은 선택이 됨

## 사용 가능 모델

| **모델** | **크기(픽셀)** | **mAP 50-95** | **속도 CPU ONNX(ms)** | **속도 T4 TensorRT10(ms)** | **파라미터(M)** | **FLOPs(B)** |
| --- | --- | --- | --- | --- | --- | --- |
| [YOLO11n](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt) | 640 | 39.5 | 56.1 ± 0.8 | 1.5 ± 0.0 | 2.6 | 6.5 |
| [YOLO11s](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s.pt) | 640 | 47.0 | 90.0 ± 1.2 | 2.5 ± 0.0 | 9.4 | 21.5 |
| [YOLO11m](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11m.pt) | 640 | 51.5 | 183.2 ± 2.0 | 4.7 ± 0.1 | 20.1 | 68.0 |
| [YOLO11l](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11l.pt) | 640 | 53.4 | 238.6 ± 1.4 | 6.2 ± 0.1 | 25.3 | 86.9 |
| [YOLO11x](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x.pt) | 640 | 54.7 | 462.8 ± 6.7 | 11.3 ± 0.2 | 56.9 | 194.9 |

- 사전 학습에 사용된 데이터셋: [COCO](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco.yaml)


## YOLO11n 모델 기반의 Object Detection 실습

1. `/home/ubuntu/ros2_ws/src/yolo_ros/yolo_bringup/launch/yolo.launch.py`를 열고 아래와 같이 수정해줍니다.
    
    ```bash
    ...
    
    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument(
        "model",
        **default_value="yolo11n.pt", # 수정**
        description="Model name or path",
    )
    
    ...
    
    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        **default_value="cpu", # 수정**
        description="Device to use (GPU/CPU)",
    )
    
    ...
    
    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        **default_value="/ascamera/camera_publisher/rgb0/image", # 수정**
        description="Name of the input image topic",
    )
    
    image_reliability = LaunchConfiguration("image_reliability")
    image_reliability_cmd = DeclareLaunchArgument(
        "image_reliability",
        **default_value="2", # 수정**
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)",
    )
    ```
    
    > **참고: Raspberry Pi 5의 GPU 사용**  
    > Raspberry Pi 5에는 VideoCore VII GPU가 내장되어 있지만, 이는 주로 그래픽 렌더링에 최적화되어 있어 딥러닝 추론에는 제한적입니다. Ultralytics YOLO는 CUDA나 TensorRT와 같은 GPU 가속을 지원하는데, Raspberry Pi의 GPU는 이러한 프레임워크와 호환되지 않습니다. 따라서 Raspberry Pi 5에서는 CPU를 사용하는 것이 일반적입니다. 성능 향상이 필요한 경우, 더 가벼운 모델(YOLO11n)을 사용하거나 외부 GPU를 연결하는 방법을 고려할 수 있습니다.
    
2. 아래 명령어로 런치파일을 실행합니다.
    
    ```bash
    ros2 launch yolo_bringup yolo.launch.py
    ```
    
    ![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2014-30-37.png)
    ![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2014-31-20.png)
3. 아래 명령어로 YOLO 관련 토픽들을 확인할 수 있습니다.
    
    ```bash
    ros2 topic list
    ```

    ![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2014-31-45.png)
    ![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2014-34-39.png)
    ![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2014-36-55.png)
    실시간으로 사용하기엔 무리가 있어 보인다.