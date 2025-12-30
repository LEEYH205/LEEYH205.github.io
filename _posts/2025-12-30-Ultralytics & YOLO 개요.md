---
title: "Ultralytics & YOLO 개요"
date: 2025-12-30 11:40:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, Ultralytics, YOLO]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

# **Ultralytics**

---

## 개요

### **Ultralytics란?**

- 대표적인 **오픈소스 Computer Vision 기업**
- YOLO 시리즈의 최신 버전(특히 YOLOv8~YOLO12까지)을 주도적으로 개발하고 관리
- Detection · Segmentation · Pose · Classification 등 다양한 모델 제공
- Python API, CLI 모두 지원 → 빠른 프로토타입 제작 가능
    
    ```python
    # 단 2줄로 모델 학습 가능
    from ultralytics import YOLO
    model = YOLO('yolo12n.pt')
    model.train(data='custom.yaml', epochs=100)
    ```
    
- 다양한 하드웨어(PC·Jetson·Raspberry Pi·Edge Device)에서 동작하도록 최적화


### **Ultralytics를 많이 쓰는 이유**

- **일관된 API** → 하나의 코드로 여러 모델 교체 가능
- **손쉬운 학습/추론 파이프라인**
    
    ```python
    # 단 2줄로 추론 가능
    from ultralytics import YOLO
    model = YOLO("yolov12n.pt")  
    results = model("bus.jpg")
    ```
    
- **ONNX / TensorRT / OpenVINO 등 다양한 포맷으로 내보내기(export)** 지원
- 풍부한 문서와 커뮤니티


# **YOLO (You Only Look Once)**

---

## 개요

### **YOLO란?**

- 대표적인 **1-stage Object Detection** 모델
- 입력 이미지를 한 번만 보며(Once) 객체의 위치와 클래스 예측
- 빠른 속도와 높은 정확도로 실시간 시스템(로봇·드론·자율주행)에 최적

### **YOLO의 발전 흐름**


| **버전** | **발표 시기** | **주요 특징** | **설명** |
| --- | --- | --- | --- |
| **YOLOv1 (2016)** | Joseph Redmon | “You Only Look Once” 탄생. 한 번에 객체 예측. | 실시간 객체 탐지 시대 개막 |
| **YOLOv2 (2017)** | Redmon | 정확도 향상, 앵커(Anchor) 도입 | 더 빠르고 실용적인 모델 |
| **YOLOv3 (2018)** | Redmon | 3-스케일 출력, 작은 물체 인식 강화 | 현재까지도 많이 쓰임 |
| **YOLOv4 (2020)** | Alexey | CSPDarknet, Mosaic 등 훈련 기술 대거 추가 | 현실 환경에 강한 모델 |
| **YOLOv5 (2020)** | Ultralytics | PyTorch 기반 YOLO. 쉬운 사용성으로 대중화 | pip만으로 설치, 사실상 표준 |
| **YOLOv6 (2022)** | Meituan(MMYOLO) | 산업용 효율 중점, 빠름 | 배달·로보틱스용 튜닝 모델 |
| **YOLOv7 (2022)** | WongKinYiu | 당시 SOTA 성능. E-ELAN 구조 | 정확도와 속도 모두 SOTA |
| **YOLOv8 (2023)** | Ultralytics | 모듈 통합, segmentation·pose 통합 모델 | 가장 널리 사용되는 범용 YOLO |
| **YOLOv9 (2024)** | WongKinYiu | GELAN + Programmable Gradient | 구조적으로 더 깊은 연구 모델 |
| **YOLOv10 (2024)** | Tencent | NMS 없는 End-to-End 설계 | NMS 제거로 속도↑ 정확도↑ |
| **YOLO11 (2024)** | Ultralytics | 최신 실용 모델. 더 정확하고 빠름 | 실제 제품/로봇/IoT에 최적화 |
| **YOLO12 (2025)** | Ultralytics | 공식 테스트 중, YOLO11 개선판 | 데이터 효율·속도 개선 가능성 |

### **Ultralytics YOLO11**
![](/assets/img/HiWonderPi/img/download/Ultralytics%20YOLO11.png)

- **향상된 특징 추출:** YOLO11은 개선된 [백본](https://www.ultralytics.com/glossary/backbone) 및 넥 아키텍처를 사용하여 보다 정확한 객체 탐지 및 복잡한 작업 수행을 위한 [특징 추출](https://www.ultralytics.com/ko/glossary/feature-extraction) 기능을 향상
- **효율성 및 속도에 최적화됨:** YOLO11은 개선된 아키텍처 설계 및 최적화된 훈련 파이프라인을 도입하여 더 빠른 처리 속도를 제공하고 정확도와 성능 간의 최적 균형을 유지
- **더 적은 파라미터로 더 높은 정확도:** YOLO11m은 COCO 데이터 세트에서 더 높은 [mAP](https://www.ultralytics.com/glossary/mean-average-precision-map)를 달성하는 동시에 YOLOv8m 대비 22% 적은 수의 매개변수를 사용하여 정확도 저하 없이 계산 효율을 높임
- **다양한 환경에 대한 적응성:** YOLO11은 엣지 장치, 클라우드 플랫폼 및 NVIDIA GPU를 지원하는 시스템을 포함한 다양한 환경에 원활하게 배포할 수 있어 최대한의 유연성을 보장

### **Ultralytics YOLO12**

![](/assets/img/HiWonderPi/img/download/Ultralytics%20YOLO12.png)

- 이전 YOLO 모델들의 CNN 기반 백본 구조에서 벗어나 **어텐션(Attention) 메커니즘**을 효율적으로 통합
- 주요 개선점
    - 최신 BackBone 아키텍처 적용 (효율 + 정확도 개선)
    - 더 깊은 Feature Fusion Layer로 작은 물체에 대한 성능 향상
- 이 모델은 실시간 성능을 유지하면서 주의 메커니즘과 전반적인 네트워크 아키텍처의 새로운 방법론적 혁신을 통해 최첨단 객체 감지 정확도를 달성
    - 이러한 장점에도 불구하고, YOLO12는 커뮤니티 중심 릴리스로서 훈련 불안정성, 메모리 사용량 증가, CPU 처리 효율 저하를 나타낼 수 있음
    - 대부분의 프로덕션 레벨에는 **`여전히 YOLO11 권장`**