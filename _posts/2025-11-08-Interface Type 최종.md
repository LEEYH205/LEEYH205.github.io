---
title: "Interface Type 비교"
date: 2025-11-08 15:50:00 +0900
categories: [ROBOTICS]
tags: [Robotics, Interface]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---


# Interface Type 비교

## 메시지
- 단일값
- 또 다른 메시지를 활용해서 메시지를 꾸밀 수 있다.
- 2025-10-30-ROS2-기초.md

토픽 같은 경우, 토픽에서 사용하는 메시지가 있었음.
토픽이라고 하는 통신 방식은 지속적인 데이터 스트림을 목적
    단방향통신, publisher, subscription 모델이였고, 주로 센서 데이터나 로봇 상태, 정기적으로 뭔가 정보를 제공해야할 때, 받아서 쓸 때, 이 때 사용하는게 토픽, 그 안에는 메시지가 구성

## 서비스
- Request, Response 으로 두 섹션으로 나눠져 있다.
- 2025-11-05-Service(1).md

즉각적으로 요청과 응답이 필요한 경우.
호흡이 짧음.
기본적으로는 동기통신이긴한데, 비동기통신이 권장됨.
간단하게 연산 수행. 어떤 특정 노드에 파라미터 세팅한다 등.

## 액션
- Goal, Result, Feedback 으로 세 섹션으로 나눠져 있다.
- 2025-11-08-Action.md

장기적인 실행 작업에 적합하다.
진행 상황 모니터링이 Feedback 값으로 중간 상태 모니터링 가능.
작업을 중간에 취소도 가능.
비동기 통신.
주행 남은 거리, 현재 속도, 현재 위치, 로봇팔의 현재 각도 등.
주행 결과, 로봇팔 결과 등.

---

