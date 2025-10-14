---
title: "Transformer 아키텍처 정리: Attention Is All You Need"
date: 2025-08-06 19:00:00 +0900
categories: [AI, NLP, 딥러닝]
tags: [Transformer, Attention, BERT, GPT, 자연어처리]
---

# Transformer 아키텍처 정리: Attention Is All You Need

2017년 Google이 발표한 "Attention Is All You Need" 논문은 자연어 처리 분야에 혁신을 가져온 Transformer 아키텍처를 소개했습니다. 이 글에서는 Transformer의 핵심 개념과 작동 원리를 자세히 살펴보겠습니다.

## 🤔 Transformer가 필요한 이유

기존의 RNN(순환 신경망)과 LSTM은 순차적으로 데이터를 처리해야 하는 한계가 있었습니다:

- **병렬 처리 불가**: 이전 단계가 끝나야 다음 단계를 처리할 수 있음
- **장기 의존성 문제**: 긴 문장에서 앞쪽 정보를 잊어버리는 현상
- **계산 복잡도**: 시퀀스 길이에 비례하여 계산량이 증가

Transformer는 이러한 문제들을 해결하기 위해 **Attention 메커니즘**을 핵심으로 하는 새로운 아키텍처를 제안했습니다.

## 🏗️ Transformer 아키텍처 구조

### 기본 구성 요소

Transformer는 크게 **인코더(Encoder)**와 **디코더(Decoder)**로 구성됩니다:

```
입력 문장 → [인코더 스택] → [디코더 스택] → 출력 문장
```

### 핵심 구성 요소

1. **Multi-Head Attention**
2. **Feed-Forward Network**
3. **Positional Encoding**
4. **Layer Normalization**
5. **Residual Connection**

## 🔍 Attention 메커니즘의 핵심: Q, K, V

Attention의 핵심은 **Query(Q)**, **Key(K)**, **Value(V)** 세 가지 벡터입니다:

### 1. Query (Q): "내가 찾고 싶은 정보"

- **역할**: 각 토큰이 "이 문장에서 어떤 다른 단어를 얼마나 주목해야 할지" 묻는 역할
- **비유**: 돋보기를 들고 "여기를 집중해서 봐 달라"고 요청하는 손가락

### 2. Key (K): "문장 전체의 특징"

- **역할**: 각 토큰이 "내가 가진 정보를 다른 토큰에게 보여줄 때 어떤 키워드를 쓰는지"를 나타냄
- **비유**: 책의 각 줄마다 붙어 있는 "색인(인덱스) 키워드"

### 3. Value (V): "실제 꺼내 보여줄 정보"

- **역할**: 각 토큰이 "자신이 전달하려는 실제 값(의미 벡터)"을 담고 있음
- **비유**: 책에서 진짜로 읽어서 들려주고 싶은 "본문 내용"

## ⚙️ Attention 계산 과정

```python
# Attention 계산 과정 (의사코드)
def attention(Q, K, V):
    # 1. Q와 K의 곱으로 Attention Score 계산
    scores = Q @ K.T / sqrt(d_k)
    
    # 2. Softmax로 확률 분포로 변환
    attention_weights = softmax(scores)
    
    # 3. 가중치로 V를 가중합하여 최종 출력
    output = attention_weights @ V
    
    return output
```

### 단계별 설명

1. **Q와 K의 곱**: "어떤 토큰이 다른 토큰을 얼마나 주목해야 할지" 점수 계산
2. **Softmax**: 점수를 확률처럼 바꿔서 중요도가 높은 쪽에 집중
3. **가중합**: 중요한 정보(V)만 추려서 최종 출력 생성

## 🎯 Multi-Head Attention

하나의 Attention만 사용하는 것이 아니라, 여러 개의 Attention을 병렬로 실행합니다:

- **장점**: 다양한 관점에서 문맥을 이해할 수 있음
- **예시**: 문법적 관계, 의미적 관계, 위치적 관계 등을 동시에 학습

## 📍 Positional Encoding

Transformer는 순차적 처리가 없기 때문에 **위치 정보**를 별도로 주입해야 합니다:

```python
# 위치 인코딩 공식
PE(pos, 2i) = sin(pos / 10000^(2i/d_model))
PE(pos, 2i+1) = cos(pos / 10000^(2i/d_model))
```

## 🚀 Transformer의 장점

### 1. 병렬 처리 가능
- RNN과 달리 모든 토큰을 동시에 처리
- 훈련 속도가 크게 향상

### 2. 장기 의존성 해결
- Attention을 통해 멀리 떨어진 단어들 간의 관계도 직접 학습
- 정보 손실 없이 긴 문장 처리 가능

### 3. 확장성
- 모델 크기와 성능이 선형적으로 증가
- 더 큰 모델로 더 좋은 성능 달성 가능

## 🌟 Transformer의 영향

Transformer 아키텍처는 다음과 같은 혁신적인 모델들의 기반이 되었습니다:

- **BERT**: 양방향 인코더만 사용
- **GPT**: 단방향 디코더만 사용  
- **T5**: 인코더-디코더 구조 활용
- **ChatGPT**: GPT 계열의 대화형 AI

## 💡 실무에서의 활용

### 1. 텍스트 분류
```python
# BERT를 활용한 감정 분석
from transformers import BertTokenizer, BertForSequenceClassification

tokenizer = BertTokenizer.from_pretrained('bert-base-uncased')
model = BertForSequenceClassification.from_pretrained('bert-base-uncased')
```

### 2. 기계 번역
```python
# T5를 활용한 번역
from transformers import T5Tokenizer, T5ForConditionalGeneration

tokenizer = T5Tokenizer.from_pretrained('t5-small')
model = T5ForConditionalGeneration.from_pretrained('t5-small')
```

### 3. 텍스트 생성
```python
# GPT를 활용한 텍스트 생성
from transformers import GPT2Tokenizer, GPT2LMHeadModel

tokenizer = GPT2Tokenizer.from_pretrained('gpt2')
model = GPT2LMHeadModel.from_pretrained('gpt2')
```

## 🎯 마무리

Transformer는 자연어 처리 분야의 패러다임을 완전히 바꾼 혁신적인 아키텍처입니다. Attention 메커니즘을 통해 순차적 처리의 한계를 극복하고, 병렬 처리의 장점을 최대화했습니다.

현재 우리가 사용하는 ChatGPT, BERT, 그리고 수많은 AI 모델들이 모두 Transformer의 후손이라고 할 수 있습니다. AI 분야에 관심이 있다면 반드시 이해해야 할 핵심 기술 중 하나입니다.

---

**참고 자료:**
- [Attention Is All You Need 논문](https://arxiv.org/abs/1706.03762)
- [Transformer 시각화 도구](http://jalammar.github.io/illustrated-transformer/)
- [Hugging Face Transformers 라이브러리](https://huggingface.co/transformers/)

*이 글이 도움이 되셨다면 댓글로 피드백을 남겨주세요! 🚀*
