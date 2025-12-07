# 음성 인식 하이퍼파라미터 가이드

## 🔧 조정 가능한 파라미터

### 1. VAD 민감도 (가장 중요!)
**위치**: `voice_agent.py` 라인 141
```python
self.vad = webrtcvad.Vad(2)
```

**값 범위**: 0 (가장 낮음) ~ 3 (가장 높음)

**조정 방법**:
- **인식이 잘 안될 때**: `Vad(3)` (가장 높은 민감도)
- **너무 민감할 때**: `Vad(1)` 또는 `Vad(0)`

### 2. 음량 임계값 (amplitude_threshold)
**위치**: `voice_agent.py` 라인 365
```python
amplitude_threshold = 1000
```

**의미**: 최소 음량 임계값 (int16 기준, 최대 32767)

**조정 방법**:
- **작은 목소리도 인식하려면**: `500` ~ `800` (낮춤)
- **잡음이 많을 때**: `1500` ~ `2000` (높임)

### 3. 최소 연속 발화 프레임 (min_consecutive_voiced)
**위치**: `voice_agent.py` 라인 366
```python
min_consecutive_voiced = 5
```

**의미**: 음성 시작을 인식하기 위한 최소 연속 발화 프레임 수

**조정 방법**:
- **빠른 시작 인식**: `3` ~ `4` (낮춤)
- **안정적인 시작**: `5` ~ `7` (높임)

### 4. 시작 트리거 임계값
**위치**: `voice_agent.py` 라인 404
```python
if num_voiced > 0.9 * NUM_WINDOW_CHUNKS
```

**의미**: 윈도우 내 유성음 프레임 비율

**조정 방법**:
- **더 쉽게 시작**: `0.7` ~ `0.8` (낮춤)
- **더 확실한 시작**: `0.9` ~ `0.95` (높임)

### 5. 종료 트리거 임계값
**위치**: `voice_agent.py` 라인 417
```python
if num_unvoiced > 0.90 * NUM_WINDOW_CHUNKS_END

**의미**: 윈도우 내 무성음 프레임 비율

**조정 방법**:
- **더 오래 녹음 (천천히 종료)**: `0.95` ~ `0.98` (높임)
- **빠르게 종료**: `0.85` ~ `0.90` (낮춤)

### 6. 윈도우 크기
**위치**: `voice_agent.py` 라인 116
```python
NUM_WINDOW_CHUNKS = int(300 / CHUNK_DURATION_MS)
NUM_WINDOW_CHUNKS_END = NUM_WINDOW_CHUNKS
```

**조정 방법**:
- **더 빠른 반응**: `200` ~ `250` (낮춤)
- **더 안정적인 감지**: `400` ~ `500` (높임)

## 💡 권장 조정 값 (인식 개선)

인식이 잘 안될 때 다음 순서로 조정:

1. **VAD 민감도 높이기** (가장 효과적)
   ```python
   self.vad = webrtcvad.Vad(3)  # 최고 민감도
   ```

2. **음량 임계값 낮추기**
   ```python
   amplitude_threshold = 500  # 작은 목소리도 인식
   ```

3. **시작 임계값 낮추기**
   ```python
   if num_voiced > 0.7 * NUM_WINDOW_CHUNKS  # 70%로 낮춤
   ```

4. **최소 연속 발화 프레임 낮추기**
   ```python
   min_consecutive_voiced = 3  # 더 빠른 시작
   ```

5. **종료 임계값 높이기** (더 오래 녹음)
   ```python
   if num_unvoiced > 0.95 * NUM_WINDOW_CHUNKS_END  # 95%로 높임
   ```

## 🎯 상황별 권장 설정

### 상황 1: 음성이 시작되지 않음
```python
self.vad = webrtcvad.Vad(3)  # 최고 민감도
amplitude_threshold = 500  # 낮은 임계값
min_consecutive_voiced = 3  # 빠른 시작
if num_voiced > 0.7 * NUM_WINDOW_CHUNKS  # 낮은 시작 임계값
```

### 상황 2: 너무 빨리 종료됨
```python
if num_unvoiced > 0.95 * NUM_WINDOW_CHUNKS_END  # 높은 종료 임계값
```

### 상황 3: 잡음에 너무 민감함
```python
self.vad = webrtcvad.Vad(1)  # 낮은 민감도
amplitude_threshold = 1500  # 높은 임계값
if num_voiced > 0.9 * NUM_WINDOW_CHUNKS  # 높은 시작 임계값
```

