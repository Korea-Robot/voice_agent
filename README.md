# Voice Agent

## Table of Contents

-   [About](#about)
-   [Getting Started](#getting_started)
-   [Usage](#usage)
-   [Troubleshooting](#troubleshooting)
-   [Contributors](#contributors)
-   [Maintainer](#maintainer)

## About <a name = "about"></a>

로봇과 사람이 양방향 소통을 할 수 있도록 만든 ROS2 기반 음성 에이전트 어플리케이션입니다.
자동으로 음성을 인식하여 파일을 저장하고, 이를 기반으로 답변을 만들어 음성으로 내뱉어줍니다.
또한 명령 모드를 통해 로봇을 제어할 수 있습니다.

### 주요 기능

- **음성 인식**: OpenAI Whisper API를 사용한 실시간 음성-텍스트 변환
- **텍스트 생성**: OpenAI GPT API를 사용한 지능형 응답 생성
- **음성 합성**: OpenAI TTS API를 사용한 텍스트-음성 변환
- **명령 모드**: 로봇 제어 명령 처리 (앉아, 일어나, 걷기, 이동, 회전 등)
- **기본 모드**: 일반 대화 처리
- **ROS2 통합**: ROS2 서비스 및 토픽을 통한 로봇 제어
- **자동 오디오 장치 감지**: PulseAudio를 통한 자동 마이크/스피커 장치 선택

## Getting Started <a name = "getting_started"></a>

### 사전 요구사항

#### 시스템 패키지

```bash
sudo apt install portaudio19-dev
```

#### ROS2 의존성

이 패키지는 ROS2 Humble 환경에서 작동합니다. 다음 ROS2 패키지가 필요합니다:

- `rclpy`
- `geometry_msgs`
- `ghost_manager_interfaces` ⚠️ **비공개 레포지토리**

> **주의**: `ghost_manager_interfaces`는 비공개(private) 레포지토리입니다. 이 패키지는 Vision60 로봇에 특화된 인터페이스를 제공합니다. 다른 로봇에 이 패키지를 사용하려면 다음 부분들을 직접 로봇에 맞게 수정해야 합니다:
> - `voice_to_action.py`의 ROS2 서비스 및 토픽 인터페이스
> - `voice_agent.py`의 `publish_ros2_command` 메서드
> - `package.xml`의 의존성 패키지

#### Python 패키지

```bash
pip install -r requirements.txt
```

또는 개별 설치:

```bash
pip install openai>=1.14.2
pip install PyAudio>=0.2.14
pip install webrtcvad>=2.0.10
pip install pydub>=0.24.0
pip install numpy>=1.21.0
pip install scipy>=1.7.0
pip install pyqt5>=5.15.0
```

### OpenAI API 키 설정

사용을 위해서는 OpenAI API 키가 필요합니다.

발급은 이 [사이트](https://platform.openai.com/api-keys)에서 진행하시면 됩니다.

발급받은 키는 아래와 같이 환경변수로 등록시켜주세요.

```bash
echo "export OPENAI_API_KEY='발급받은 API키'" >> ~/.bashrc
source ~/.bashrc
```

### ROS2 워크스페이스 빌드

```bash
cd /path/to/your/workspace
colcon build --packages-select voice_agent
source install/setup.bash
```

## Usage <a name = "usage"></a>

### 마이크 테스트

마이크 장치 선택 및 녹음 기능을 테스트할 수 있습니다:

```bash
ros2 run voice_agent test_microphone
```

### ROS2 노드로 실행 (권장)

음성 에이전트를 ROS2 노드로 실행합니다. 이 방법은 로봇 제어 기능을 포함합니다.

```bash
# Launch 파일 사용
ros2 launch voice_agent voice_agent_launch.py

# 또는 직접 실행
ros2 run voice_agent voice_to_action
```

### UI 모드로 실행

PyQt5 기반 GUI를 사용하여 단계별로 실행할 수 있습니다.

```bash
ros2 run voice_agent voice_ui
```

또는 직접 실행:

```bash
python3 -m voice_agent.ui
```

### 직접 실행 (비-ROS2 모드)

ROS2 없이 기본 음성 에이전트만 실행하려면:

```bash
python3 -m voice_agent.voice_agent --mic_index 0
```

### 마이크 테스트

마이크 장치 선택 및 녹음 기능을 테스트할 수 있습니다:

```bash
python3 -m voice_agent.test_microphone
```

### 사용 방법

#### 기본 모드

음성 에이전트는 기본적으로 일반 대화 모드로 작동합니다. 말을 하면 AI가 응답을 생성하고 음성으로 답변합니다.

#### 명령 모드

"명령 모드"라고 말하면 로봇 제어 명령을 처리할 수 있습니다.

**지원되는 명령:**

| 카테고리 | 명령어 | 설명 |
|---------|--------|------|
| 상태 변경 | "앉아" | 로봇을 앉힘 (action: 0) |
| | "일어나" | 로봇을 일으킴 (action: 1) |
| | "걷기 모드" | 걷기 모드 활성화 (action: 2) |
| 이동 명령 | "앞으로 가" | 앞으로 이동 |
| | "뒤로 가" | 뒤로 이동 |
| | "왼쪽으로 가" | 왼쪽으로 이동 |
| | "오른쪽으로 가" | 오른쪽으로 이동 |
| 회전 명령 | "왼쪽으로 회전" 또는 "왼쪽으로 돌아" | 왼쪽으로 회전 |
| | "오른쪽으로 회전" 또는 "오른쪽으로 돌아" | 오른쪽으로 회전 |
| 속도 조절 | "빠르게" 또는 "최대한 빠르게" | 빠른 속도로 실행 (0.7) |
| | "느리게" 또는 "천천히" | 느린 속도로 실행 (0.2) |
| 정지 | "멈춰" 또는 "정지" | 현재 동작 중단 |

**명령 모드 종료:**

"기본 모드"라고 말하면 일반 대화 모드로 돌아갑니다.

#### 종료

"종료"라고 말하면 음성 에이전트가 종료됩니다.

### ROS2 통합

음성 에이전트는 다음 ROS2 인터페이스를 사용합니다:

- **서비스 클라이언트**: `/ensure_mode` (상태 변경 명령)
- **토픽 퍼블리셔**: `/mcu/command/manual_twist` (이동 명령)

## Troubleshooting <a name = "troubleshooting"></a>

### 마이크 장치 문제

음성 에이전트는 자동으로 사용 가능한 마이크 장치를 감지합니다. 우선순위는 다음과 같습니다:

1. `pulse` 장치 (PulseAudio를 통해 C-type 마이크 접근)
2. `default` 장치
3. 첫 번째 사용 가능한 입력 장치

마이크가 제대로 인식되지 않는 경우:

```bash
# 오디오 장치 사용 확인
fuser -v /dev/snd/*
```

PulseAudio가 정상적으로 작동하는지 확인:

```bash
pulseaudio --check -v
```

### 오디오 장치 오류

```
ALSA lib pcm_dsnoop.c:567:(snd_pcm_dsnoop_open) unable to open slave
```

이는 정상적인 경고 메시지일 수 있으며, 에이전트는 정상적으로 작동할 수 있습니다.

### API 키 오류

```
OpenAI API key not found
```

`OPENAI_API_KEY` 환경 변수가 설정되어 있는지 확인하세요:

```bash
echo $OPENAI_API_KEY
```

### 마이크가 뒷쪽으로 설정되는 문제 (Vision60 특화)

만약 마이크가 뒷쪽으로 설정이 되고, 음성이 한쪽만 나온다면 다른 프로세스가 장치를 사용하고 있을 수 있습니다.

연결된 태블릿에서 `sounds` 설정을 확인하고, 필요시 체크박스를 해제한 후 재부팅하세요.

### ROS2 서비스 연결 오류 (Vision60 특화)

```
ensure_mode 서비스가 사용 불가능합니다.
```

ROS2 노드가 실행 중인지 확인하세요:

```bash
ros2 service list | grep ensure_mode
```

### VAD (Voice Activity Detection) 민감도 조정

음성 인식이 잘 안될 때는 `voice_agent.py`의 VAD 설정을 조정할 수 있습니다:

- `self.vad = webrtcvad.Vad(3)` - 최고 민감도 (기본값)
- `amplitude_threshold` - 음량 임계값 (낮을수록 작은 목소리도 인식)
- `min_consecutive_voiced` - 최소 연속 발화 프레임 (낮을수록 빠른 시작)
- 더 디테일한 조정은 [VAD 파라미터 조정 가이드](docs/VAD_PARAMETERS.md)를 참고하세요.

## Contributors

- **[Incheol Cho](https://github.com/incheol67)** (dlscjf1998@gmail.com)
- **[Jinwon Kim](https://github.com/mqjinwon)** (jwkim@krm.co.kr) - Maintainer

## Maintainer

- **[Jinwon Kim](https://github.com/mqjinwon)**
