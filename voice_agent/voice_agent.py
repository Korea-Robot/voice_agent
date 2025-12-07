"""
This file contains the implementation of a VoiceAgent class that interacts with the OpenAI API to perform speech-to-text, text generation, and text-to-speech tasks. It also includes an example usage of the VoiceAgent class.

The VoiceAgent class has the following methods:
- speech_to_text: Converts an audio file to text using the OpenAI API.
- text_generation: Generates a response to a user's question using the OpenAI API.
- text_to_speech: Converts text to speech using the OpenAI API.
- save_audio: Saves the generated audio content to a file.

Example usage:
- Create an instance of the VoiceAgent class with an API key.
- Use the speech_to_text method to convert an audio file to text.
- Use the text_generation method to generate a response to the user's question.
- Use the text_to_speech method to convert the generated response to speech.
- Use the  save_audio method to save the generated audio content to a file.
"""

from openai import OpenAI
import pyaudio
import rclpy.node
import webrtcvad
from pydub import AudioSegment
import numpy as np
from scipy import signal

import subprocess
from typing import Union
from threading import Lock, Thread
from array import array
from struct import pack
import wave
import time
import collections
import sys
import os

import abc
import argparse

import rclpy

DEFAULT_PERSONA = (
    "You are Vision 60, a quadruped robot designed to navigate and adapt in complex environments. "
    "You understand Korean, English. You can answer questions freely like a helpful assistant."
    "you will response to my question up to 50 tokens."
)

COMMAND_PERSONA = (
    "You are Vision 60, a quadruped robot designed to navigate and adapt in complex environments. "
    "You understand both basic **state change** commands and **motion commands**.\n\n"

    "1. **State Change Commands**:\n"
    "These are simple behavioral commands. Respond with only the number:\n"
    "- '앉아' (sit) → 0\n"
    "- '일어나' (stand) → 1\n"
    "- '걷기 모드' (walk) → 2\n\n"

    "### Examples:\n"
    "'앉아줘' → 0\n"
    "'일어나자' → 1\n"
    "'걷기 시작해' → 2\n\n"

    "2. **Motion Commands (Twist Publishing)**:\n"
    "Respond with a list of 4 values:\n"
    "[vx, vy, vyaw, duration]  (all in float)\n\n"

    "Speed Interpretation:\n"
    "- Default speed: 0.4\n"
    "- If command contains '빠르게' or '최대한 빠르게': speed = 0.7\n"
    "- If command contains '느리게' or '천천히': speed = 0.2\n\n"

    "Direction Interpretation:\n"
    "- vx > 0 → forward\n"
    "- vx < 0 → backward\n"
    "- vy > 0 → move left\n"
    "- vy < 0 → move right\n"
    "- vyaw > 0 → rotate left\n"
    "- vyaw < 0 → rotate right\n\n"
    "- duration: how long to move (seconds)\n\n"

    "Rules:\n"
    "- Respond ONLY with the list.\n"
    "- No explanation or extra text.\n"
    "- You understand Korean, English.\n"
    
    "### Examples:\n"
    "'앞으로 가' → [0.4, 0.0, 0.0, 1]\n"
    "'앞으로 3초 동안 가' → [0.4, 0.0, 0.0, 3]\n"
    "'빠르게 앞으로 가' → [0.7, 0.0, 0.0, 1]\n"
    "'느리게 앞으로 2초 동안 가' → [0.2, 0.0, 0.0, 2]\n"
    "'왼쪽으로 가' → [0.0, 0.4, 0.0, 1]\n"
    "'오른쪽으로 가' → [0.0, -0.4, 0.0, 1]\n"
    "'천천히 오른쪽으로 5초 동안 가' → [0.0, -0.4, 0.0, 5]\n"
    "'오른쪽으로 회전해' 또는 '오른쪽으로 돌아' → [0.0, 0.0, -0.4, 1]\n"
    "'오른쪽으로 5초 동안 회전해' 또는 '오른쪽으로 5초 동안 돌아' → [0.0, 0.0, -0.4, 5]\n"
    "'왼쪽으로 회전해' 또는 '왼쪽으로 돌아' → [0.0, 0.0, 0.4, 1]\n"
    "'왼쪽으로 5초 동안 회전해' 또는 '왼쪽으로 5초 동안 돌아' → [0.0, 0.0, 0.4, 5]\n"
    "'빠르게 왼쪽으로 3초 동안 회전해' → [0.0, 0.0, 0.7, 3]\n"
    "'빠르게 오른쪽으로 3초 동안 회전해' → [0.0, 0.0, -0.7, 3]\n"
    "'느리게 오른쪽으로 회전해' → [0.0, 0.0, -0.2, 1]\n"
    "'멈춰' 또는 '정지' → [0.0, 0.0, 0.0, 1]\n"
    "'뒤로 가' → [-0.4, 0.0, 0.0, 1]\n"
    "'천천히 뒤로 가' → [-0.2, 0.0, 0.0, 1]\n"
    "'천천히 뒤로 4초 동안 가' → [-0.2, 0.0, 0.0, 4]\n"
    "'빠르게 왼쪽으로 2초 동안 가' → [0.0, 0.7, 0.0, 2]\n"

    "- Return int -1 if the command is invalid or ambiguous.\n\n"
)

# Audio settings
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 48000
CHUNK_DURATION_MS = 20  # supports 10, 20 and 30 (ms)
PADDING_DURATION_MS = 1000  # 1 sec jugement
CHUNK_SIZE = int(RATE * CHUNK_DURATION_MS / 1000)  # chunk to read
NUM_PADDING_CHUNKS = int(PADDING_DURATION_MS / CHUNK_DURATION_MS)
NUM_WINDOW_CHUNKS = int(300 / CHUNK_DURATION_MS)  # 300 ms/ 30ms  ge
NUM_WINDOW_CHUNKS_END = NUM_WINDOW_CHUNKS # originally, it was NUM_WINDOW_CHUNKS * 2


class VoiceAgent:

    def __init__(
        self,
        api_key=None,
        base_path=os.path.dirname(os.path.abspath(__file__)),
        mic_index=0,
    ):
        """
        Initializes a VoiceAgent object with the provided API key.

        Args:
            api_key (str): The API key for accessing the OpenAI API.
            base_path (str, optional): The base path for file operations. Defaults to the directory of the current file.
        """

        if api_key is None:
            self.client = OpenAI()  # Get from environment variable
        else:
            self.client = OpenAI(api_key=api_key)

        # VAD 민감도: 0(가장 낮음) ~ 3(가장 높음)
        # 인식이 잘 안될 때는 3으로 높이기
        self.vad = webrtcvad.Vad(3)  # 최고 민감도로 설정 (인식 개선)
        self.base_path = base_path
        self.pa = pyaudio.PyAudio()

        # self.speaker_devices = ["plughw:0,0", "plughw:1,0"]
        self.speaker_devices = self.get_speaker_devices()
        self.microphone_devices = self.get_microphone_devices()

        print("--------------------------------")
        print(f"Speaker Devices: {self.speaker_devices}")
        print(f"Microphone Devices: {self.microphone_devices}")
        print("--------------------------------")

        self.sound_lock = Lock()
        self.command_mode = False
        self.active_twist_process = None  # 현재 Twist 퍼블리시 프로세스 저장

        self.ros_node = rclpy.node

        self.ensure_mode = self.ros_node.Service 

        try:
            # test.py와 동일한 방식으로 스트림 열기
            device_index = self.microphone_devices
            actual_rate = RATE  # 기본값
            
            # 장치 정보 가져오기 (test.py 방식)
            if device_index is not None:
                dev_info = self.pa.get_device_info_by_host_api_device_index(0, device_index)
                actual_rate = int(dev_info.get('defaultSampleRate', RATE))
                if actual_rate != RATE:
                    print(f"[INFO] 장치 샘플 레이트 ({actual_rate}Hz)에 맞춰 조정합니다.")
                print(f"[INFO] 선택 장치: {dev_info.get('name', 'Unknown')}")
                print(f"[INFO] 샘플링 레이트: {actual_rate}Hz")
            else:
                print("⚠️  마이크 장치가 None입니다. 기본 장치를 사용합니다.")
            
            # CHUNK_SIZE를 실제 샘플 레이트에 맞게 조정
            actual_chunk_size = int(actual_rate * CHUNK_DURATION_MS / 1000)
            
            # 실제 샘플 레이트와 청크 크기를 인스턴스 변수로 저장
            self.actual_rate = actual_rate
            self.actual_chunk_size = actual_chunk_size
            
            # VAD를 위한 샘플 레이트 결정 (webrtcvad는 8000, 16000, 32000, 48000만 지원)
            # 가장 가까운 지원 레이트로 선택하거나 48000으로 리샘플링
            vad_supported_rates = [8000, 16000, 32000, 48000]
            if actual_rate in vad_supported_rates:
                self.vad_rate = actual_rate
                self.needs_resample = False
            else:
                # 48000Hz로 리샘플링 (가장 일반적인 고품질 레이트)
                self.vad_rate = 48000
                self.needs_resample = True
                print(f"[INFO] VAD를 위해 오디오를 {actual_rate}Hz → {self.vad_rate}Hz로 리샘플링합니다.")
            
            self.stream = self.pa.open(
                format=FORMAT,
                channels=CHANNELS,
                rate=actual_rate,  # 장치의 실제 샘플 레이트 사용
                input=True,
                input_device_index=device_index,  # None이면 기본 장치 사용
                start=False,
                frames_per_buffer=actual_chunk_size,  # 실제 샘플 레이트에 맞춘 청크 크기
            )
        except Exception as e:
            raise RuntimeError(f"🎤 마이크 초기화 실패: {e}\n팁: 'pulse'나 'default' 장치를 사용해보세요.")

        os.makedirs(os.path.dirname("./media/audio.wav"), exist_ok=True)

    def get_speaker_devices(self):
        devices = []
        for i in range(self.pa.get_device_count()):
            device_info = self.pa.get_device_info_by_index(i)
            name = device_info["name"]
            max_output = device_info["maxOutputChannels"]

            # 출력 가능한 장치 + 필터링된 이름 조건만 등록
            if max_output > 0 and (name == "default" or "plughw:" in name):
                devices.append(name)

        return devices

    def get_microphone_devices(self):
        """
        마이크 장치를 찾아 인덱스를 반환합니다.
        test.py 기반의 간단한 접근 방식 사용.
        
        우선순위:
        1. 'pulse' 장치 (PulseAudio를 통해 C-type 마이크 접근 가능)
        2. 'default' 장치
        3. 첫 번째 사용 가능한 입력 장치
        
        Returns:
            int or None: 마이크 장치 인덱스
        """
        print("=" * 60)
        print(" [연결된 오디오 장치 목록] ")
        print("=" * 60)
        
        # test.py와 동일한 방식으로 장치 목록 조회
        info = self.pa.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')
        
        input_devices = []
        pulse_idx = None
        default_idx = None
        
        # test.py와 동일한 방식으로 장치 목록 생성
        for i in range(0, numdevices):
            device_info = self.pa.get_device_info_by_host_api_device_index(0, i)
            if device_info.get('maxInputChannels', 0) > 0:
                device_name = device_info.get('name', '')
                sample_rate = int(device_info.get('defaultSampleRate', 0))
                print(f"Index {i}: {device_name} (Default Rate: {sample_rate}Hz)")
                input_devices.append(i)
                
                # 'pulse' 장치 찾기
                if device_name.lower() == 'pulse':
                    pulse_idx = i
                
                # 'default' 장치 찾기
                if device_name.lower() == 'default':
                    default_idx = i
        
        print("=" * 60)
        
        # 우선순위 1: 'pulse' 장치 (PulseAudio를 통해 C-type 마이크 접근)
        if pulse_idx is not None:
            device_info = self.pa.get_device_info_by_host_api_device_index(0, pulse_idx)
            print(f"✅ 선택된 마이크: Index {pulse_idx} - {device_info['name']}")
            print("   💡 PulseAudio를 통해 C-type 마이크에 접근합니다.")
            return pulse_idx
        
        # 우선순위 2: 'default' 장치
        if default_idx is not None:
            device_info = self.pa.get_device_info_by_host_api_device_index(0, default_idx)
            print(f"✅ 선택된 마이크: Index {default_idx} - {device_info['name']}")
            return default_idx
        
        # 우선순위 3: 첫 번째 사용 가능한 입력 장치
        if input_devices:
            device_idx = input_devices[0]
            device_info = self.pa.get_device_info_by_host_api_device_index(0, device_idx)
            print(f"✅ 선택된 마이크: Index {device_idx} - {device_info['name']}")
            return device_idx
        
        # 입력 장치가 없는 경우
        print("❌ 사용 가능한 마이크 장치를 찾을 수 없습니다.")
        return None

    def loop(self):
        """
        Starts the main loop of the VoiceAgent.

        The loop continuously listens for audio input, converts it to text, generates a response, converts the response to speech,
        saves the speech as an audio file, and plays the audio file.

        Returns:
            None
        """
        while True:
            if self.check_break_signal():
                break

            self.listen()
            question = self.speech_to_text()
            print(f"Question: {question}")

            # 명령 모드 진입 또는 종료 키워드 체크
            if "명령 모드" in question:
                self.command_mode = True
                print(f"Command Mode Activated - Command_flag: {self.command_mode}")
                continue
            elif "기본 모드" in question:
                self.command_mode = False
                print(f"Default Mode Activated - Command_flag: {self.command_mode}")

            # 응답 생성
            answer = self.text_generation(question)
            print(f"Answer: {answer}")

            # (선택) 명령 모드일 경우 숫자만 추출
            if self.command_mode:
                try:
                    action_code = int(answer)
                    print(f"Action Code: {action_code} → ROS2로 전송 중...")
                    # self.publish_ros2_command(action_code)
                except ValueError:
                    print("명령 파싱 실패: 숫자 응답 아님")

            else : 
                audio_content = self.text_to_speech(answer)
                self.save_audio(audio_content)
                self.speak()

    @abc.abstractmethod
    def check_break_signal(self):
        """
        Checks for a break signal from the user.

        Returns:
            bool: True if a break signal is received, False otherwise.
        """
        # TODO: Implement a way to check for a break signal
        # Add your code here to check for a break signal
        # For example, you can use the keyboard module to check for keyboard input
        # If a break signal is received, return True
        # Otherwise, return False
        return False
    def listen(self, output_file="./media/audio.wav"):

        """
        Listens for audio input and saves it as a WAV file.

        Args:
            output_file (str, optional): The path to save the audio file. Defaults to "./media/audio.wav".

        Returns:
            None
        """
        got_a_sentence = False
        leave = False

        ring_buffer = collections.deque(maxlen=NUM_PADDING_CHUNKS)
        triggered = False
        ring_buffer_flags = [0] * NUM_WINDOW_CHUNKS
        ring_buffer_index = 0

        ring_buffer_flags_end = [0] * NUM_WINDOW_CHUNKS_END
        ring_buffer_index_end = 0
        # WangS
        raw_data = array("h")
        index = 0
        start_point = 0
        StartTime = time.time()
        print("* recording: ")
        self.stream.start_stream()

        # 음성 인식 민감도 파라미터 (조정 가능)
        # 인식이 잘 안될 때: amplitude_threshold 낮추기, min_consecutive_voiced 낮추기
        amplitude_threshold = 500  # 음량 임계값 (낮을수록 작은 목소리도 인식, 기본: 1000)
        min_consecutive_voiced = 3  # 최소 연속 발화 프레임 (낮을수록 빠른 시작, 기본: 5)
        consecutive_voiced = 0

        # 실제 샘플 레이트와 청크 크기 사용
        actual_rate = getattr(self, 'actual_rate', RATE)
        actual_chunk_size = getattr(self, 'actual_chunk_size', CHUNK_SIZE)
        vad_rate = getattr(self, 'vad_rate', 48000)
        needs_resample = getattr(self, 'needs_resample', False)
        
        while not got_a_sentence and not leave:
            chunk = self.stream.read(actual_chunk_size, exception_on_overflow=False)
            # add WangS
            samples = array("h", chunk)
            raw_data.extend(samples)
            index += actual_chunk_size
            # VAD + 음량 기반 이중 조건
            # 평균 절대값(RMS 유사) 계산으로 작은 잡음 억제
            if len(samples) > 0:
                avg_abs = sum(abs(s) for s in samples) / len(samples)
            else:
                avg_abs = 0
            
            # VAD를 위한 오디오 처리
            if needs_resample:
                # 리샘플링이 필요한 경우
                samples_np = np.array(samples, dtype=np.int16)
                num_samples = len(samples_np)
                # scipy.signal.resample 사용
                resampled = signal.resample(samples_np, int(num_samples * vad_rate / actual_rate))
                resampled = resampled.astype(np.int16)
                vad_chunk = resampled.tobytes()
            else:
                # 리샘플링이 필요 없는 경우
                vad_chunk = chunk
            
            vad_active = self.vad.is_speech(vad_chunk, vad_rate)  # VAD 지원 레이트 사용
            active = vad_active and (avg_abs >= amplitude_threshold)

            # 연속 발화 프레임 카운트
            if active:
                consecutive_voiced += 1
            else:
                consecutive_voiced = 0

            ring_buffer_flags[ring_buffer_index] = 1 if active else 0
            ring_buffer_index += 1

            ring_buffer_index %= NUM_WINDOW_CHUNKS

            ring_buffer_flags_end[ring_buffer_index_end] = 1 if active else 0
            ring_buffer_index_end += 1
            ring_buffer_index_end %= NUM_WINDOW_CHUNKS_END

            # start point detection
            if not triggered:
                ring_buffer.append(chunk)
                num_voiced = sum(ring_buffer_flags)
                # 시작 트리거 임계값 (낮을수록 쉽게 시작, 기본: 0.9)
                start_trigger_threshold = 0.7  # 70%로 낮춤 (인식 개선)
                if num_voiced > start_trigger_threshold * NUM_WINDOW_CHUNKS and consecutive_voiced >= min_consecutive_voiced:
                    sys.stdout.write(" Open ")
                    StartTime = time.time()
                    triggered = True
                    start_point = index - actual_chunk_size * 20  # start point
                    ring_buffer.clear()

            # end point detection
            else:
                ring_buffer.append(chunk)
                num_unvoiced = NUM_WINDOW_CHUNKS_END - sum(ring_buffer_flags_end)
                # 종료 트리거 임계값 (높을수록 더 오래 녹음, 기본: 0.90)
                end_trigger_threshold = 0.95  # 95%로 높임 (더 오래 녹음)
                max_recording_time = 10  # 최대 녹음 시간 (초)
                if (
                    num_unvoiced > end_trigger_threshold * NUM_WINDOW_CHUNKS_END
                    or (time.time() - StartTime) > max_recording_time
                ):
                    sys.stdout.write(" Close \n")
                    triggered = False
                    got_a_sentence = True

            sys.stdout.flush()

        self.stream.stop_stream()
        print("* done recording")
        got_a_sentence = False

        # write to file
        raw_data.reverse()
        for index in range(start_point):
            raw_data.pop()
        raw_data.reverse()
        raw_data = self.normalize(raw_data)
        # 실제 샘플 레이트 사용
        actual_rate = getattr(self, 'actual_rate', RATE)
        self.record_to_file(output_file, raw_data, 2, sample_rate=actual_rate)

    def manual_listen(self, duration=5, output_file="./media/audio.wav"):
        """
        Listens for audio input and saves it as a WAV file.

        Args:
            duration (int, optional): The duration to listen for in seconds. Defaults to 5.
            output_file (str, optional): The path to save the audio file. Defaults to "./media/audio.wav".

        Returns:
            None
        """
        stream = self.pa.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            start=False,
            frames_per_buffer=CHUNK_SIZE,
        )

        raw_data = array("h")
        print("* recording: ")
        stream.start_stream()

        for _ in range(int(RATE / CHUNK_SIZE * duration)):
            chunk = stream.read(CHUNK_SIZE, exception_on_overflow=False)
            raw_data.extend(array("h", chunk))

        stream.stop_stream()
        print("* done recording")

        raw_data = self.normalize(raw_data)
        self.record_to_file(output_file, raw_data, 2)

    def normalize(self, snd_data):
        "Average the volume out"
        MAXIMUM = 32767  # 16384
        times = float(MAXIMUM) / max(abs(i) for i in snd_data)
        r = array("h")
        for i in snd_data:
            r.append(int(i * times))
        return r

    def record_to_file(self, path, data, sample_width, sample_rate=None):
        """
        Records audio data to a file.

        Args:
            path (str): The path to save the audio file.
            data (list): The audio data.
            sample_width (int): The sample width of the audio data.
            sample_rate (int, optional): The sample rate. If None, uses RATE.

        Returns:
            None
        """
        if sample_rate is None:
            sample_rate = getattr(self, 'actual_rate', RATE)
        
        data = pack("<" + ("h" * len(data)), *data)
        wf = wave.open(path, "wb")
        wf.setnchannels(1)
        wf.setsampwidth(sample_width)
        wf.setframerate(sample_rate)  # 실제 샘플 레이트 사용
        wf.writeframes(data)
        wf.close()

    def speech_to_text(self, audio_file="./media/audio.wav"):
        """
        Converts an audio file to text using the OpenAI API.

        Args:
            audio_file (str, optional): The path to the audio file. Defaults to "./audio.wav".

        Returns:
            str: The transcribed text.
        """
        audio_file = open(audio_file, "rb")
        stt = self.client.audio.transcriptions.create(
            model="gpt-4o-mini-transcribe", file=audio_file
        )
        return stt.text

    def text_generation(self, question):
        """
        Generates a response to a user's question using the OpenAI API.

        Args:
            question (str): The user's question.

        Returns:
            str: The generated response.
        """
        persona = COMMAND_PERSONA if self.command_mode else DEFAULT_PERSONA
        response = self.client.chat.completions.create(
            model="gpt-5-nano",
            max_completion_tokens=600,
            messages=[
                {"role": "system", "content": persona},
                {"role": "user", "content": question},
            ],
        )

        return response.choices[0].message.content

    def text_to_speech(self, text):
        """
        Converts text to speech using the OpenAI API.

        Args:
            text (str): The text to convert to speech.

        Returns:
            bytes: The generated audio content.
        """
        speech_output = self.client.audio.speech.create(
            model="tts-1",
            voice="onyx",
            response_format="wav",
            input=text,
        )

        return speech_output.content

    def save_audio(self, content, audio_file="./media/output.wav"):
        """
        Saves the generated audio content to a file.

        Args:
            content (bytes): The audio content to save.
            audio_file (str, optional): The path to save the audio file. Defaults to "./output.wav".

        Returns:
            bool: True if the audio file was saved successfully, False otherwise.
        """
        if content:
            with open(audio_file, "wb") as f:
                f.write(content)
            return True
        else:
            return False

    def speak(self, audio_file="./media/output.wav"):
        """
        Plays the audio file.

        Args:
            audio_file (str, optional): The path to the audio file. Defaults to "./output.wav".

        Returns:
            None
        """

        # self.increase_volume_pydub(audio_file, audio_file, 10)
        Thread(target=self.increase_volume_pydub, args=(audio_file, audio_file, 10)).start()

        with self.sound_lock:
            threads = []
            for device in self.speaker_devices:
                # Start a thread for each device
                thread = Thread(
                    target=self.play_sound_on_device, args=(device, audio_file)
                )
                thread.start()
                threads.append(thread)

            # Wait for all threads to complete
            for thread in threads:
                thread.join()

    def increase_volume_pydub(self, wave_file_path, output_file_path, dB):
        sound = AudioSegment.from_wav(wave_file_path)
        louder_sound = sound + dB  # Increase volume by dB decibels
        louder_sound.export(output_file_path, format="wav")

    def play_sound_on_device(self, device, sound_file):
        command = ["aplay", "-D", device, sound_file]
        try:
            subprocess.run(command, check=True)
            print(f"✅ 스피커 재생 성공한 디바이스: {device}")
        except subprocess.CalledProcessError as e:
            print(f"Error playing sound on {device}: {e}")

    def publish_ros2_command(self, command: Union[int, dict]):
        if isinstance(command, int):
            # 기존 상태 전환 명령
            cmd = [
                "ros2", "service", "call",
                "/ensure_mode",
                "ghost_manager_interfaces/srv/EnsureMode",
                f"{{field: 'action', valdes: {command}}}"
            ]
            try:
                subprocess.run(cmd, check=True)
                print(f"[ACTION] 명령 전송 완료: {command}")
            except subprocess.CalledProcessError as e:
                print(f"[ERROR] 상태 전환 명령 실패: {e}")

        elif isinstance(command, dict):
            try:
                # 이전 퍼블리셔 종료 처리 추가!
                if self.active_twist_process:
                    print("[INFO] 이전 퍼블리시 프로세스 종료")
                    self.active_twist_process.terminate()
                    self.active_twist_process = None

                topic = command.get("topic")
                msg_type = command.get("msg_type")
                msg = command.get("msg", {})
                rate = str(command.get("rate", 10))  # 기본 10Hz

                linear = msg.get("linear", {})
                angular = msg.get("angular", {})

                # Twist 메시지를 문자열로 YAML 포맷 구성
                twist_msg = (
                    f"{{linear: {{x: {linear.get('x', 0.0)}, y: {linear.get('y', 0.0)}, z: {linear.get('z', 0.0)}}}, "
                    f"angular: {{x: {angular.get('x', 0.0)}, y: {angular.get('y', 0.0)}, z: {angular.get('z', 0.0)}}}}}"
                )

                cmd = [
                    "ros2", "topic", "pub", "--rate", rate,
                    topic,
                    msg_type,
                    twist_msg
                ]

                print(f"[TWIST-CLI] 실행 명령: {' '.join(cmd)}")
                self.active_twist_process = subprocess.Popen(cmd)

            except Exception as e:
                print(f"[ERROR] CLI Twist 퍼블리시 실패: {e}")

        else:
            print("[ERROR] 명령 형식이 올바르지 않습니다. (int 또는 dict)")

# gstreamer

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mic_index", type=int, default=0, help="Microphone index")
    args = parser.parse_args()

    print("mic_index:", args.mic_index)

    va = VoiceAgent(mic_index=args.mic_index)
    va.loop()

