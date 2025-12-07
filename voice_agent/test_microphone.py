#!/usr/bin/env python3
"""
마이크 장치 선택 및 실시간 루프백 디버깅 스크립트
마이크로 입력받은 오디오를 실시간으로 스피커로 출력하여 디버깅합니다.
"""

import pyaudio
import time
import sys


def list_devices(p):
    """연결된 오디오 장치 목록 표시"""
    print("=" * 70)
    print(" [연결된 오디오 장치 목록] ")
    print("=" * 70)
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    found_devices = []
    
    for i in range(0, numdevices):
        device_info = p.get_device_info_by_host_api_device_index(0, i)
        if device_info.get('maxInputChannels', 0) > 0:
            name = device_info.get('name', 'Unknown')
            rate = int(device_info.get('defaultSampleRate', 0))
            print(f"Index {i}: {name} (Default Rate: {rate}Hz)")
            found_devices.append(i)
    print("=" * 70)
    return found_devices


def audio_callback(in_data, frame_count, time_info, status):
    """
    오디오 처리를 담당할 콜백 함수
    입력받은 데이터(in_data)를 그대로 출력으로 내보냄 (Loopback)
    """
    return (in_data, pyaudio.paContinue)


def run_loopback_optimized():
    """최적화된 루프백 모드 실행"""
    # 끊김 방지를 위해 버퍼 크기를 넉넉하게 잡음
    CHUNK = 4096
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    
    p = pyaudio.PyAudio()
    valid_indices = list_devices(p)
    
    if not valid_indices:
        print("❌ 사용 가능한 입력 장치가 없습니다.")
        p.terminate()
        return
    
    try:
        user_input = input("\nIndex 번호를 입력하세요 (추천: 'pulse'나 'default'에 해당하는 번호, 취소: q): ").strip()
        
        if user_input.lower() == 'q':
            print("선택이 취소되었습니다.")
            p.terminate()
            return
        
        input_index = int(user_input)
        
        # 입력 장치 목록에 있는지 확인
        if input_index not in valid_indices:
            print(f"⚠️  [{input_index}]는 입력 장치가 아닙니다.")
            p.terminate()
            return
            
    except ValueError:
        print("⚠️  잘못된 입력입니다. 숫자를 입력하세요.")
        p.terminate()
        return
    except KeyboardInterrupt:
        print("\n선택이 취소되었습니다.")
        p.terminate()
        return

    # 장치 정보 확인
    dev_info = p.get_device_info_by_index(input_index)
    RATE = int(dev_info.get('defaultSampleRate', 48000))
    
    print(f"\n[INFO] 선택 장치: {dev_info.get('name', 'Unknown')}")
    print(f"[INFO] 샘플링 레이트: {RATE}Hz, 버퍼 크기: {CHUNK}")
    print("[INFO] 실시간 루프백 실행 중... (종료: Ctrl+C)")
    print("[INFO] 마이크로 말하면 스피커로 바로 들립니다.\n")

    stream = None
    try:
        # 스트림을 '콜백 모드'로 엽니다 (Non-blocking)
        stream = p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            output=True,
            input_device_index=input_index,
            frames_per_buffer=CHUNK,
            stream_callback=audio_callback  # 여기가 핵심
        )

        stream.start_stream()

        # 메인 스레드는 스트림이 살아있는 동안 대기만 함 (CPU 부하 감소)
        while stream.is_active():
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[INFO] 중지 요청됨.")
    except Exception as e:
        print(f"[ERROR] {e}")
        print("💡 팁: 하드웨어 직접 연결(hw:X,Y)보다는 'default'나 'pulse' 장치를 선택해보세요.")
        import traceback
        traceback.print_exc()

    finally:
        print("[INFO] 종료합니다.")
        if stream is not None:
            stream.stop_stream()
            stream.close()
        p.terminate()


def main():
    """메인 함수"""
    print("\n" + "=" * 70)
    print("🎤 Voice Agent 마이크 루프백 디버깅 스크립트")
    print("=" * 70)
    print("\n이 스크립트는 마이크로 입력받은 오디오를 실시간으로 스피커로 출력합니다.")
    print("마이크와 스피커가 정상적으로 작동하는지 확인하는 데 사용합니다.")
    print("\n💡 사용법:")
    print("   1. 장치 목록에서 마이크 장치의 Index 번호를 확인합니다.")
    print("   2. Index 번호를 입력하면 실시간 루프백이 시작됩니다.")
    print("   3. 마이크로 말하면 스피커로 바로 들립니다.")
    print("   4. Ctrl+C로 종료합니다.\n")
    
    try:
        run_loopback_optimized()
    except KeyboardInterrupt:
        print("\n\n👋 사용자에 의해 종료되었습니다.")
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
