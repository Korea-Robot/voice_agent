import os
import yaml
import time
import sys
import json
import threading

import rclpy
from rclpy.node import Node

import sys
import os
from voice_agent.voice_agent import VoiceAgent
from geometry_msgs.msg import Twist
from ghost_manager_interfaces.srv import EnsureMode


class VoiceAgentNode(Node):
    def __init__(self, node_name="voice_agent_node"):
        super().__init__(node_name)
        self.voice_agent = None
        self._twist_timer = None
        self._twist_tick_count = 0
        self.motion_thread = None
        self.motion_lock = threading.Lock()
        self._stop_motion = threading.Event()
        self.twist_cmd_pub = self.create_publisher(Twist, "/mcu/command/manual_twist", 10)
        self.ensure_mode_client = self.create_client(EnsureMode, "/ensure_mode")
        
        # VoiceAgent 초기화
        try:
            self.voice_agent = VoiceAgent()
            self.get_logger().info("VoiceAgent 초기화 완료")
        except Exception as e:
            self.get_logger().error(f"[INIT ERROR] VoiceAgent 초기화 실패: {e}")
            raise e

    def run(self):
        self.get_logger().info("Voice Agent 시작")

        while rclpy.ok():
            # 음성 인식 → 응답 생성
            self.voice_agent.listen()
            question = self.voice_agent.speech_to_text()
            self.get_logger().info(f"Question : {question}")

            # 명령 모드 진입 또는 종료 키워드 체크
            if "명령 모드" in question:
                self.voice_agent.command_mode = True
                self.get_logger().info(f"Command Mode Activated - Command_flag: {self.voice_agent.command_mode}")

                # 180번 Control에서 140번 Control로 자동 변환
                self.call_ensure_mode("control_mode", 140)
                self.get_logger().info(f"Command Heartbeat Number : 140")

                answer = "명령 모드"
                audio_content = self.voice_agent.text_to_speech(answer)
                self.voice_agent.save_audio(audio_content)
                self.voice_agent.speak()
                continue

            elif "기본 모드" in question:
                self.voice_agent.command_mode = False
                self.get_logger().info(f"Default Mode Activated - Command_flag: {self.voice_agent.command_mode}")

                # 140번 Control에서 180번 Control로 자동 변환
                self.call_ensure_mode("control_mode", 180)
                self.get_logger().info(f"Command Heartbeat Number : 180")

                answer = "기본 모드"
                audio_content = self.voice_agent.text_to_speech(answer)
                self.voice_agent.save_audio(audio_content)
                self.voice_agent.speak()
                continue

            elif "종료" in question:
                self.get_logger().info("Exit Command Received - Interactive Mode Turn Off")
                break

            elif "멈춰" in question or "정지" in question:
                self.get_logger().info("🛑 정지 명령 수신됨. 현재 동작 중단.")
                self._stop_motion.set()
                continue

            answer = self.voice_agent.text_generation(question)
            self.get_logger().info(f"Answer : {answer}")

            # 명령 모드 처리 여부
            if self.voice_agent.command_mode:
                try:
                    # 1. 정수 명령인지 확인
                    if isinstance(answer, int) or (isinstance(answer, str) and answer.strip() in {"-1", "0", "1", "2"}):
                        action_code = int(answer)
                        self.call_ensure_mode("action", action_code)
                        self.get_logger().info(f"action code : {action_code}")

                    # 2. 문자열인데 List처럼 생겼으면 → 파싱 시도
                    elif isinstance(answer, str) and answer.strip().startswith("["):
                        twist_command = json.loads(answer)
                        self.get_logger().info(f"Twist dict로 변환 완료 : {twist_command}")
                        self.publish_twist_command(twist_command)
                        self.get_logger().info("Twist 명령 실행")

                    else:
                        raise ValueError("지원되지 않는 명령 형식")

                except Exception as e:
                    self.get_logger().error(f"[PARSE ERROR] {e}")
            else:
                audio_content = self.voice_agent.text_to_speech(answer)
                self.voice_agent.save_audio(audio_content)
                self.voice_agent.speak()
    
    def call_ensure_mode(self, field: str, valdes: int):
        if not self.ensure_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("ensure_mode 서비스가 사용 불가능합니다.")
            return

        request = EnsureMode.Request()
        request.field = field
        request.valdes = valdes  # valdes는 float[] 타입이어야 함

        future = self.ensure_mode_client.call_async(request)

        def callback(fut):
            try:
                result = fut.result()
                if result.success:
                    self.get_logger().info(f"서비스 응답 성공: {result.result_str}")
                else:
                    self.get_logger().warn(f"서비스 응답 실패: {result.result_str}")
            except Exception as e:
                self.get_logger().error(f"서비스 호출 예외 발생: {e}")

        future.add_done_callback(callback)

    
    def start_twist_thread(self, twist_command: list):
        def run_twist():
            vx, vy, vyaw, duration = twist_command
            rate = 30
            total_ticks = int(rate * duration)

            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = vyaw

            self.get_logger().info(f"🌀 Twist 스레드 실행: {vx=}, {vy=}, {vyaw=}, {duration=}")

            for i in range(total_ticks):
                if not rclpy.ok() or self._stop_motion.is_set():
                    self.get_logger().info("🛑 Twist 스레드 중단 요청 수신됨")
                    break
                self.twist_cmd_pub.publish(msg)
                self.get_logger().info(f"🔁 [{i+1}/{total_ticks}] Twist 발행: {msg}")
                time.sleep(1.0 / rate)

            self.get_logger().info("✅ Twist 스레드 완료")

        # 이전 동작이 있다면 중지
        with self.motion_lock:
            if self.motion_thread and self.motion_thread.is_alive():
                self.get_logger().info("🛑 이전 동작 중지 요청")
                self.motion_thread.join(timeout=0.1)

            # 새로운 동작 스레드 시작
            self._stop_motion.clear()
            self.motion_thread = threading.Thread(target=run_twist, daemon=True)
            self.motion_thread.start()

    # 기존 publish_twist_command 함수 수정
    def publish_twist_command(self, twist_command: list):
        if not isinstance(twist_command, list) or len(twist_command) != 4:
            self.get_logger().error("⚠️ Twist 명령은 [vx, vy, vyaw, duration] 형식의 리스트여야 합니다.")
            return

        self.start_twist_thread(twist_command)



def main(args=None):
    rclpy.init(args=args)
    voice_agent_node = VoiceAgentNode()
    
    try:
        voice_agent_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        voice_agent_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()