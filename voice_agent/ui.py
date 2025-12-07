from PyQt5 import QtWidgets
from voice_agent.voice_agent import VoiceAgent

import time
import signal


class voiceUI:
    def __init__(self) -> None:
        self.va = VoiceAgent()

        self.make_app()
        self.make_window()
        self.make_ui()

        signal.signal(signal.SIGINT, signal.SIG_DFL)  # Shutdown when putting "ctrl + c"

    def execute(self):
        self.window.show()
        self.app.exec_()

    def make_app(self):
        # Create the main application
        self.app = QtWidgets.QApplication([])

    def make_window(self):
        self.window = QtWidgets.QWidget()
        self.window.setWindowTitle("Voice Agent")
        self.window.resize(800, 600)

    def make_ui(self):
        # Create the recording button
        self.record_button = QtWidgets.QPushButton("Start Recording")
        self.record_button.clicked.connect(self.recording)

        # Create the speak button
        self.speak_button = QtWidgets.QPushButton("Speak")
        self.speak_button.clicked.connect(self.speak)

        # Make logging window
        self.log_window = QtWidgets.QTextEdit()

        # Create a vertical layout for the buttons
        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(self.record_button)
        self.layout.addWidget(self.speak_button)
        self.layout.addWidget(self.log_window)

        # Set the layout for the main window
        self.window.setLayout(self.layout)

    def recording(self):
        # Code to start recording audio
        if self.record_button.text() == "Start Recording":
            self.log_window.clear()

            self.record_button.setText("Recording...")
            QtWidgets.QApplication.processEvents()

            self.va.listen()

            self.record_button.setText("Start understanding...")
            QtWidgets.QApplication.processEvents()

            question = self.va.speech_to_text()
            # 명령 모드 진입 또는 종료 키워드 체크
            if "명령 모드" in question:
                self.va.command_mode = True
            elif "기본 모드" in question:
                self.va.command_mode = False

            self.log_window.append("Question: " + question + "\n")
            QtWidgets.QApplication.processEvents()

            answer = self.va.text_generation(question)
            command_flag = str(self.va.command_mode)
            self.log_window.append("Answer: " + answer + "\n")
            self.log_window.append("Command Mode Flag: " + command_flag + "\n")
            print(f"Answer: {answer}, Type : {type(answer)}")

            if self.va.command_mode:
                try:
                    action_code = int(answer)
                    self.log_window.append(f"Action Code: {action_code} → ROS2로 전송 중...\n")
                    self.va.publish_ros2_command(action_code)
                except ValueError:
                    self.log_window.append("명령 파싱 실패: 숫자 응답 아님\n")

            QtWidgets.QApplication.processEvents()

            audio_content = self.va.text_to_speech(answer)
            self.va.save_audio(audio_content)

            self.record_button.setText("Finished understanding")
            QtWidgets.QApplication.processEvents()

            time.sleep(2)
            self.record_button.setText("Start Recording")
            QtWidgets.QApplication.processEvents()


    def speak(self):
        # Code to process and speak the recorded audio
        self.va.speak()
        pass

def main(args=None):
    voice_ui = voiceUI()
    voice_ui.execute()


if __name__ == "__main__":
    main()