# Author: Ashutosh Singh

import rclpy
import sys
from rclpy.node import Node
import json
from vosk import Model, KaldiRecognizer
import pyaudio

from geometry_msgs.msg import Twist

class speech2text(Node):
    def __init__(self):
        super().__init__('VoiceController')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        model_path = "/home/singh/robot_ws/src/voiceRecognition/vosk-model-small-en-us-0.15" 
        
        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)
        mic = pyaudio.PyAudio()
        self.stream = mic.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)
        self.stream.start_stream()

        self.cmd_string = ""
        self.prev_cmd = ""

        processing_rate = 0.1
        self.timer = self.create_timer(processing_rate,self.voice_cmd)

    def voice_cmd(self):
        data = self.stream.read(4096, exception_on_overflow=False)
        if self.recognizer.AcceptWaveform(data):
            result = json.loads(self.recognizer.Result())
            self.cmd_string = result.get("text", "").strip()
            # print(self.cmd_string)

            if not self.cmd_string:
                self.cmd_string = self.prev_cmd

            if self.cmd_string:
                self.get_logger().info(f"Recognized Command: {self.cmd_string}")
                self.execute_command(self.cmd_string)

            self.prev_cmd = self.cmd_string 

    def execute_command(self, command):
        twist = Twist()

        movement_commands = {
            "left": (0.0, 1.0),
            "right": (0.0, -1.0),
            "forward": (1.0, 0.0),
            "backward": (-1.0, 0.0),
            "stop": (0.0, 0.0)
        }

        if command in movement_commands:
            twist.linear.x, twist.angular.z = movement_commands[command]
            self.publisher.publish(twist)
            self.get_logger().info(f"Executed: {command}")

def main():
    
    rclpy.init(args=sys.argv)

    node = speech2text() 

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
