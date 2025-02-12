# Approach Explanation for My Voice-Controlled Robot Navigation in ROS 2

This project integrates speech recognition with robot navigation in ROS 2 using Vosk for speech-to-text conversion. The goal is to process voice commands and translate them into movement instructions for the robot.
1. Voice Recognition Setup

    I am using Vosk’s offline speech recognition model (vosk-model-small-en-us-0.15) for processing spoken commands.
    The KaldiRecognizer from Vosk converts the audio into text.
    PyAudio captures real-time audio from the microphone:
        It streams mono-channel audio at 16 kHz.
        Captures 4096-byte chunks at a time and feeds them to Vosk for processing.

2. ROS 2 Node Structure

    The speech2text class inherits from rclpy.node.Node, making it a ROS 2 node.
    The node:
        Publishes velocity commands (Twist messages) to the /cmd_vel topic.
        Uses a ROS 2 timer to check for voice input every 0.1 seconds.

3. Speech-to-Text Processing

    The function voice_cmd():
        Reads audio data from the microphone.
        Uses recognizer.AcceptWaveform(data) to process audio input.
        Extracts the recognized text command from Vosk’s JSON output.
        If no command is detected, it repeats the last recognized command.
        Logs the recognized command for debugging.

4. Command Execution & Robot Control

    The method execute_command(command):
        Uses a dictionary-based mapping of commands to linear/angular velocities:
            "left" → Turns left (angular.z = 1.0)
            "right" → Turns right (angular.z = -1.0)
            "forward" → Moves forward (linear.x = 1.0)
            "backward" → Moves backward (linear.x = -1.0)
            "stop" → Stops the robot (linear.x = 0.0, angular.z = 0.0)
        Publishes the computed Twist message to the /cmd_vel topic.
        Logs the executed command.
