# Quickstart Guide: Vision-Language-Action (VLA) System

**Module**: Module 4: Vision-Language-Action (VLA)
**Date**: 2025-12-16

## Overview

This guide will help you set up and run the Vision-Language-Action (VLA) system for humanoid robotics. The system integrates voice processing, cognitive planning with LLMs, and ROS 2 action execution.

## Prerequisites

- Python 3.8 or higher
- ROS 2 (Humble Hawksbill or later recommended)
- OpenAI API key (for Whisper and LLM services)
- NVIDIA Isaac ROS packages (if using Isaac ROS for vision)
- Nav2 (Navigation2) packages for navigation

## Installation

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Create a virtual environment:**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install Python dependencies:**
   ```bash
   pip install openai torch torchaudio
   pip install rclpy  # ROS 2 Python client library
   ```

4. **Set up OpenAI API key:**
   ```bash
   export OPENAI_API_KEY="your-api-key-here"
   ```

## Basic Usage

### 1. Running Voice-to-Action Processing

```python
from vla.voice.whisper_integration import WhisperProcessor

# Initialize the voice processor
processor = WhisperProcessor()

# Process audio input
transcript = processor.transcribe_audio("path/to/audio.wav")
print(f"Transcribed: {transcript}")
```

### 2. Cognitive Planning with LLM

```python
from vla.planning.llm_planner import LLMPlanner

# Initialize the planner
planner = LLMPlanner()

# Generate action sequence from high-level command
command = "Clean the room"
action_sequence = planner.generate_plan(command)
print(f"Action sequence: {action_sequence}")
```

### 3. Complete VLA Pipeline

```python
from vla.capstone.vla_pipeline import VLAPipeline

# Initialize the complete pipeline
vla_system = VLAPipeline()

# Run the full pipeline: Voice -> Plan -> Action
result = vla_system.process_command("Pick up the red cup from the table")
print(f"Execution result: {result}")
```

## Example: "Clean the Room" Implementation

The following example demonstrates how the system processes the command "Clean the room":

1. **Voice Processing**: Whisper converts speech to text
2. **Cognitive Planning**: LLM breaks down "Clean the room" into:
   - Map the room
   - Identify objects that need to be moved
   - Navigate to each object
   - Pick up each object
   - Place objects in appropriate locations
3. **Action Execution**: Each step is converted to ROS 2 actions

## Running the Complete System

1. **Start ROS 2:**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 launch your_robot_bringup.launch.py
   ```

2. **Run the VLA system:**
   ```bash
   python -m vla.main
   ```

3. **Issue a voice command** through the system interface.

## Troubleshooting

- **Audio input issues**: Ensure microphone permissions and proper audio format (WAV, MP3, etc.)
- **API rate limits**: Implement retry logic with exponential backoff for OpenAI API calls
- **ROS 2 connection issues**: Verify ROS 2 network configuration and topic availability
- **Performance issues**: Consider using local Whisper models for offline processing

## Next Steps

- Explore advanced voice processing techniques
- Implement custom LLM prompts for specific robot capabilities
- Integrate with your specific robot hardware
- Add error recovery mechanisms
- Extend the action vocabulary for your robot's capabilities