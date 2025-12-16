---
sidebar_position: 3
---

# Practical Implementation: Bridging Python AI Agents to ROS 2 Controllers

This chapter demonstrates how to create bridges between Python-based AI agents and ROS 2 controllers using the rclpy library.

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement Python AI agents that communicate with ROS 2
- Create bridges between AI decision-making and ROS 2 control systems
- Design message passing between AI agents and robot controllers
- Handle real-time communication requirements for humanoid robots

## AI-Agent to ROS 2 Bridge Architecture

The bridge between AI agents and ROS 2 controllers is critical for humanoid robotics applications. It enables high-level decision making by AI systems to be translated into low-level control commands for the robot.

## Setting Up the Bridge

First, let's establish the basic structure for our AI-ROS bridge:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge')

        # Publishers for sending commands to the robot
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscribers for receiving sensor data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Store current joint states
        self.current_joint_states = JointState()

        # Timer for AI decision making
        self.timer = self.create_timer(0.1, self.ai_decision_callback)

        self.get_logger().info('AI Bridge Node Initialized')

    def joint_state_callback(self, msg):
        self.current_joint_states = msg
        # Process joint states for AI agent
        self.update_ai_agent_state(msg)

    def ai_decision_callback(self):
        # This is where AI logic would run
        ai_decision = self.run_ai_decision_process()
        if ai_decision:
            self.send_command_to_robot(ai_decision)
```

## Implementing AI Decision Logic

The AI agent needs to process sensor data and make decisions:

```python
def update_ai_agent_state(self, joint_states):
    # Update internal AI agent state with sensor data
    self.ai_agent.current_joint_positions = joint_states.position
    self.ai_agent.current_joint_velocities = joint_states.velocity

def run_ai_decision_process(self):
    # Example: Simple balance controller using AI
    # In a real implementation, this could be a neural network or planning algorithm
    if self.should_adjust_balance():
        return self.calculate_balance_adjustment()
    return None

def send_command_to_robot(self, command):
    # Convert AI decision to ROS 2 message
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = command.joint_names

    point = JointTrajectoryPoint()
    point.positions = command.positions
    point.velocities = command.velocities
    point.time_from_start.sec = 1
    point.time_from_start.nanosec = 0

    trajectory_msg.points = [point]

    self.joint_trajectory_pub.publish(trajectory_msg)
```

## Handling Real-Time Constraints

Humanoid robots require strict timing for balance and coordination:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Configure QoS for real-time performance
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

self.joint_trajectory_pub = self.create_publisher(
    JointTrajectory,
    '/joint_trajectory_controller/joint_trajectory',
    qos_profile
)
```

## Practical Example: Walking Pattern Generation

Here's a complete example of an AI agent generating walking patterns:

```python
class WalkingPatternGenerator:
    def __init__(self):
        self.step_phase = 0.0
        self.step_frequency = 0.5  # steps per second

    def generate_step_pattern(self, current_joint_states):
        # Generate walking pattern based on current state
        target_positions = []

        # Simple walking pattern for humanoid
        for i, joint_name in enumerate(current_joint_states.name):
            if 'hip' in joint_name:
                # Hip movement for walking
                target_positions.append(
                    current_joint_states.position[i] +
                    0.1 * math.sin(self.step_phase)
                )
            elif 'knee' in joint_name:
                # Knee movement synchronized with hip
                target_positions.append(
                    current_joint_states.position[i] +
                    0.05 * math.cos(self.step_phase)
                )
            else:
                # Default: maintain current position
                target_positions.append(current_joint_states.position[i])

        self.step_phase += 0.1  # Increment phase
        if self.step_phase > 2 * math.pi:
            self.step_phase = 0.0

        return target_positions
```

## Error Handling and Safety

Safety is paramount in humanoid robotics:

```python
def send_command_to_robot(self, command):
    # Safety checks before sending commands
    if not self.is_command_safe(command):
        self.get_logger().error('Unsafe command detected, not executing')
        return

    # Send command with safety monitoring
    trajectory_msg = self.create_trajectory_message(command)
    self.joint_trajectory_pub.publish(trajectory_msg)

    # Monitor execution
    self.start_execution_monitoring()

def is_command_safe(self, command):
    # Check if joint positions are within safe limits
    for pos in command.positions:
        if abs(pos) > self.max_safe_joint_position:
            return False
    return True
```

## Integration with Humanoid Robot Systems

The bridge should integrate with existing humanoid robot frameworks like ROS 2 Control and MoveIt2 for advanced manipulation and navigation capabilities.