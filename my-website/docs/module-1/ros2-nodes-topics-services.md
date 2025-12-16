---
sidebar_position: 2
---

# Deep Dive: ROS 2 Nodes, Topics, and Services

This chapter provides a comprehensive exploration of ROS 2 communication patterns, including nodes, topics, and services, with practical examples.

## Learning Objectives

By the end of this chapter, you will be able to:
- Create and manage ROS 2 nodes for humanoid robotics applications
- Implement publisher-subscriber communication using topics
- Design and use service-based communication
- Understand Quality of Service (QoS) settings for humanoid robot systems

## ROS 2 Architecture Overview

ROS 2 follows a distributed architecture where different components (nodes) communicate through a publish-subscribe model and service-based interactions. This architecture is particularly important for humanoid robots due to their complex multi-sensor, multi-actuator nature.

## Nodes

A node is a process that performs computation. In ROS 2, nodes are the basic building blocks of a robotic system:

```python
import rclpy
from rclpy.node import Node

class HumanoidControllerNode(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.get_logger().info('Humanoid Controller Node Started')
```

## Topics and Publishers/Subscribers

Topics enable asynchronous communication between nodes using a publish-subscribe pattern:

```python
# Publisher example
from std_msgs.msg import String
publisher = self.create_publisher(String, 'robot_status', 10)

# Subscriber example
def status_callback(self, msg):
    self.get_logger().info(f'Received status: {msg.data}')

subscriber = self.create_subscription(
    String,
    'robot_status',
    self.status_callback,
    10
)
```

## Services

Services provide synchronous request-response communication:

```python
from example_interfaces.srv import AddTwoInts

def add_service_callback(self, request, response):
    response.sum = request.a + request.b
    return response

service = self.create_service(
    AddTwoInts,
    'add_two_ints',
    self.add_service_callback
)
```

## Quality of Service (QoS) for Humanoid Robots

QoS settings are crucial for humanoid robots that require reliable, real-time communication:

- **Reliability**: Ensuring critical messages (like balance data) are delivered
- **Durability**: Keeping important configuration data persistent
- **History**: Managing how many messages to store
- **Deadline**: Setting time constraints for message delivery

## Practical Implementation for Humanoid Robots

Humanoid robots require specialized communication patterns:

- Joint state topics for coordination between different body parts
- Sensor fusion topics combining data from multiple sensors
- Safety service calls for emergency stop functionality
- Behavior trees using services for decision making