---
sidebar_position: 4
---

# Anatomy of a Humanoid: Understanding and Creating URDF Files for Bipedal Robots

This chapter covers the creation and understanding of URDF (Unified Robot Description Format) files specifically for bipedal humanoid robots.

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the structure and components of URDF files
- Create URDF files for bipedal humanoid robots
- Define joints, links, and physical properties for humanoid robots
- Integrate URDF files with ROS 2 simulation environments

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML format used to describe robots in ROS. For humanoid robots, URDF files define:

- Physical structure (links)
- Joints connecting different parts
- Kinematic properties
- Visual and collision models

## Basic URDF Structure

A basic URDF file for a humanoid robot follows this structure:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Define materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <!-- Define the base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Define joints connecting links -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0.0 0.0 0.1"/>
  </joint>

  <!-- Define additional links -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

## Humanoid-Specific Considerations

Humanoid robots have unique requirements that must be considered in URDF:

### Joint Types for Humanoid Robots

- **Revolute joints**: For rotating joints like knees and elbows
- **Continuous joints**: For joints that can rotate indefinitely like shoulders
- **Fixed joints**: For non-moving connections

```xml
<!-- Hip joint example -->
<joint name="left_hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="left_thigh"/>
  <origin xyz="0.0 -0.1 -0.25" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
  <dynamics damping="1.0" friction="0.1"/>
</joint>
```

### Center of Mass Considerations

For stable humanoid locomotion, the center of mass must be carefully modeled:

```xml
<!-- Proper inertial properties for balance -->
<inertial>
  <mass value="0.5"/>
  <origin xyz="0 0 0"/>
  <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
</inertial>
```

## Complete Humanoid URDF Example

Here's a more complete example for a simple bipedal humanoid:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Materials -->
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Torso (base) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.35"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="2"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="0.0 -0.1 -0.25"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="3"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <geometry>
        <capsule radius="0.05" length="0.3"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <capsule radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Additional joints and links would continue for knees, ankles, arms, etc. -->
</robot>
```

## URDF Best Practices for Humanoid Robots

1. **Proper Mass Distribution**: Ensure realistic mass properties for stable simulation
2. **Joint Limits**: Set appropriate limits to prevent impossible poses
3. **Collision Avoidance**: Define collision properties to prevent self-collision
4. **Kinematic Chains**: Structure the robot with proper parent-child relationships

## Integration with ROS 2

URDF files can be loaded into ROS 2 using the robot_state_publisher:

```xml
<!-- Launch file example -->
<launch>
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(var robot_description_file)"/>
  </node>

  <node pkg="joint_state_publisher" exec="joint_state_publisher" name="joint_state_publisher"/>
</launch>
```

## Validation and Testing

Always validate your URDF files:

- Check for proper kinematic chains
- Verify mass properties are realistic
- Test in simulation before real robot deployment
- Use tools like `check_urdf` to validate syntax