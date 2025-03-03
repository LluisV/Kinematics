
# Kinematics Library for Arduino/ESP32

## Overview

This library provides a set of tools to model and compute the kinematics of robotic systems using Denavit-Hartenberg (DH) parameters. The library supports both forward and inverse kinematics calculations and can be used with microcontrollers like Arduino and ESP32.

It includes methods for defining robot joints, setting joint parameters, calculating transformations, and handling kinematic calculations with ease. The library is built to be compatible with the `ArduinoEigen` C++ library, providing high-performance matrix operations.

## Features

- **DH Parameterization**: Set and manage Denavit-Hartenberg parameters for revolute and prismatic joints.
- **Forward Kinematics**: Compute the transformation matrix for the end-effector position and orientation.
- **Inverse Kinematics**: Calculate joint angles (or positions) to achieve a desired end-effector pose using an iterative method based on the pseudo-inverse of the Jacobian.
- **Joint Positioning**: Set joint positions within the specified limits and retrieve current joint positions.
- **Jacobian Calculation**: Compute the geometric Jacobian matrix to understand the relationship between joint velocities and end-effector velocity.
- **End-Effector Pose**: Retrieve the position and orientation of the robot's end-effector.

## Requirements

- Arduino or ESP32 development environment (ArduinoIDE, PlatformIO, ...)
- ArduinoEigen C++ library for matrix operations 
- C++11 or later compiler support

## Installation

1. TODO

## Example Usage


```cpp
#include "DH.h"
#include "Kinematics.h"
#include "RoboticsUtils.h"

DH scara(3);

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  // Define the DH parameters of the robot
  scara.setJointParameters(0, 0.0, 150, 0.0, 0.0, JointType::PRISMATIC);
  scara.setJointParameters(1, 0.0, 0.0, 200, 0.0,  JointType::REVOLUTE);
  scara.setJointParameters(2, 0.0, 0.0, 200, 2*M_PI,  JointType::REVOLUTE);

  scara.setJointLimits(0, 0.0, 300);  // Z limit
  scara.setJointLimits(1, -1.5*M_PI, 1.5*M_PI);  // Q0 limit
  scara.setJointLimits(2, -1.5*M_PI, 1.5*M_PI);  // Q1 limit

  scara.setJointPosition(0, 150);
  scara.setJointPosition(1, 0);
  scara.setJointPosition(2, 0);

  // Forward kinematics
  Serial.println("--------------Forward Kinematics--------------");
  Eigen::Matrix4f fk_result = forward_kinematics(scara);
  RoboticsUtils::print_matrix(fk_result);
  
  // EF position
  Serial.println("--------------End effector position--------------");
  Eigen::VectorXf ef_pos = scara.getEndEffectorPose();
  RoboticsUtils::print_vector(ef_pos);

  // Jacobian
  Serial.println("--------------Jacobian--------------");
  Eigen::Matrix<float, 6, -1> jacobian = scara.computeJacobian();
  RoboticsUtils::print_matrix(jacobian);

  Serial.println("--------------End effector velocity--------------");
  Eigen::VectorXf jointVelocities(3);
          jointVelocities(0) = 10;  // Z
          jointVelocities(1) = 10;  // X
          jointVelocities(2) = 10;  // Y  
  Eigen::VectorXf ef_vel = scara.computeEndEffectorVelocity(jointVelocities);
  RoboticsUtils::print_vector(ef_vel);

  // Inverse kinematics
  Serial.println("--------------Inverse kinematics--------------");
  Eigen::Matrix4f desiredTransform; 
  desiredTransform.setIdentity();
  desiredTransform(0, 3) = 200;  // X
  desiredTransform(1, 3) = 200;  // Y
  desiredTransform(2, 3) = 150; // Z

  Eigen::VectorXf ik_solution(3);
  bool res = inverse_kinematics(scara, desiredTransform, ik_solution, 0.5, 1000);

  if (res) {
    Serial.print("Solucion IK: ");
    RoboticsUtils::print_vector(ik_solution); 
  } else {
      Serial.println("IK solution not found.");
  }
}

```

## Functions

### DH Class

- `setJointParameters(int jointIndex, double theta, double d, double a, double alpha, JointType jointType)`: Set the DH parameters for a specific joint.
- `setJointPosition(int jointIndex, double position)`: Set the position of a joint.
- `getJointPosition(int jointIndex)`: Get the current position of a joint.
- `getTransform(int endJointIndex)`: Get the overall transformation matrix from the base to a specific joint.

### Kinematics

- `forward_kinematics(const DH& robot)`: Compute the forward kinematics and return the transformation matrix.
- `inverse_kinematics(DH& robot, const Eigen::Matrix4f& desiredTransform, Eigen::VectorXf& solution)`: Calculate the joint positions for a desired end-effector pose.

### RoboticsUtils

- `wrap_angle(double angle)`: Wraps an angle to the range [-π, π].
- `print_matrix(const Eigen::MatrixBase<Derived>& mat)`: Print an Eigen matrix to the serial monitor.
- `print_vector(const Eigen::MatrixBase<Derived>& vec)`: Print an Eigen vector to the serial monitor.

## Notes

- This library is designed to run efficiently on microcontrollers like Arduino and ESP32, but it may also work on other platforms that support C++11 or later and the ArduinoEigen library.
- Ensure your microcontroller has enough resources (memory and processing power) to handle the calculations if you're using a large number of joints or complex kinematic chains.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
