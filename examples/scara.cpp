#include <Arduino.h>
#include "DH.h"
#include "Kinematics.h"
#include "RoboticsUtils.h"

DH scara(3);

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  // Define the DH parameters of the robot
  scara.setJointParameters(0, 0.0, 150, 0.0, 0.0, JointType::PRISMATIC);  // Joint 0, prismatic
  scara.setJointParameters(1, 0.0, 0.0, 200, 0.0, JointType::REVOLUTE);   // Joint 1, revolute
  scara.setJointParameters(2, 0.0, 0.0, 200, 2*M_PI, JointType::REVOLUTE); // Joint 2, revolute

  // Set the joint limits
  scara.setJointLimits(0, 0.0, 300);  // Z limit
  scara.setJointLimits(1, -1.5*M_PI, 1.5*M_PI);  // Q0 limit
  scara.setJointLimits(2, -1.5*M_PI, 1.5*M_PI);  // Q1 limit

  // Set the joint initial positions
  scara.setJointPosition(0, 150);  // Z position
  scara.setJointPosition(1, 0);    // Q0 position
  scara.setJointPosition(2, 0);    // Q1 position

  // Forward kinematics
  Serial.println("--------------Forward Kinematics--------------");
  Eigen::Matrix4f fk_result = forward_kinematics(scara);
  RoboticsUtils::print_matrix(fk_result);
  
  // End effector position
  Serial.println("--------------End effector position--------------");
  Eigen::VectorXf ef_pos = scara.getEndEffectorPose();
  RoboticsUtils::print_vector(ef_pos);

  // Jacobian
  Serial.println("--------------Jacobian--------------");
  Eigen::Matrix<float, 6, -1> jacobian = scara.computeJacobian();
  RoboticsUtils::print_matrix(jacobian);

  Serial.println("--------------End effector speed--------------");
  Eigen::VectorXf jointVelocities(3);
          jointVelocities(0) = 10;  // Z velocity
          jointVelocities(1) = 10;  // X velocity
          jointVelocities(2) = 10;  // Y velocity
  Eigen::VectorXf ef_vel = scara.computeEndEffectorVelocity(jointVelocities);
  RoboticsUtils::print_vector(ef_vel);

  // Inverse kinematics
  Serial.println("--------------Inverse kinematics--------------");
  Eigen::Matrix4f desiredTransform; 
  desiredTransform.setIdentity();
  desiredTransform(0, 3) = 200;  // X position
  desiredTransform(1, 3) = 200;  // Y position
  desiredTransform(2, 3) = 150;  // Z position

  Eigen::VectorXf ik_solution(3);
  bool res = inverse_kinematics(scara, desiredTransform, ik_solution, 0.5, 1000);

  if (res) {
    Serial.print("IK Solution: ");
    RoboticsUtils::print_vector(ik_solution); 
  } else {
    Serial.println("IK solution not found.");
  }
}

void loop() {
  if (Serial.available()) {
    float x, y, z;

    // Read a line of data from serial input
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Extract the X, Y, Z values
    if (sscanf(input.c_str(), "%f %f %f", &x, &y, &z) == 3) {
        Eigen::Matrix4f desiredTransform;
        desiredTransform.setIdentity();
        desiredTransform(0, 3) = x;  // X position
        desiredTransform(1, 3) = y;  // Y position
        desiredTransform(2, 3) = z;  // Z position

        Eigen::VectorXf ik_solution(3);
        bool res = inverse_kinematics(scara, desiredTransform, ik_solution, 0.5, 1000);

        if (res) {
          Serial.print("IK Solution: ");
          RoboticsUtils::print_vector(ik_solution);
        } else {
          Serial.println("IK solution not found.");
        }
    } else {
        Serial.println("Invalid input. Use the format: X Y Z");
    }
  }
}
