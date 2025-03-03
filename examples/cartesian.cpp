#include <Arduino.h>
#include "DH.h"
#include "Kinematics.h"
#include "RoboticsUtils.h"

DH cartesian(3);

void setup() {
    Serial.begin(115200);
    delay(3000); 
    
    // Cartesian robot setup
    cartesian.setJointParameters(0,   M_PI/2,    0.0,  0.0,  M_PI/2,  JointType::PRISMATIC); // Z axis 
    cartesian.setJointParameters(1,   M_PI/2, 0.0,  0.0,  M_PI/2,   JointType::PRISMATIC); // X axis
    cartesian.setJointParameters(2,   0, 0.0,  0.0,  -M_PI/2,       JointType::PRISMATIC); // Y axis

    // Set joint limits
    cartesian.setJointLimits(1, 0, 400);  // X axis limits
    cartesian.setJointLimits(2, 0, 600);  // Y axis limits
    cartesian.setJointLimits(0, 0, 300);  // Z axis limits

    // Set initial joint positions
    cartesian.setJointPosition(0, 100);
    cartesian.setJointPosition(1, 100);
    cartesian.setJointPosition(2, 100);

    // Forward kinematics
    Serial.println("--------------Forward Kinematics--------------");
    Eigen::Matrix4f fk_result = forward_kinematics(cartesian);
    RoboticsUtils::print_matrix(fk_result);

    // End effector position
    Serial.println("--------------End effector position--------------");
    Eigen::VectorXf ef_pos = cartesian.getEndEffectorPose();
    RoboticsUtils::print_vector(ef_pos);

    // Jacobian
    Serial.println("--------------Jacobian--------------");
    Eigen::Matrix<float, 6, -1> jacobian = cartesian.computeJacobian();
    RoboticsUtils::print_matrix(jacobian);

    // End effector speed
    Serial.println("--------------End effector speed--------------");
    Eigen::VectorXf jointVelocities(3);
            jointVelocities(0) = 10;  // Z velocity
            jointVelocities(1) = 10;  // X velocity
            jointVelocities(2) = 10;  // Y velocity  
    Eigen::VectorXf ef_vel = cartesian.computeEndEffectorVelocity(jointVelocities);
    RoboticsUtils::print_vector(ef_vel);

    // Inverse kinematics
    Serial.println("--------------Inverse kinematics--------------");
    Eigen::Matrix4f desiredTransform; 
    desiredTransform.setIdentity();
    desiredTransform(0, 3) = 150;  // X position
    desiredTransform(1, 3) = 150;  // Y position
    desiredTransform(2, 3) = 150; // Z position

    Eigen::VectorXf ik_solution(3);
    bool res = inverse_kinematics(cartesian, desiredTransform, ik_solution, 0.5, 1000);

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
    
        // Read a line of data from the serial input
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
            bool res = inverse_kinematics(cartesian, desiredTransform, ik_solution, 0.5, 1000);
    
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
