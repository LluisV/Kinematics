#include <Arduino.h>
#include "DH.h"
#include "Kinematics.h"
#include "RoboticsUtils.h"

DH cartesian(3);

void setup() {
    Serial.begin(115200);
    delay(3000); 
    
    // Cartesian robot
    cartesian.setJointParameters(0,   M_PI/2,    0.0,  0.0,  M_PI/2,  JointType::PRISMATIC); // Z 
    cartesian.setJointParameters(1,   M_PI/2, 0.0,  0.0,  M_PI/2,   JointType::PRISMATIC); // X
    cartesian.setJointParameters(2,   0, 0.0,  0.0,  -M_PI/2,       JointType::PRISMATIC); // Y

    cartesian.setJointLimits(1, 0, 400);  // X limits
    cartesian.setJointLimits(2, 0, 600);  // Y limits
    cartesian.setJointLimits(0, 0, 300);  // Z limits

    cartesian.setJointPosition(0, 100);
    cartesian.setJointPosition(1, 100);
    cartesian.setJointPosition(2, 100);


    // Forward kinematics
    Serial.println("--------------Forward Kinematics--------------");
    Eigen::Matrix4f fk_result = forward_kinematics(cartesian);
    RoboticsUtils::print_matrix(fk_result);

    // EF position
    Serial.println("--------------End effector position--------------");
    Eigen::VectorXf ef_pos = cartesian.getEndEffectorPose();
    RoboticsUtils::print_vector(ef_pos);

    // Jacobian
    Serial.println("--------------Jacobian--------------");
    Eigen::Matrix<float, 6, -1> jacobian = cartesian.computeJacobian();
    RoboticsUtils::print_matrix(jacobian);

    Serial.println("--------------End effector speed--------------");
    Eigen::VectorXf jointVelocities(3);
            jointVelocities(0) = 10;  // Z
            jointVelocities(1) = 10;  // X
            jointVelocities(2) = 10;  // Y  
    Eigen::VectorXf ef_vel = cartesian.computeEndEffectorVelocity(jointVelocities);
    RoboticsUtils::print_vector(ef_vel);

    // Inverse kinematics
    Serial.println("--------------Inverse kinematics--------------");
    Eigen::Matrix4f desiredTransform; 
    desiredTransform.setIdentity();
    desiredTransform(0, 3) = 150;  // X
    desiredTransform(1, 3) = 150;  // Y
    desiredTransform(2, 3) = 150; // Z

    Eigen::VectorXf ik_solution(3);
    bool res = inverse_kinematics(cartesian, desiredTransform, ik_solution, 0.5, 1000);

    if (res) {
      Serial.print("Solucion IK: ");
      RoboticsUtils::print_vector(ik_solution);  // Mostrar la solución si se encuentra
    } else {
        Serial.println("No se pudo encontrar una solución para la cinemática inversa.");
        RoboticsUtils::print_vector(ik_solution);
    }
}

void loop() {
    if (Serial.available()) {
        float x, y, z;
    
        // Leer línea de datos del serial
        String input = Serial.readStringUntil('\n');
        input.trim();
    
        // Extraer los valores de X, Y, Z
        if (sscanf(input.c_str(), "%f %f %f", &x, &y, &z) == 3) {
            Eigen::Matrix4f desiredTransform;
            desiredTransform.setIdentity();
            desiredTransform(0, 3) = x;  // X
            desiredTransform(1, 3) = y;  // Y
            desiredTransform(2, 3) = z;  // Z
    
            Eigen::VectorXf ik_solution(3);
            bool res = inverse_kinematics(cartesian, desiredTransform, ik_solution, 0.5, 1000);
    
            if (res) {
                Serial.print("Solución IK: ");
                RoboticsUtils::print_vector(ik_solution);
            } else {
                Serial.println("No se encontró una solución para la cinemática inversa.");
                RoboticsUtils::print_vector(ik_solution);
            }
        } else {
            Serial.println("Entrada inválida. Use el formato: X Y Z");
        }
      }
}
