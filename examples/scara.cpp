#include <Arduino.h>
#include "DH.h"
#include "Kinematics.h"
#include "RoboticsUtils.h"

DH scara(3);

void setup() {
  Serial.begin(115200);
  delay(3000); // Espera para abrir el Serial Monitor
  
  
    // Definir parámetros DH para cada articulación
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
    Eigen::Matrix4f fk_result = forward_kinematics(scara);
    Serial.println("--------------Forward Kinematics--------------");
    RoboticsUtils::print_matrix(fk_result);
    
    // EF position
    Serial.println("--------------End effector position--------------");
    Eigen::VectorXf ef_pos = scara.getEndEffectorPose();
    RoboticsUtils::print_vector(ef_pos);

    // Jacobian
    Serial.println("--------------Jacobian--------------");
    Eigen::Matrix<float, 6, -1> jacobian = scara.computeJacobian();
    RoboticsUtils::print_matrix(jacobian);

    Serial.println("--------------End effector speed--------------");
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
    desiredTransform(0, 3) = 141.4;  // X
    desiredTransform(1, 3) = 341.4;  // Y
    desiredTransform(2, 3) = 150; // Z

    Eigen::VectorXf ik_solution(3);
    bool res = inverse_kinematics(scara, desiredTransform, ik_solution, 0.5, 1000);

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
        bool res = inverse_kinematics(scara, desiredTransform, ik_solution, 0.5, 1000);

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
