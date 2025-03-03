#include "Kinematics.h"
#include <cmath>
#include "RoboticsUtils.h"

Eigen::Matrix4f forward_kinematics(const DH& robot) {
    return robot.getTransformUpToJoint(-1); 
}

bool inverse_kinematics(DH &robot, const Eigen::Matrix4f &desiredTransform, Eigen::VectorXf &solution, float tolerance, int maxIterations) {
    bool debug = false;
    // Usamos el vector 'solution' como adivinanza inicial.
    Eigen::VectorXf q = robot.getJointPositions();

    // Precalcular partes invariantes de la transformación deseada
    const Eigen::Vector3f p_des = desiredTransform.block<3,1>(0,3);
    const Eigen::Matrix3f R_des = desiredTransform.block<3,3>(0,0);

    // Variables temporales para el error
    Eigen::Vector3f error_position, error_orientation;
    Eigen::Matrix<float,6,1> deltaX;

    // Parámetros de la optimización
    const float damping = 0.001f;  // Factor de amortiguamiento
    const float lambda = 0.1f;    // Ganancia para la actualización
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        // Actualizar la configuración del robot
        robot.setJointPositions(q);
        
        // Calcular la transformación actual (usando forward kinematics)
        Eigen::Matrix4f currentTransform = forward_kinematics(robot);
        
        // --- Cálculo del error de posición ---
        Eigen::Vector3f p_current = currentTransform.block<3,1>(0,3);
        error_position = p_des - p_current;
        
        // --- Cálculo del error de orientación ---
        Eigen::Matrix3f R_current = currentTransform.block<3,3>(0,0);
        // R_error es la diferencia entre la orientación deseada y la actual
        Eigen::Matrix3f R_error = R_des * R_current.transpose();
        float trace_R = R_error.trace();
        float angle = acosf((trace_R - 1.0f) / 2.0f);
        if (fabs(angle) < 1e-6f) {
            error_orientation.setZero();
        } else {
            // Se extrae el vector de error a partir del logaritmo de la matriz de rotación
            error_orientation << R_error(2,1) - R_error(1,2),
                                 R_error(0,2) - R_error(2,0),
                                 R_error(1,0) - R_error(0,1);
            error_orientation *= (angle / (2.0f * sin(angle)));
        }
        
        // Se forma el vector de error total (6x1): [error de posición; error de orientación]
        deltaX << error_position, error_orientation;

        // Si el modo debug está activado, imprimir errores y posición actual
        if (debug) {
            Serial.print("Iteración: ");
            Serial.println(iter);
            Serial.print("Posición deseada: ");
            RoboticsUtils::print_vector(p_des);
            Serial.print("Posición actual: ");
            RoboticsUtils::print_vector(p_current);
            Serial.print("Error de posición: ");
            RoboticsUtils::print_vector(error_position);
            Serial.print("Error de orientación (ángulo): ");
            Serial.println(angle);
        }

        // Comprobar si se ha alcanzado la convergencia
        if (deltaX.norm() < tolerance) {
            solution = q;
            return true;
        }
        
        // --- Cálculo del Jacobiano y su pseudoinversa ---
        Eigen::MatrixXf J = robot.computeJacobian();  // Matriz de dimensiones 6 x n (n: número de joints)
        Eigen::MatrixXf JJT = J * J.transpose();        // 6 x 6
        JJT.diagonal().array() += damping;
        // Calcular la pseudoinversa usando la descomposición LDLT
        Eigen::MatrixXf J_pinv = J.transpose() * JJT.ldlt().solve(Eigen::MatrixXf::Identity(JJT.rows(), JJT.cols()));
              
        // Actualizar la configuración utilizando una ganancia (lambda)
        q += lambda * (J_pinv * deltaX);
    }
    
    // Si no converge, se retorna la última solución calculada
    solution = q;
    return false;
}


