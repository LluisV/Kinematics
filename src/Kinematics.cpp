#include "Kinematics.h"
#include <cmath>
#include "RoboticsUtils.h"

Eigen::Matrix4f forward_kinematics(const DH& robot) {
    return robot.getTransformUpToJoint(-1); 
}

bool inverse_kinematics(DH &robot, const Eigen::Matrix4f &desiredTransform, Eigen::VectorXf &solution, float tolerance, int maxIterations) {
    bool debug = false;
    
    // We use the current pose as the initial guess.
    Eigen::VectorXf q = robot.getJointPositions();

    // Store the initial pose to return it if the algorithm doesn't converge
    Eigen::VectorXf initial_q = q;

    // Pre-calculate invariant parts of the desired transformation
    const Eigen::Vector3f p_des = desiredTransform.block<3,1>(0,3);
    const Eigen::Matrix3f R_des = desiredTransform.block<3,3>(0,0);

    // Temporary variables for the error
    Eigen::Vector3f error_position, error_orientation;
    Eigen::Matrix<float,6,1> deltaX;

    // Optimization parameters
    const float damping = 0.001f;  // Damping factor
    const float lambda = 0.1f;    // Gain for the update
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        // Update the robot configuration
        robot.setJointPositions(q);
        
        // Calculate the current transformation (using forward kinematics)
        Eigen::Matrix4f currentTransform = forward_kinematics(robot);
        
        // --- Calculate position error ---
        Eigen::Vector3f p_current = currentTransform.block<3,1>(0,3);
        error_position = p_des - p_current;

        // --- Calculate orientation error ---
        Eigen::Matrix3f R_current = currentTransform.block<3,3>(0,0);
        // R_error is the difference between the desired and current orientation
        Eigen::Matrix3f R_error = R_des * R_current.transpose();
        float trace_R = R_error.trace();
        float angle = acosf((trace_R - 1.0f) / 2.0f);
        if (fabs(angle) < 1e-6f) {
            error_orientation.setZero();
        } else {
            // Extract the error vector from the logarithm of the rotation matrix
            error_orientation << R_error(2,1) - R_error(1,2),
                                 R_error(0,2) - R_error(2,0),
                                 R_error(1,0) - R_error(0,1);
            error_orientation *= (angle / (2.0f * sin(angle)));
        }
        
        // Form the total error vector (6x1): [position error; orientation error]
        deltaX << error_position, error_orientation;

        // If debug mode is enabled, print errors and current position
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

        // Check if convergence has been reached
        if (deltaX.norm() < tolerance) {
            solution = q;
            return true;
        }
        
        // --- Calculate the Jacobian and its pseudoinverse ---
        Eigen::MatrixXf J = robot.computeJacobian();  // Matrix of dimensions 6 x n (n: number of joints)
        Eigen::MatrixXf JJT = J * J.transpose();      // 6 x 6
        JJT.diagonal().array() += damping;
        // Calculate the pseudoinverse using LDLT decomposition
        Eigen::MatrixXf J_pinv = J.transpose() * JJT.ldlt().solve(Eigen::MatrixXf::Identity(JJT.rows(), JJT.cols()));
              
        // Update the configuration using a gain (lambda)
        q += lambda * (J_pinv * deltaX);
    }
    
    // If it does not converge, return the initial pose
    solution = initial_q;
    
    return false;
}


