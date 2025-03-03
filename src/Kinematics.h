#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <ArduinoEigen.h>
#include "DH.h"

/**
 * @brief Computes the forward kinematics of the robot, obtaining the full transformation matrix.
 *
 * @param robot The DH object representing the robot.
 * @return Eigen::Matrix4f The 4x4 transformation matrix representing the end-effector position and orientation.
 */
Eigen::Matrix4f forward_kinematics(const DH& robot);


/**
 * @brief Calculates the robot's inverse kinematics using an iterative method based on the pseudo-inverse of the Jacobian.
 *
 * @param robot DH object representing the robot. The joint positions will be updated internally.
 * @param desiredTransform 4x4 matrix representing the desired position and orientation of the end effector.
 * @param solution Vector where the found solution (joint positions) will be stored.
 * @param tolerance Convergence tolerance for the error (default is 1e-3).
 * @param maxIterations Maximum number of iterations (default is 100).
 * @return true if convergence is reached; false if the maximum number of iterations is exceeded.
 */
bool inverse_kinematics(DH &robot,
    const Eigen::Matrix4f &desiredTransform,
    Eigen::VectorXf &solution,
    float tolerance = 1e-3,
    int maxIterations = 1000);

#endif // KINEMATICS_H 