#include "DH.h"
#include "RoboticsUtils.h"
#include <cmath>
#include <cassert>


DH::DH(int numberOfJoints)
    : numberOfJoints(numberOfJoints) {
    // Initialize joints with default values
    joints.resize(numberOfJoints);
}

void DH::setJointParameters(
    int jointIndex, 
    double theta, 
    double d, 
    double a, 
    double alpha, 
    JointType jointType) {
    
    assert(jointIndex >= 0 && jointIndex < numberOfJoints);
    
    DHJoint& joint = joints[jointIndex];
    joint.theta = theta;
    joint.d = d;
    joint.a = a;
    joint.alpha = alpha;
    joint.type = jointType;
}

bool DH::setJointPosition(int jointIndex, double position) {
    assert(jointIndex >= 0 && jointIndex < numberOfJoints);
    
    DHJoint& joint = joints[jointIndex];
    
    // Check if joint limits are defined and position is within limits
    if (joint.minLimit < joint.maxLimit) {
        double adjustedPosition = position;
        
        // Wrap angle for revolute joints
        if (joint.type == JointType::REVOLUTE) {
            adjustedPosition = RoboticsUtils::wrap_angle(position);
        }
        
        if (adjustedPosition < joint.minLimit || adjustedPosition > joint.maxLimit) {
            return false;
        }
    }
    
    // Update the appropriate parameter based on joint type
    if (joint.type == JointType::REVOLUTE) {
        joint.theta = RoboticsUtils::wrap_angle(position);
    } else {
        joint.d = position;
    }
    
    return true;
}

bool DH::setJointPositions(const Eigen::VectorXf& positions) {
    assert(positions.size() == numberOfJoints);
    
    // First check if all positions are within limits
    if (!areJointPositionsWithinLimits(positions)) {
        return false;
    }
    
    // Apply all positions
    for (int i = 0; i < numberOfJoints; ++i) {
        if (joints[i].type == JointType::REVOLUTE) {
            joints[i].theta = RoboticsUtils::wrap_angle(positions(i));
        } else {
            joints[i].d = positions(i);
        }
    }
    
    return true;
}

double DH::getJointPosition(int jointIndex) const {
    assert(jointIndex >= 0 && jointIndex < numberOfJoints);
    
    const DHJoint& joint = joints[jointIndex];
    return (joint.type == JointType::REVOLUTE) ? joint.theta : joint.d;
}


Eigen::VectorXf DH::getJointPositions() const {
    Eigen::VectorXf positions(numberOfJoints);
    
    for (int i = 0; i < numberOfJoints; ++i) {
        positions(i) = getJointPosition(i);
    }
    
    return positions;
}


void DH::setJointLimits(int jointIndex, double minValue, double maxValue) {
    assert(jointIndex >= 0 && jointIndex < numberOfJoints);
    
    joints[jointIndex].minLimit = minValue;
    joints[jointIndex].maxLimit = maxValue;
}


double DH::getJointLimit(int jointIndex, LimitType limitType) const {
    assert(jointIndex >= 0 && jointIndex < numberOfJoints);
    
    return (limitType == LimitType::MIN) ? 
           joints[jointIndex].minLimit : 
           joints[jointIndex].maxLimit;
}


bool DH::areJointPositionsWithinLimits(const Eigen::VectorXf& positions) const {
    assert(positions.size() == numberOfJoints);
    
    constexpr double epsilon = 1e-6; 

    for (int i = 0; i < numberOfJoints; ++i) {
        const DHJoint& joint = joints[i];

        if (joint.minLimit < joint.maxLimit) {
            double value = positions(i);

            if (joint.type == JointType::REVOLUTE) {
                value = RoboticsUtils::wrap_angle(value);
            }

            // Comprobación con tolerancia
            if (value < joint.minLimit - epsilon || value > joint.maxLimit + epsilon) {
                return false;
            }
        }
    }

    return true;
}


Eigen::Matrix4f DH::getJointTransform(int jointIndex) const {
    assert(jointIndex >= 0 && jointIndex < numberOfJoints);
    
    const DHJoint& joint = joints[jointIndex];
    
    double ct = cos(joint.theta);
    double st = sin(joint.theta);
    double ca = cos(joint.alpha);
    double sa = sin(joint.alpha);
    double d = joint.d;
    double a = joint.a;
    
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    
    // Standard DH convention
    transform << ct, -st * ca, st * sa, a * ct,
                    st, ct * ca, -ct * sa, a * st,
                    0, sa, ca, d,
                    0, 0, 0, 1;
                     
    return transform;
}


Eigen::Matrix4f DH::getTransformUpToJoint(int endJointIndex) const {
    // If endJointIndex is negative, use all joints
    if (endJointIndex < 0) endJointIndex = numberOfJoints;
    
    // Verify endJointIndex is valid
    assert(endJointIndex <= numberOfJoints);
    
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    
    for (int i = 0; i < endJointIndex; ++i) {
        transform *= getJointTransform(i);
    }
    
    return transform;
}


Eigen::VectorXf DH::getEndEffectorPose() const {
    Eigen::Matrix4f transform = getTransformUpToJoint(-1);
    Eigen::Matrix3f rotationMatrix = transform.block<3, 3>(0, 0);
    
    // Extract position
    Eigen::Vector3f position = transform.block<3, 1>(0, 3);
    
    // Calculate Euler angles (XYZ convention)
    float roll, pitch, yaw;
    
    // Extract yaw (around z-axis)
    yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    
    // Extract pitch (around y-axis)
    pitch = atan2(-rotationMatrix(2, 0), 
                  sqrt(rotationMatrix(2, 1) * rotationMatrix(2, 1) + 
                       rotationMatrix(2, 2) * rotationMatrix(2, 2)));
    
    // Extract roll (around x-axis)
    roll = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));

    //float a_z = atan2(R(1, 0), R(0, 0));
    //float a_y = atan2(-R(2, 0), R(0, 0) * cos(a_z) + R(1, 0) * sin(a_z));
    //float a_x = atan2(-R(1, 2) * cos(a_z) + R(0, 2) * sin(a_z), R(1, 1) * cos(a_z) - R(0, 1) * sin(a_z));
    //pose << rth(0, 3), rth(1, 3), rth(2, 3), a_x, a_y, a_z;

    // Create the pose vector
    Eigen::VectorXf pose(6);
    pose << position, roll, pitch, yaw;
    
    return pose;
}


int DH::getNumberOfJoints() const {
    return numberOfJoints;
}


Eigen::Matrix<float, 6, Eigen::Dynamic> DH::computeJacobian() const {
    Eigen::Matrix<float, 6, Eigen::Dynamic> jacobian =
        Eigen::Matrix<float, 6, Eigen::Dynamic>::Zero(6, numberOfJoints);
    
    // Obtiene la transformación completa del efector final
    Eigen::Matrix4f T_end = getTransformUpToJoint(-1);
    Eigen::Vector3f p_end = T_end.block<3,1>(0, 3);

    for (int i = 0; i < numberOfJoints; ++i) {
        Eigen::Matrix4f T_i = getTransformUpToJoint(i);
        Eigen::Vector3f z_i = T_i.block<3, 1>(0, 2);  // Eje Z del sistema i-1
        Eigen::Vector3f p_i = T_i.block<3, 1>(0, 3);  // Origen del sistema i-1
        
        if (joints[i].type == JointType::REVOLUTE) {
            // Para articulación rotacional
            jacobian.block<3, 1>(0, i) = z_i.cross(p_end - p_i);
            jacobian.block<3, 1>(3, i) = z_i;
        } else {
            // Para articulación prismática
            jacobian.block<3, 1>(0, i) = z_i;
            jacobian.block<3, 1>(3, i) = Eigen::Vector3f::Zero();
        }
    }

    return jacobian;
}

Eigen::VectorXf DH::computeEndEffectorVelocity(
    const Eigen::VectorXf& jointVelocities) const {
    
    assert(jointVelocities.size() == numberOfJoints);
    
    // Compute the Jacobian matrix at the current configuration
    Eigen::Matrix<float, 6, Eigen::Dynamic> jacobian = computeJacobian();
    
    // Multiply the Jacobian by the joint velocities to get the end-effector velocity
    return jacobian * jointVelocities;
}
