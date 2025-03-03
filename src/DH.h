#ifndef DH_H
#define DH_H

#include <ArduinoEigen.h>
#include <string>

/**
 * @brief Joint type enumeration
 */
enum class JointType {
    REVOLUTE,  // Angular joint
    PRISMATIC  // Linear joint
};

/**
 * @brief Limit type enumeration
 */
enum class LimitType {
    MIN = 0,
    MAX = 1
};


/**
 * @brief Class implementing the Denavit-Hartenberg parameters for robot kinematics.
 * 
 * This class provides functionality to model robot kinematics using standard Denavit-Hartenberg conventions.
 */
class DH {
public:
    /**
     * @brief Structure holding DH parameters for a single joint
     */
    struct DHJoint {
        double theta;     // Joint angle in radians (for revolute joints)
        double d;         // Joint offset/distance (for prismatic joints)
        double a;         // Link length
        double alpha;     // Link twist
        JointType type;   // Joint type (revolute or prismatic)
        double minLimit;  // Minimum joint limit
        double maxLimit;  // Maximum joint limit

        /**
         * @brief Default constructor with sensible defaults
         */
        DHJoint() : 
            theta(0.0), 
            d(0.0), 
            a(0.0), 
            alpha(0.0), 
            type(JointType::REVOLUTE), 
            minLimit(0.0), 
            maxLimit(0.0) {}
    };

    /**
     * @brief Constructor for the Robot Kinematics model.
     * 
     * @param numberOfJoints Number of degrees of freedom (joints) for the robot.
     */
    DH(int numberOfJoints);

    /**
     * @brief Set the Denavit-Hartenberg parameters for a specific joint.
     * 
     * @param jointIndex Index of the joint.
     * @param theta Joint angle in radians (for revolute joints).
     * @param d Distance (for prismatic joints).
     * @param a Link length.
     * @param alpha Link twist.
     * @param jointType Type of joint (revolute by default).
     */
    void setJointParameters(
        int jointIndex, 
        double theta, 
        double d, 
        double a, 
        double alpha, 
        JointType jointType = JointType::REVOLUTE
    );

    /**
     * @brief Set the position of a specific joint.
     * 
     * @param jointIndex Index of the joint.
     * @param position Position value for the joint (angle for revolute, distance for prismatic).
     * @return bool True if the position is within limits, false otherwise.
     */
    bool setJointPosition(int jointIndex, double position);

    /**
     * @brief Set the positions for all joints in the robot.
     * 
     * @param positions Vector of joint positions.
     * @return bool True if all positions are within limits, false otherwise.
     */
    bool setJointPositions(const Eigen::VectorXf& positions);

    /**
     * @brief Get the current position of a specific joint.
     * 
     * @param jointIndex Index of the joint.
     * @return double Joint position value (angle in radians or distance in meters).
     */
    double getJointPosition(int jointIndex) const;

    /**
     * @brief Get the current positions of all joints in the robot.
     * 
     * @return Eigen::VectorXf A vector containing the positions of all joints.
     */
    Eigen::VectorXf getJointPositions() const;

    /**
     * @brief Set the limits for a specific joint.
     * 
     * @param jointIndex Index of the joint.
     * @param minValue Minimum value for the joint position.
     * @param maxValue Maximum value for the joint position.
     */
    void setJointLimits(int jointIndex, double minValue, double maxValue);

    /**
     * @brief Get the limit of a specific joint.
     *
     * @param jointIndex Index of the joint.
     * @param limitType Which limit to retrieve (MIN or MAX).
     * @return double The limit value of the joint.
     */
    double getJointLimit(int jointIndex, LimitType limitType) const;

    /**
     * @brief Check if a given joint configuration is within the limits.
     * 
     * @param positions Joint configuration to check.
     * @return bool True if all joints are within limits, false otherwise.
     */
    bool areJointPositionsWithinLimits(const Eigen::VectorXf& positions) const;

    /**
     * @brief Get the transformation matrix (4x4) for a specific joint.
     * 
     * @param jointIndex Index of the joint.
     * @return Eigen::Matrix4f The 4x4 transformation matrix for the joint.
     */
    Eigen::Matrix4f getJointTransform(int jointIndex) const;

    /**
     * @brief Get the overall transformation matrix for the robot from base to a specific joint.
     * 
     * @param endJointIndex Last joint index to consider. If -1, considers all joints.
     * @return Eigen::Matrix4f The overall transformation matrix.
     */
    Eigen::Matrix4f getTransform(int endJointIndex = -1) const;

    /**
     * @brief Computes transformation matrix from base to specified joint.
     * 
     * @param jointIndex The joint index up to which the transformation is calculated.
     * @return Eigen::Matrix4f The transformation matrix from base to joint.
     */
    Eigen::Matrix4f getTransformUpToJoint(int jointIndex) const;

    /**
     * @brief Get the pose of the end-effector of the robot.
     * 
     * @return Eigen::VectorXf A vector containing position (x,y,z) and orientation (roll,pitch,yaw).
     */
    Eigen::VectorXf getEndEffectorPose() const;

    /**
     * @brief Get the number of degrees of freedom (joints) for the robot.
     * 
     * @return int The number of degrees of freedom.
     */
    int getNumberOfJoints() const;

    /**
     * @brief Computes the geometric Jacobian matrix at the current configuration.
     * 
     * @return Eigen::Matrix<float, 6, Eigen::Dynamic> The Jacobian matrix.
     */
    Eigen::Matrix<float, 6, Eigen::Dynamic> computeJacobian() const;

    /**
     * @brief Calculates the velocity of the end-effector from the joint velocities.
     * 
     * @param jointVelocities Vector of joint velocities.
     * @return Eigen::VectorXf The velocity of the end-effector (linear and angular).
     */
    Eigen::VectorXf computeEndEffectorVelocity(const Eigen::VectorXf& jointVelocities) const;

private:
    int numberOfJoints;         // Number of degrees of freedom
    std::vector<DHJoint> joints; // Vector of joint parameters
};

#endif // DH_H