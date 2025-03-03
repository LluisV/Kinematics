#include <Arduino.h>
#include "DH.h"
#include "RoboticsUtils.h"
#include <ArduinoEigen.h>

// Constants for test validation
const float EPSILON = 1e-5; // Tolerance for floating point comparisons

// Function to check if two matrices are approximately equal
bool matrix_approx_equal(const Eigen::MatrixXf& m1, const Eigen::MatrixXf& m2, float epsilon = EPSILON) {
  if (m1.rows() != m2.rows() || m1.cols() != m2.cols()) {
    Serial.println("Matrix dimensions don't match");
    return false;
  }
  
  for (int i = 0; i < m1.rows(); i++) {
    for (int j = 0; j < m1.cols(); j++) {
      if (fabs(m1(i, j) - m2(i, j)) > epsilon) {
        Serial.print("Matrices differ at (");
        Serial.print(i);
        Serial.print(",");
        Serial.print(j);
        Serial.print("): ");
        Serial.print(m1(i, j), 6);
        Serial.print(" vs ");
        Serial.println(m2(i, j), 6);
        return false;
      }
    }
  }
  return true;
}

// Function to check if two vectors are approximately equal
bool vector_approx_equal(const Eigen::VectorXf& v1, const Eigen::VectorXf& v2, float epsilon = EPSILON) {
  if (v1.size() != v2.size()) {
    Serial.println("Vector dimensions don't match");
    return false;
  }
  
  for (int i = 0; i < v1.size(); i++) {
    if (fabs(v1(i) - v2(i)) > epsilon) {
      Serial.print("Vectors differ at index ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(v1(i), 6);
      Serial.print(" vs ");
      Serial.println(v2(i), 6);
      return false;
    }
  }
  return true;
}

// Function to test the wrap_angle utility
void test_wrap_angle() {
  Serial.println("\n=== Testing wrapAngle ===");
  
  // Test single angle wrapping
  double angle_tests[][2] = {
    {0.0, 0.0},
    {PI, PI},
    {-PI, -PI},
    {2*PI, 0.0},
    {-2*PI, 0.0},
    {3*PI, PI},
    {-3*PI, -PI},
    {PI/2, PI/2},
    {-PI/2, -PI/2},
    {5*PI/2, PI/2}
  };
  
  int num_tests = sizeof(angle_tests) / sizeof(angle_tests[0]);
  bool all_passed = true;
  
  for (int i = 0; i < num_tests; i++) {
    double input = angle_tests[i][0];
    double expected = angle_tests[i][1];
    double result = RoboticsUtils::wrap_angle(input);
    
    bool passed = fabs(result - expected) < EPSILON;
    if (!passed) {
      Serial.print("wrapAngle(");
      Serial.print(input, 6);
      Serial.print(") = ");
      Serial.print(result, 6);
      Serial.print(", expected ");
      Serial.println(expected, 6);
      all_passed = false;
    }
  }
  
  if (all_passed) {
    Serial.println("All wrapAngle tests passed!");
  } else {
    Serial.println("Some wrapAngle tests failed!");
  }
}

// Test basic functionality of the DH class (constructor, getters, setters)
void test_dh_basics() {
  Serial.println("\n=== Testing DH Basics ===");
  
  // Create a DH model with 3 DOF
  DH dh(3); // Using classic DH
  
  // Check number of DOF
  if (dh.getNumberOfJoints() != 3) {
    Serial.println("FAILED: Wrong number of DOF");
    return;
  }
  
  // Set DH parameters for a simple 3R planar robot
  dh.setJointParameters(0, 0, 0, 1.0, 0, JointType::REVOLUTE);  // Joint 1
  dh.setJointParameters(1, 0, 0, 1.0, 0, JointType::REVOLUTE);  // Joint 2
  dh.setJointParameters(2, 0, 0, 1.0, 0, JointType::REVOLUTE);  // Joint 3
  
  // Set joint limits
  dh.setJointLimits(0, -PI, PI);
  dh.setJointLimits(1, -PI, PI);
  dh.setJointLimits(2, -PI, PI);
  
  // Check joint limits
  if (fabs(dh.getJointLimit(0, LimitType::MIN) - (-PI)) > EPSILON ||
      fabs(dh.getJointLimit(0, LimitType::MAX) - PI) > EPSILON) {
    Serial.println("FAILED: Wrong joint limits");
    return;
  }
  
  // Set joint values
  Eigen::VectorXf q(3);
  q << PI/4, PI/3, PI/6;
  dh.setJointPositions(q);
  
  // Check individual joint values
  for (int i = 0; i < 3; i++) {
    if (fabs(dh.getJointPosition(i) - q(i)) > EPSILON) {
      Serial.print("FAILED: getJointPosition(");
      Serial.print(i);
      Serial.print(") = ");
      Serial.print(dh.getJointPosition(i), 6);
      Serial.print(", expected ");
      Serial.println(q(i), 6);
      return;
    }
  }
  
  // Check all joint values
  Eigen::VectorXf joints = dh.getJointPositions();
  if (!vector_approx_equal(joints, q)) {
    Serial.println("FAILED: getJointPositions() returned wrong values");
    return;
  }
  
  Serial.println("PASSED: All DH basic tests passed!");
}

// Test the check_limits function
void test_check_limits() {
  Serial.println("\n=== Testing areJointPositionsWithinLimits ===");
  
  DH dh(2);
  
  // Set DH parameters
  dh.setJointParameters(0, 0, 0, 1.0, 0, JointType::REVOLUTE);  // Angular joint
  dh.setJointParameters(1, 0, 1.0, 0, 0, JointType::PRISMATIC); // Prismatic joint
  
  // Set joint limits
  dh.setJointLimits(0, -PI/2, PI/2);
  dh.setJointLimits(1, 0.5, 1.5);
  
  // Test cases that should pass
  Eigen::VectorXf q_good1(2);
  q_good1 << 0, 1.0;
  
  Eigen::VectorXf q_good2(2);
  q_good2 << -PI/2, 0.5;
  
  Eigen::VectorXf q_good3(2);
  q_good3 << PI/2, 1.5;
  
  // Test cases that should fail
  Eigen::VectorXf q_bad1(2);
  q_bad1 << -PI, 1.0;  // First joint out of limits
  
  Eigen::VectorXf q_bad2(2);
  q_bad2 << 0, 2.0;    // Second joint out of limits
  
  Serial.println("Testing valid joint configurations:");
  bool passed = true;
  
  if (!dh.areJointPositionsWithinLimits(q_good1)) {
    Serial.println("FAILED: Valid configuration 1 rejected");
    passed = false;
  }
  
  if (!dh.areJointPositionsWithinLimits(q_good2)) {
    Serial.println("FAILED: Valid configuration 2 rejected");
    passed = false;
  }
  
  if (!dh.areJointPositionsWithinLimits(q_good3)) {
    Serial.println("FAILED: Valid configuration 3 rejected");
    passed = false;
  }
  
  Serial.println("Testing invalid joint configurations:");
  if (dh.areJointPositionsWithinLimits(q_bad1)) {
    Serial.println("FAILED: Invalid configuration 1 accepted");
    passed = false;
  }
  
  if (dh.areJointPositionsWithinLimits(q_bad2)) {
    Serial.println("FAILED: Invalid configuration 2 accepted");
    passed = false;
  }
  
  if (passed) {
    Serial.println("PASSED: All areJointPositionsWithinLimits tests passed!");
  }
}

// Test the transformation matrix calculation for a single DOF
void test_get_joint_transform() {
  Serial.println("\n=== Testing getJointTransform ===");
  
  // Test with Classic DH
  {
    DH dh_classic(1);
    dh_classic.setJointParameters(0, PI/2, 2.0, 1.0, PI/4, JointType::REVOLUTE);
    
    Eigen::Matrix4f expected;
    float c_theta = cos(PI/2);
    float s_theta = sin(PI/2);
    float c_alpha = cos(PI/4);
    float s_alpha = sin(PI/4);
    
    // Expected matrix for classic DH with given parameters
    expected << c_theta, -s_theta*c_alpha, s_theta*s_alpha, 1.0*c_theta,
                s_theta, c_theta*c_alpha, -c_theta*s_alpha, 1.0*s_theta,
                0, s_alpha, c_alpha, 2.0,
                0, 0, 0, 1;
    
    Eigen::Matrix4f result = dh_classic.getJointTransform(0);
    
    if (matrix_approx_equal(result, expected)) {
      Serial.println("PASSED: Classic DH transformation matrix test passed!");
    } else {
      Serial.println("FAILED: Classic DH transformation matrix test failed!");
      Serial.println("Expected:");
      RoboticsUtils::print_matrix(expected);
      Serial.println("Got:");
      RoboticsUtils::print_matrix(result);
    }
  }
  

}

// Test the full transformation matrix calculation (getTransform)
void test_get_transform() {
  Serial.println("\n=== Testing getTransform ===");
  
  // Create a simple 2R planar robot
  DH dh(2);  // Classic DH
  
  // Set DH parameters
  dh.setJointParameters(0, 0, 0, 1.0, 0, JointType::REVOLUTE);  // Joint 1
  dh.setJointParameters(1, 0, 0, 1.0, 0, JointType::REVOLUTE);  // Joint 2
  
  // Test case 1: Both joints at 0
  {
    Eigen::VectorXf q(2);
    q << 0, 0;
    dh.setJointPositions(q);
    
    Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
    expected(0, 3) = 2.0;  // End effector at (2,0,0)
    
    Eigen::Matrix4f result = dh.getTransformUpToJoint(-1);
    
    if (matrix_approx_equal(result, expected)) {
      Serial.println("PASSED: getTransform test 1 passed!");
    } else {
      Serial.println("FAILED: getTransform test 1 failed!");
      Serial.println("Expected:");
      RoboticsUtils::print_matrix(expected);
      Serial.println("Got:");
      RoboticsUtils::print_matrix(result);
    }
  }
  
  // Test case 2: First joint at 90 degrees, second at 0
  {
    Eigen::VectorXf q(2);
    q << PI/2, 0;
    dh.setJointPositions(q);
    
    Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
    expected(0, 0) = 0;
    expected(0, 1) = -1;
    expected(1, 0) = 1;
    expected(1, 1) = 0;
    expected(0, 3) = 0;
    expected(1, 3) = 1;
    expected(2, 3) = 0;
    
    // Second transformation
    Eigen::Matrix4f T2 = Eigen::Matrix4f::Identity();
    T2(0, 3) = 1.0;
    
    expected = expected * T2;
    
    Eigen::Matrix4f result = dh.getTransformUpToJoint(-1);
    
    if (matrix_approx_equal(result, expected)) {
      Serial.println("PASSED: getTransform test 2 passed!");
    } else {
      Serial.println("FAILED: getTransform test 2 failed!");
      Serial.println("Expected:");
      RoboticsUtils::print_matrix(expected);
      Serial.println("Got:");
      RoboticsUtils::print_matrix(result);
    }
  }
  
  // Test case 3: First joint at 0, second at 90 degrees
  {
    Eigen::VectorXf q(2);
    q << 0, PI/2;
    dh.setJointPositions(q);
    
    // First transformation
    Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
    T1(0, 3) = 1.0;
    
    // Second transformation
    Eigen::Matrix4f T2 = Eigen::Matrix4f::Identity();
    T2(0, 0) = 0;
    T2(0, 1) = -1;
    T2(1, 0) = 1;
    T2(1, 1) = 0;
    T2(0, 3) = 0;
    T2(1, 3) = 1;
    
    Eigen::Matrix4f expected = T1 * T2;
    
    Eigen::Matrix4f result = dh.getTransformUpToJoint(-1);
    
    if (matrix_approx_equal(result, expected)) {
      Serial.println("PASSED: getTransform test 3 passed!");
    } else {
      Serial.println("FAILED: getTransform test 3 failed!");
      Serial.println("Expected:");
      RoboticsUtils::print_matrix(expected);
      Serial.println("Got:");
      RoboticsUtils::print_matrix(result);
    }
  }
  
  // Test partial transformation (up to first joint)
  {
    Eigen::VectorXf q(2);
    q << PI/4, PI/3;
    dh.setJointPositions(q);
    
    Eigen::Matrix4f full = dh.getTransformUpToJoint(-1);
    Eigen::Matrix4f partial = dh.getTransformUpToJoint(1); // Up to first joint
    
    // The partial transform should be different from the full transform
    if (!matrix_approx_equal(full, partial)) {
      Serial.println("PASSED: Partial getTransformUpToJoint test passed!");
    } else {
      Serial.println("FAILED: Partial getTransformUpToJoint test failed!");
    }
  }
}

// Test the end effector pose calculation
void test_get_end_effector_pose() {
  Serial.println("\n=== Testing getEndEffectorPose ===");
  
  // Create a simple 2R planar robot
  DH dh(2);  // Classic DH
  
  // Set DH parameters
  dh.setJointParameters(0, 0, 0, 1.0, 0, JointType::REVOLUTE);  // Joint 1
  dh.setJointParameters(1, 0, 0, 1.0, 0, JointType::REVOLUTE);  // Joint 2
  
  // Test case: Both joints at 45 degrees
  Eigen::VectorXf q(2);
  q << PI/4, PI/4;
  dh.setJointPositions(q);
  
  // Calculate expected position manually
  // For a 2R planar robot with both links = 1 and both joints at 45 degrees
  // x = cos(PI/4) + cos(PI/4 + PI/4) = 0.7071 + 0 = 0.7071
  // y = sin(PI/4) + sin(PI/4 + PI/4) = 0.7071 + 1 = 1.7071
  // z = 0
  
  // Calculate expected orientation (roll, pitch, yaw)
  // The final orientation should be PI/2 (roll=0, pitch=0, yaw=PI/2)
  
  Eigen::VectorXf expected(6);
  expected << 0.7071, 1.7071, 0, 0, 0, PI/2;
  
  Eigen::VectorXf result = dh.getEndEffectorPose();
  
  if (vector_approx_equal(result, expected, 0.001)) {
    Serial.println("PASSED: getEndEffectorPose test passed!");
  } else {
    Serial.println("FAILED: getEndEffectorPose test failed!");
    Serial.println("Expected:");
    RoboticsUtils::print_vector(expected);
    Serial.println("Got:");
    RoboticsUtils::print_vector(result);
  }
}

// Test the Jacobian calculation
void test_jacobian() {
  Serial.println("\n=== Testing computeJacobian ===");
  
  // Create a simple 2R planar robot
  DH dh(2);  // Classic DH
  
  // Set DH parameters
  dh.setJointParameters(0, 0, 0, 1.0, 0, JointType::REVOLUTE);  // Joint 1 (rotational)
  dh.setJointParameters(1, 0, 0, 1.0, 0, JointType::REVOLUTE);  // Joint 2 (rotational)
  
  // Set joint values: both at 0
  Eigen::VectorXf q(2);
  q << 0, 0;
  dh.setJointPositions(q);
  
  // For a 2R planar robot with joints at 0, the Jacobian should be:
  // Linear part (first 3 rows):
  // [0   0]   (x-component from joint 1)
  // [1   1]   (y-component from joint 1 and 2)
  // [0   0]   (z-component)
  // Angular part (last 3 rows):
  // [0   0]   (rotation around x)
  // [0   0]   (rotation around y)
  // [1   1]   (rotation around z for both joints)
  
  Eigen::MatrixXf expected(6, 2);
  expected << 0, 0,
              1, 1,
              0, 0,
              0, 0,
              0, 0,
              1, 1;
  
  Eigen::MatrixXf J = dh.computeJacobian();
  
  if (matrix_approx_equal(J, expected)) {
    Serial.println("PASSED: Jacobian at zero position test passed!");
  } else {
    Serial.println("FAILED: Jacobian at zero position test failed!");
    Serial.println("Expected:");
    RoboticsUtils::print_matrix(expected);
    Serial.println("Got:");
    RoboticsUtils::print_matrix(J);
  }
  
  // Test with non-zero joint values
  q << PI/2, 0;
  dh.setJointPositions(q);
  
  // For a 2R planar robot with first joint at PI/2 and second at 0:
  // Linear part:
  // [-1  -1]  (x-component)
  // [0   0]   (y-component)
  // [0   0]   (z-component)
  // Angular part remains the same
  
  expected << -1, -1,
              0, 0,
              0, 0,
              0, 0,
              0, 0,
              1, 1;
  
  J = dh.computeJacobian();
  
  if (matrix_approx_equal(J, expected)) {
    Serial.println("PASSED: Jacobian at non-zero position test passed!");
  } else {
    Serial.println("FAILED: Jacobian at non-zero position test failed!");
    Serial.println("Expected:");
    RoboticsUtils::print_matrix(expected);
    Serial.println("Got:");
    RoboticsUtils::print_matrix(J);
  }
}

// Test the end effector velocity calculation
void test_get_end_effector_velocity() {
  Serial.println("\n=== Testing computeEndEffectorVelocity ===");
  
  // Create a simple 2R planar robot
  DH dh(2);  // Classic DH
  
  // Set DH parameters
  dh.setJointParameters(0, 0, 0, 1.0, 0, JointType::REVOLUTE);  // Joint 1
  dh.setJointParameters(1, 0, 0, 1.0, 0, JointType::REVOLUTE);  // Joint 2
  
  // Set joint values: both at 0
  Eigen::VectorXf q(2);
  q << 0, 0;
  dh.setJointPositions(q);
  
  // Set joint velocities: first joint moving at 1 rad/s, second not moving
  Eigen::VectorXf q_dot(2);
  q_dot << 1.0, 0.0;
  
  // Expected velocity (from Jacobian calculation above):
  // Linear: [0, 1, 0] * [1, 0]' = [0, 1, 0]
  // Angular: [0, 0, 1] * [1, 0]' = [0, 0, 1]
  Eigen::VectorXf expected(6);
  expected << 0, 1, 0, 0, 0, 1;
  
  Eigen::VectorXf result = dh.computeEndEffectorVelocity(q_dot);
  
  if (vector_approx_equal(result, expected)) {
    Serial.println("PASSED: End effector velocity test 1 passed!");
  } else {
    Serial.println("FAILED: End effector velocity test 1 failed!");
    Serial.println("Expected:");
    RoboticsUtils::print_vector(expected);
    Serial.println("Got:");
    RoboticsUtils::print_vector(result);
  }
  
  // Test with both joints moving
  q_dot << 1.0, 2.0;
  
  // Expected velocity:
  // Linear: [0, 1, 0] * [1, 2]' = [0, 3, 0]
  // Angular: [0, 0, 1] * [1, 2]' = [0, 0, 3]
  expected << 0, 3, 0, 0, 0, 3;
  
  result = dh.computeEndEffectorVelocity(q_dot);
  
  if (vector_approx_equal(result, expected)) {
    Serial.println("PASSED: End effector velocity test 2 passed!");
  } else {
    Serial.println("FAILED: End effector velocity test 2 failed!");
    Serial.println("Expected:");
    RoboticsUtils::print_vector(expected);
    Serial.println("Got:");
    RoboticsUtils::print_vector(result);
  }
}

// Test a more complex 3D robot
void test_complex_robot() {
  Serial.println("\n=== Testing Complex Robot ===");
  
  // Create a 3-DOF robot with 2 rotational and 1 prismatic joint
  DH dh(3);  // Classic DH
  
  // Set DH parameters for a simple robot with:
  // - First joint rotates around Z
  // - Second joint rotates around Y
  // - Third joint extends along Z
  dh.setJointParameters(0, 0, 0, 0, PI/2, JointType::REVOLUTE);     // Joint 1
  dh.setJointParameters(1, 0, 0, 1.0, -PI/2, JointType::REVOLUTE);  // Joint 2
  dh.setJointParameters(2, 0, 0, 0, 0, JointType::PRISMATIC);       // Joint 3 (prismatic)
  
  // Set joint values
  Eigen::VectorXf q(3);
  q << PI/4, PI/3, 0.5;
  dh.setJointPositions(q);
  
  // Test getTransform
  Eigen::Matrix4f tf = dh.getTransformUpToJoint(-1);
  
  // Print the transformation matrix
  Serial.println("Full transformation matrix:");
  RoboticsUtils::print_matrix(tf);
  
  // Make sure the result seems reasonable
  if (tf(3, 3) == 1.0) {
    Serial.println("PASSED: Complex robot transformation has proper format");
  } else {
    Serial.println("FAILED: Complex robot transformation has wrong format");
  }
  
  // Test the Jacobian
  Eigen::MatrixXf J = dh.computeJacobian();
  
  Serial.println("Jacobian matrix:");
  RoboticsUtils::print_matrix(J);
  
  // Check Jacobian dimensions
  if (J.rows() == 6 && J.cols() == 3) {
    Serial.println("PASSED: Complex robot Jacobian has correct dimensions");
  } else {
    Serial.println("FAILED: Complex robot Jacobian has wrong dimensions");
  }
  
  // Test end effector pose
  Eigen::VectorXf pose = dh.getEndEffectorPose();
  
  Serial.println("End effector pose:");
  RoboticsUtils::print_vector(pose);
  
  // Check pose dimensions
  if (pose.size() == 6) {
    Serial.println("PASSED: Complex robot pose has correct dimensions");
  } else {
    Serial.println("FAILED: Complex robot pose has wrong dimensions");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  delay(3000);

  Serial.println("=== DH Library Test Program ===");
  Serial.println("Testing all methods of the DH kinematics library...");
  
  // Run all tests
  test_wrap_angle();
  test_dh_basics();
  test_check_limits();
  test_get_joint_transform();
  test_get_transform();
  test_get_end_effector_pose();
  test_jacobian();
  test_get_end_effector_velocity();
  test_complex_robot();
  
  Serial.println("\n=== All Tests Completed ===");
}

void loop() {
  // Nothing to do in loop
  delay(1000);
}