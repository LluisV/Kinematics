#ifndef ROBOTICS_UTILS_H
#define ROBOTICS_UTILS_H

#include <cmath> 
#include <vector> 
#include <ArduinoEigen.h> 

namespace RoboticsUtils {

/**
 * @brief Wraps an angle to the range [-Pi, Pi].
 *
 * This function ensures that the given angle is wrapped within the range
 * of -Pi to Pi radians.
 *
 * @param angle Angle in radians to be wrapped.
 * @return Wrapped angle in radians within the range [-Pi, Pi].
 */
double wrap_angle(double angle);

/**
 * @brief Wraps a list of angles to the range [-Pi, Pi].
 *
 * This function processes a vector of angles, ensuring that each one
 * is wrapped within the range of -Pi to Pi radians.
 *
 * @param angles Vector of angles in radians to be wrapped.
 * @return Vector of wrapped angles within the range [-Pi, Pi].
 */
std::vector<double> wrap_angle(const std::vector<double>& angles);

template <typename Derived>
void print_matrix(const Eigen::MatrixBase<Derived>& mat) {
    for (int i = 0; i < mat.rows(); i++) {
        for (int j = 0; j < mat.cols(); j++) {
            Serial.print(mat(i, j), 6);
            Serial.print(" ");
        }
        Serial.println();
    }
}

template <typename Derived>
void print_vector(const Eigen::MatrixBase<Derived>& vec) {
    for (int i = 0; i < vec.size(); i++) {
        Serial.print(vec(i), 6);
        Serial.print(" ");
    }
    Serial.println();
}


} // namespace RoboticsUtils

#endif // ROBOTICS_UTILS_H
