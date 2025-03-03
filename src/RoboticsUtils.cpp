#include "RoboticsUtils.h"

namespace RoboticsUtils {
    
    double wrap_angle(double angle) {
        // Use fmod to reduce the angle to the range [-2π, 2π]
        angle = fmod(angle, 2.0 * M_PI);

        // Adjust the angle if it's greater than π or less than -π
        if (angle > M_PI) angle -= 2.0 * M_PI;
        if (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    std::vector<double> wrap_angle(const std::vector<double>& angles) {
        std::vector<double> wrapped_angles;
        wrapped_angles.reserve(angles.size()); // Reserve memory for efficiency
    
        // Iterate through each angle in the input vector and wrap it
        for (double angle : angles) {
            wrapped_angles.push_back(wrap_angle(angle));
        }

        // Return the vector of wrapped angles
        return wrapped_angles;
    }


} // namespace RoboticsUtils
