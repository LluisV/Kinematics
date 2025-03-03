#include "RoboticsUtils.h"

namespace RoboticsUtils {
    
    double wrap_angle(double angle) {
        angle = fmod(angle, 2.0 * M_PI);
        if (angle > M_PI) angle -= 2.0 * M_PI;
        if (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    std::vector<double> wrap_angle(const std::vector<double>& angles) {
        std::vector<double> wrapped_angles;
        wrapped_angles.reserve(angles.size()); // Reservar memoria para eficiencia
    
        for (double angle : angles) {
            wrapped_angles.push_back(wrap_angle(angle));
        }
        return wrapped_angles;
    }


} // namespace RoboticsUtils
