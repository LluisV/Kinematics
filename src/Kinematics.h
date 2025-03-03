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
 * @brief Calcula la cinemática inversa del robot utilizando un método iterativo basado en la pseudoinversa del Jacobiano.
 *
 * @param robot Objeto DH que representa al robot. Se actualizarán internamente las posiciones articulares.
 * @param desiredTransform Matriz 4x4 deseada que representa la posición y orientación del efector final.
 * @param solution Vector donde se almacenará la solución (posición articular) encontrada.
 * @param tolerance Tolerancia para la convergencia del error (por defecto 1e-3).
 * @param maxIterations Número máximo de iteraciones (por defecto 100).
 * @return true si se alcanza la convergencia; false si se excede el número máximo de iteraciones.
 */
bool inverse_kinematics(DH &robot,
    const Eigen::Matrix4f &desiredTransform,
    Eigen::VectorXf &solution,
    float tolerance = 1e-3,
    int maxIterations = 1000);

#endif // KINEMATICS_H 