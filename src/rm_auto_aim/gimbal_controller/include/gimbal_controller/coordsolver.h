/**
 * @file coordsolver.h
 * @brief 位姿解算器，主要是用来解算弹道的
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-04-09
 * 
 */

#ifndef SERIAL__COORDSOLVER_H_
#define SERIAL__COORDSOLVER_H_

// c++
#include <utility>

// eigen
#include <Eigen/Core>

class CoordSolver
{
public:
    CoordSolver(int max_iter, double stop_error, int rk_iterations);

    double calcYaw(const Eigen::Vector3d& xyz) const;
    double calcPitch(const Eigen::Vector3d& xyz) const;

    // Solve the final firing pitch in radians. Returns false if the target is
    // unreachable or the iteration fails to converge.
    bool solveBallisticPitch(const Eigen::Vector3d& xyz, double& pitch_rad) const;

    double bullet_speed = 15.0;
    double drag_coeff = 0.001;
    double gravity = 9.80665;
private:
    bool simulateTrajectory(double pitch_rad, double horizontal_distance, double& height) const;

    int max_iterations;
    double stop_error;
    int runge_kutta_iterations;
};

#endif  // SERIAL__COORDSOLVER_H_
