#include "../include/gimbal_controller/coordsolver.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double MIN_HORIZONTAL_DISTANCE = 1e-4;
constexpr double MIN_FORWARD_SPEED = 1e-4;
constexpr double MIN_BULLET_SPEED = 1e-3;
constexpr double MIN_PITCH_RAD = -20.0 * M_PI / 180.0;
constexpr double MAX_PITCH_RAD = 45.0 * M_PI / 180.0;

struct TrajectoryState
{
    double z;
    double vx;
    double vz;
};

struct TrajectoryDerivative
{
    double dz_dx;
    double dvx_dx;
    double dvz_dx;
};
}  // namespace

CoordSolver::CoordSolver(int max_iter, double stop_error, int rk_iterations)
    : max_iterations(max_iter),
      stop_error(stop_error),
      runge_kutta_iterations(rk_iterations)
{
}

double CoordSolver::calcPitch(const Eigen::Vector3d& xyz) const
{
    return std::atan2(xyz.z(), std::hypot(xyz.x(), xyz.y()));
}

double CoordSolver::calcYaw(const Eigen::Vector3d& xyz) const
{
    return -std::atan2(xyz.y(), xyz.x());
}

bool CoordSolver::solveBallisticPitch(const Eigen::Vector3d& xyz, double& pitch_rad) const
{
    const double horizontal_distance = std::hypot(xyz.x(), xyz.y());
    if (!std::isfinite(horizontal_distance) || !std::isfinite(xyz.z()) ||
        bullet_speed < MIN_BULLET_SPEED || gravity <= 0.0 || drag_coeff < 0.0) {
        return false;
    }

    if (horizontal_distance < MIN_HORIZONTAL_DISTANCE) {
        pitch_rad = calcPitch(xyz);
        return std::isfinite(pitch_rad);
    }

    const double target_height = xyz.z();
    double low = MIN_PITCH_RAD;
    double high = MAX_PITCH_RAD;

    auto evaluate = [&](double test_pitch, double& error) -> bool {
        double simulated_height = 0.0;
        if (!simulateTrajectory(test_pitch, horizontal_distance, simulated_height)) {
            return false;
        }
        error = simulated_height - target_height;
        return std::isfinite(error);
    };

    double low_error = 0.0;
    double high_error = 0.0;
    if (!evaluate(low, low_error) || !evaluate(high, high_error)) {
        return false;
    }

    if (low_error > 0.0 || high_error < 0.0) {
        return false;
    }

    double best_pitch = calcPitch(xyz);
    double best_error = std::numeric_limits<double>::infinity();
    for (int i = 0; i < max_iterations; ++i) {
        const double mid = 0.5 * (low + high);
        double mid_error = 0.0;
        if (!evaluate(mid, mid_error)) {
            return false;
        }

        best_pitch = mid;
        best_error = mid_error;
        if (std::abs(mid_error) <= stop_error) {
            pitch_rad = best_pitch;
            return true;
        }

        if (mid_error > 0.0) {
            high = mid;
        } else {
            low = mid;
        }
    }

    if (!std::isfinite(best_error)) {
        return false;
    }

    pitch_rad = best_pitch;
    return true;
}

bool CoordSolver::simulateTrajectory(double pitch_rad, double horizontal_distance, double& height) const
{
    if (horizontal_distance < MIN_HORIZONTAL_DISTANCE) {
        height = 0.0;
        return true;
    }

    auto derivative = [this](const TrajectoryState& state, TrajectoryDerivative& out) -> bool {
        if (!std::isfinite(state.z) || !std::isfinite(state.vx) || !std::isfinite(state.vz) ||
            state.vx < MIN_FORWARD_SPEED) {
            return false;
        }

        const double speed = std::hypot(state.vx, state.vz);
        out.dz_dx = state.vz / state.vx;
        out.dvx_dx = -drag_coeff * speed;
        out.dvz_dx = (-gravity - drag_coeff * speed * state.vz) / state.vx;
        return std::isfinite(out.dz_dx) && std::isfinite(out.dvx_dx) && std::isfinite(out.dvz_dx);
    };

    auto advance = [&](const TrajectoryState& state, const TrajectoryDerivative& slope, double dx) {
        return TrajectoryState{
            state.z + slope.dz_dx * dx,
            state.vx + slope.dvx_dx * dx,
            state.vz + slope.dvz_dx * dx,
        };
    };

    TrajectoryState state{0.0, bullet_speed * std::cos(pitch_rad), bullet_speed * std::sin(pitch_rad)};
    if (state.vx < MIN_FORWARD_SPEED) {
        return false;
    }

    const int steps = std::max(1, runge_kutta_iterations);
    const double dx = horizontal_distance / static_cast<double>(steps);

    for (int i = 0; i < steps; ++i) {
        TrajectoryDerivative k1{};
        TrajectoryDerivative k2{};
        TrajectoryDerivative k3{};
        TrajectoryDerivative k4{};

        if (!derivative(state, k1)) {
            return false;
        }

        const TrajectoryState state_k2 = advance(state, k1, dx * 0.5);
        if (!derivative(state_k2, k2)) {
            return false;
        }

        const TrajectoryState state_k3 = advance(state, k2, dx * 0.5);
        if (!derivative(state_k3, k3)) {
            return false;
        }

        const TrajectoryState state_k4 = advance(state, k3, dx);
        if (!derivative(state_k4, k4)) {
            return false;
        }

        state.z += dx / 6.0 * (k1.dz_dx + 2.0 * k2.dz_dx + 2.0 * k3.dz_dx + k4.dz_dx);
        state.vx += dx / 6.0 * (k1.dvx_dx + 2.0 * k2.dvx_dx + 2.0 * k3.dvx_dx + k4.dvx_dx);
        state.vz += dx / 6.0 * (k1.dvz_dx + 2.0 * k2.dvz_dx + 2.0 * k3.dvz_dx + k4.dvz_dx);

        if (!std::isfinite(state.z) || !std::isfinite(state.vx) || !std::isfinite(state.vz) ||
            state.vx < MIN_FORWARD_SPEED) {
            return false;
        }
    }

    height = state.z;
    return std::isfinite(height);
}
