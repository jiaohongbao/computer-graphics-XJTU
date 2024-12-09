#include "solver.h"

#include <Eigen/Core>

using Eigen::Vector3f;

// External Force does not changed.

// Function to calculate the derivative of KineticState
KineticState derivative(const KineticState& state)
{
    return KineticState(state.velocity, state.acceleration, Eigen::Vector3f(0, 0, 0));
}

// Function to perform a single Forward Euler step
KineticState forward_euler_step([[maybe_unused]] const KineticState& previous,
                                const KineticState& current)
{
    KineticState next;

    next.acceleration = current.acceleration;

    next.position = current.position + current.velocity * time_step;

    next.velocity = current.velocity + current.acceleration * time_step;

    return next;
}

// Function to perform a single Runge-Kutta step
KineticState runge_kutta_step([[maybe_unused]] const KineticState& previous,
                              const KineticState& current)
{
    KineticState next = current;
    Vector3f k1       = current.acceleration;
    Vector3f k2 = k1, k3 = k1, k4 = k1;

    next.velocity = current.velocity + time_step / 6.0f * (k1 + 2 * k2 + 2 * k3 + k4);

    k1 = current.velocity;
    k2 = current.velocity + time_step / 2.0f * current.acceleration;
    k3 = k2;
    k4 = current.velocity + time_step * current.acceleration;

    next.position = current.position + time_step / 6.0f * (k1 + 2 * k2 + 2 * k3 + k4);
    return next;
    //  return current;
}

// Function to perform a single Backward Euler step
KineticState backward_euler_step([[maybe_unused]] const KineticState& previous,
                                 const KineticState& current)
{
    KineticState next;
    next.acceleration = current.acceleration;
    next.velocity     = current.velocity + next.acceleration * time_step;
    next.position     = current.position + next.velocity * time_step;

    return next;

    // return current;
}

// Function to perform a single Symplectic Euler step
KineticState symplectic_euler_step(const KineticState& previous, const KineticState& current)
{

    KineticState next;

    next.acceleration = current.acceleration;

    next.velocity = current.velocity + current.acceleration * time_step;

    next.position = current.position + next.velocity * time_step;

    return next;
    (void)previous;
    // return current;
}
