#include "propagators.h"

/* ========== Integrator Class ========== */

// Propagates state through
void Integrator::propagate(const ob::State *state, const oc::Control *x, double duration, ob::State *result) const {
    // UPDATES: state -> new state after propogation
    ob::State* rk4Result = RK4(state, x, duration);
    space->copyState(result, rk4Result);
    space->freeState(rk4Result);
}

// Given x_k, u_k, & dt, returns x_{k+1} using RK4 integration, where states k and k+1 are seperated by time dt
ob::State* Integrator::RK4(const ob::State *x, const oc::Control *u, double dt) const {
    ob::State* result = space->allocState();

    // Allocate temporary state for intermediate steps
    ob::State* tmp = space->allocState();

    Eigen::VectorXd w1 = dynamics(x, u);
    setStateUsingDelta(tmp, x, dt/2*w1);
    Eigen::VectorXd w2 = dynamics(tmp, u);
    setStateUsingDelta(tmp, x, dt/2*w2);
    Eigen::VectorXd w3 = dynamics(tmp, u);
    setStateUsingDelta(tmp, x, dt*w3);
    Eigen::VectorXd w4 = dynamics(tmp, u);
    space->freeState(tmp);  // Free temporary state memory

    // Find the "dx" to add to the state
    Eigen::VectorXd dx = dt/6*(w1 + 2*w2 + 2*w3 + w4);

    // Set result
    setStateUsingDelta(result, x, dx);

    return result;
}

/* ========== SimpleUnicycle Class ========== */

// Update state using state = x + dx
void SimpleUnicycle::setStateUsingDelta(ob::State *state, const ob::State *stateRef, const Eigen::VectorXd &dx) const {
    ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
    const ob::SE2StateSpace::StateType *x = stateRef->as<ob::SE2StateSpace::StateType>();
    s->setX(x->getX() + dx[0]);
    s->setY(x->getY() + dx[1]);
    s->setYaw(x->getYaw() + dx[2]);
}

// EoM of Simple Unicycle
Eigen::VectorXd SimpleUnicycle::dynamics(const ob::State *x, const oc::Control *control) const {
    // Initialize helper variables
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = x->as<ob::SE2StateSpace::StateType>()->getYaw();
    Eigen::Vector3d dx;

    // Dynamics
    dx[0] = u[0] * r * cos(theta);
    dx[1] = u[0] * r * sin(theta);
    dx[2] = u[1] + angleBias;

    return dx;
}

/* ========== SimpleCar Class ========== */

// Update state using state = x + dx
void SimpleCar::setStateUsingDelta(ob::State *state, const ob::State *stateRef, const Eigen::VectorXd &dx) const {
    ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
    const ob::SE2StateSpace::StateType *x = stateRef->as<ob::SE2StateSpace::StateType>();
    s->setX(x->getX() + dx[0]);
    s->setY(x->getY() + dx[1]);
    s->setYaw(x->getYaw() + dx[2]);
}

// EoM of Simple Car
Eigen::VectorXd SimpleCar::dynamics(const ob::State *x, const oc::Control *control) const {
    // Initialize helper variables
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = x->as<ob::SE2StateSpace::StateType>()->getYaw();
    Eigen::Vector3d dx;

    // Dynamics
    dx[0] = u[0] * cos(theta);
    dx[1] = u[0] * sin(theta);
    dx[2] = u[0] * tan(u[1]) / L;

    return dx;
}

/* ========== SimpleCarAccel Class ========== */

// Update state using state = x + dx
void SimpleCarAccel::setStateUsingDelta(ob::State *state, const ob::State *stateRef, const Eigen::VectorXd &dx) const {
    // Decompose compound state
    const ob::SE2StateSpace::StateType* se2StateRef = stateRef->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType* integratorStateRef = stateRef->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);
    ob::SE2StateSpace::StateType* se2State = state->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0);
    ob::RealVectorStateSpace::StateType* integratorState = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);

    // Update state
    se2State->setX(se2StateRef->getX() + dx[0]);
    se2State->setY(se2StateRef->getY() + dx[1]);
    se2State->setYaw(se2StateRef->getYaw() + dx[2]);
    integratorState->values[0] = integratorStateRef->values[0] + dx[3];
    integratorState->values[1] = integratorStateRef->values[1] + dx[4];
}

// EoM of Simple Car with Acceleration Controls
Eigen::VectorXd SimpleCarAccel::dynamics(const ob::State *x, const oc::Control *control) const {
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    
    // Extract SE2 state
    const auto* se2State = x->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0);
    const double theta = se2State->getYaw();

    // Extract integrator states
    const auto* integratorState = x->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);
    const double v = integratorState->values[0];
    const double delta = integratorState->values[1];

    // u[0] = linear acceleration, u[1] = steering angle
    Eigen::VectorXd dx(5);
    dx[0] = v * cos(theta);         // x_dot
    dx[1] = v * sin(theta);         // y_dot
    dx[2] = v * tan(delta) / L;     // theta_dot
    dx[3] = u[0];                   // v_dot
    dx[4] = u[1];                   // delta_dot

    return dx;
}