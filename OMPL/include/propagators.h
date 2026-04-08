#pragma once

#include <Eigen/Core>
#include <cmath>

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class Integrator : public oc::StatePropagator {
    public:
        Integrator(oc::SpaceInformationPtr &si) : oc::StatePropagator(si.get()), space(si->getStateSpace()) {}
        void propagate(const ob::State *state, const oc::Control *control, double duration, ob::State *result) const override;
        ob::State* RK4(const ob::State *x, const oc::Control *u, const double dt) const;
    protected:
        // Mapping from X x U (Cartesian product) to R^n through dynamics of problem.
        // Returns [dx, dy, dtheta]
        virtual void setStateUsingDelta(ob::State *state, const ob::State *stateRef, const Eigen::VectorXd &dx) const = 0;
        virtual Eigen::VectorXd dynamics(const ob::State *x, const oc::Control *u) const = 0;

        ob::StateSpacePtr space;
};

class SimpleUnicycle : public Integrator {
    // Simple car with acceleration controls
    public:
        SimpleUnicycle(oc::SpaceInformationPtr &si, const double angleBias = 0.0, const double r = 0.5) : Integrator(si), angleBias(angleBias), r(r) {}
    protected:
        void setStateUsingDelta(ob::State *state, const ob::State *stateRef, const Eigen::VectorXd &dx) const override;
        Eigen::VectorXd dynamics(const ob::State *x, const oc::Control *control) const override;
    private:
        double angleBias; // Angle bias in dynamics [rad]
        double r; // Radius of wheel
};

class SimpleCar : public Integrator{
    // ASSUMES SE2
    public:
        SimpleCar(oc::SpaceInformationPtr &si, const double length = 0.2, const double width = 0.1) : Integrator(si), L(length), W(width) {}
        const double getLength() {return L;}
        const double getWidth() {return W;}
    protected:
        void setStateUsingDelta(ob::State *state, const ob::State *stateRef, const Eigen::VectorXd &dx) const override;
        Eigen::VectorXd dynamics(const ob::State *x, const oc::Control *control) const override;
    private:
        double L; // Length of vehicle
        double W; // Width of vehicle
};

class SimpleCarAccel : public Integrator {
    // Simple car with acceleration controls
    public:
        SimpleCarAccel(oc::SpaceInformationPtr &si, const double length = 0.2, const double width = 0.1) : Integrator(si), L(length), W(width) {}
        const double getLength() {return L;}
        const double getWidth() {return W;}
    protected:
        void setStateUsingDelta(ob::State *state, const ob::State *stateRef, const Eigen::VectorXd &dx) const override;
        Eigen::VectorXd dynamics(const ob::State *x, const oc::Control *control) const override;
    private:
        double L; // Length of vehicle
        double W; // Width of vehicle
};