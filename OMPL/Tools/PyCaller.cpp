#include "PyCaller.h"

PyObject* PyCaller::toPyObject(const ompl::base::SE2StateSpace::StateType& state) {
    PyObject* pyTuple = PyTuple_New(3);
    PyTuple_SET_ITEM(pyTuple, 0, PyFloat_FromDouble(state.getX()));
    PyTuple_SET_ITEM(pyTuple, 1, PyFloat_FromDouble(state.getY()));
    PyTuple_SET_ITEM(pyTuple, 2, PyFloat_FromDouble(state.getYaw()));
    return pyTuple;
}
PyObject* PyCaller::toPyObject(ompl::geometric::PathGeometric* path) {
    const ompl::base::StateSpacePtr space = path->getSpaceInformation()->getStateSpace();
    int ssType(space->getType());

    const std::vector<ompl::base::State*>& states = path->getStates();
    PyObject* pyList = PyList_New(states.size());

    for (std::size_t i = 0; i < states.size(); ++i)
    {
        if (ssType == ompl::base::StateSpaceType::STATE_SPACE_SE2) {
            const ompl::base::SE2StateSpace::StateType* state = states[i]->as<ompl::base::SE2StateSpace::StateType>();
            PyList_SET_ITEM(pyList, i, toPyObject(*state));
        }
        else if (ssType == ompl::base::StateSpaceType::STATE_SPACE_UNKNOWN) { // Compound SS Type
            const ompl::base::SE2StateSpace::StateType* state = states[i]->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0);
            PyList_SET_ITEM(pyList, i, toPyObject(*state));
        }
    }
    return pyList;
}
PyObject* PyCaller::toPyObject(const Eigen::VectorXd& vector) {
    PyObject* pyList = PyList_New(vector.size());
    for (std::size_t i = 0; i < vector.size(); ++i) {
        PyList_SET_ITEM(pyList, i, PyFloat_FromDouble(vector(i)));
    }
    return pyList;
}