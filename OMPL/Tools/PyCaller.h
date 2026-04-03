#pragma once

#include <Python.h>
#include <numpy/arrayobject.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <string>
#include <tuple>
#include <memory>
#include <iostream>

class PyCaller {
public:
    // Call a Python function with arguments stored in a tuple
    template <typename... ARGS_T>
    static void call(const std::string& module, const std::string& function, const std::tuple<ARGS_T...>& args_tuple = std::tuple<>()) {
    {
        GILGuard gilLock;

        // Import module
        PyObjectRef pName(PyUnicode_FromString(module.c_str()));
        PyObjectRef pModule(PyImport_Import(pName));
        if (!pModule) {
            PyErr_Print();
            std::cerr << "Failed to import Python module: " << module << "\n";
            return;
        }

        // Get function
        PyObjectRef pFunc(PyObject_GetAttrString(pModule, function.c_str()));
        if (!pFunc || !PyCallable_Check(pFunc)) {
            PyErr_Print();
            std::cerr << "Function not found or not callable: " << function << "\n";
            return;
        }

        // Build arguments tuple
        PyObjectRef pArgs(buildTuple(args_tuple));

        // Call Python function
        PyObject_CallObject(pFunc, pArgs);
    }
}

private:
    // Initializes Python at program start, finalizes at program end
    struct _Lifeline {
            _Lifeline() {
                Py_Initialize();
                initialize_numpy();
                {
                    GILGuard gilLock;
                    // Add Tools folder to sys.path
                    PyRun_SimpleString("import sys");
                    PyRun_SimpleString("sys.path.append('./../python')");
                }
            }

            ~_Lifeline() {
                Py_Finalize();
            }
        private:
            static void initialize_numpy() {
                if (_import_array() < 0) {
                    throw std::runtime_error("NumPy import_array failed");
                }
            }
    };

    // Converting to python object
    static PyObject* toPyObject(int value) { return PyLong_FromLong(value); }
    static PyObject* toPyObject(double value) { return PyFloat_FromDouble(value); }
    static PyObject* toPyObject(bool value) { return PyBool_FromLong(value); }
    static PyObject* toPyObject(const char* value) { return PyUnicode_FromString(value); }
    static PyObject* toPyObject(const std::string& value) { return PyUnicode_FromString(value.c_str()); }
    static PyObject* toPyObject(const ompl::base::SE2StateSpace::StateType& state);
    static PyObject* toPyObject(ompl::geometric::PathGeometric* path);
    static PyObject* toPyObject(const Eigen::VectorXd& vector);
    // Template python conversions
    template <typename T>
    static PyObject* toPyObject(const std::vector<T>& vec) {
        PyObject* list = PyList_New(vec.size());
        for (std::size_t i = 0; i < vec.size(); ++i) {
            PyList_SET_ITEM(list, i, toPyObject(vec[i]));
        }
        return list;
    }
    template <typename T>
    static PyObject* toPyObject(const ompl::base::ScopedState<T>& state) {
        const int dimSize = state.getSpace()->getDimension();
        PyObject* list = PyList_New(dimSize);
        for (std::size_t i = 0; i < dimSize; ++i) {
            PyList_SET_ITEM(list, i, toPyObject(state[i]));
        }
        return list;
    }

    // Functions to build python tuple from C++ tuple
    template <typename Tuple, std::size_t... I>
    static PyObject* buildTupleImpl(const Tuple& t, std::index_sequence<I...>) {
        // PyTuple_Pack requires each argument as PyObject*
        return PyTuple_Pack(sizeof...(I), toPyObject(std::get<I>(t))...);
    }

    template <typename... Args>
    static PyObject* buildTuple(const std::tuple<Args...>& t) {
        return buildTupleImpl(t, std::index_sequence_for<Args...>{});
    }

    // RAII Wrapper for PyObjects and GIL Lock
    struct PyObjectRef {
        PyObject* obj;
        PyObjectRef(PyObject* o) : obj(o) {}
        ~PyObjectRef() { Py_XDECREF(obj); }
        operator PyObject*() { return obj; }
    };

    struct GILGuard {
        PyGILState_STATE state;
        GILGuard() { state = PyGILState_Ensure(); }
        ~GILGuard() { PyGILState_Release(state); }
    };


    static inline _Lifeline s_lifeline; // ensures Python is initialized
};

// Struct to store state type as well as path
