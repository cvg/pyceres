#pragma once

#include <ceres/ceres.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;

// Trampoline class so we can create an EvaluationCallback in Python.
class PyEvaluationCallBack : public ceres::EvaluationCallback {
 public:
  /* Inherit the constructors */
  using ceres::EvaluationCallback::EvaluationCallback;

  void PrepareForEvaluation(bool evaluate_jacobians,
                            bool new_evaluation_point) override {
    PYBIND11_OVERLOAD_PURE(
        void,                      /* Return type */
        ceres::EvaluationCallback, /* Parent class */
        PrepareForEvaluation,      // Name of function in C++ (fn)
        evaluate_jacobians,
        new_evaluation_point /* Argument(s) */
    );
  }
};

class PyIterationCallback : public ceres::IterationCallback {
 public:
  using ceres::IterationCallback::IterationCallback;

  ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary) override {
    PYBIND11_OVERRIDE_PURE_NAME(
        ceres::CallbackReturnType,  // Return type (ret_type)
        ceres::IterationCallback,   // Parent class (cname)
        "__call__",                 // Name of method in Python (name)
        operator(),                 // Name of function in C++ (fn)
        summary);
  }
};

PYBIND11_MAKE_OPAQUE(std::vector<ceres::IterationCallback*>);

void BindCallbacks(py::module& m) {
  py::class_<ceres::EvaluationCallback,
             PyEvaluationCallBack /* <--- trampoline*/>(m, "EvaluationCallback")
      .def(py::init<>());
}
