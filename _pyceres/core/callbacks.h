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

void init_callbacks(py::module& m) {
  py::class_<ceres::IterationCallback,
             PyIterationCallback /* <--- trampoline*/>(m, "IterationCallback")
      .def(py::init<>())
      .def("__call__", &ceres::IterationCallback::operator());

  py::class_<ceres::EvaluationCallback,
             PyEvaluationCallBack /* <--- trampoline*/>(m, "EvaluationCallback")
      .def(py::init<>());

  auto vec_it_cb = py::bind_vector<std::vector<ceres::IterationCallback*>>(
      m, "ListIterationCallback");

  vec_it_cb.def(
      py::init<>([](py::list list) {
        std::vector<ceres::IterationCallback*> callbacks;
        for (auto& handle : list) {
          callbacks.push_back(handle.cast<ceres::IterationCallback*>());
        }
        return callbacks;
      }),
      py::keep_alive<1, 2>());
  py::implicitly_convertible<py::list,
                             std::vector<ceres::IterationCallback*>>();
}
