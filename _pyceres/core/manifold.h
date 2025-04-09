#pragma once

#include "_pyceres/helpers.h"
#include "_pyceres/logging.h"

#include <ceres/ceres.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

class PyManifold : public ceres::Manifold {
  /* Inherit the constructors */
  using ceres::Manifold::Manifold;
  bool Plus(const double* x,
            const double* delta,
            double* x_plus_delta) const override {
    LOG_FATAL_THROW(std::runtime_error) << "<Plus> not implemented.";
    return true;
  }

  bool PlusJacobian(const double* x, double* jacobian) const override {
    LOG_FATAL_THROW(std::runtime_error) << "<PlusJacobian> not implemented.";
    return true;
  }

  bool Minus(const double* y,
             const double* x,
             double* y_minus_x) const override {
    LOG_FATAL_THROW(std::runtime_error) << "<Minus> not implemented.";
    return true;
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    LOG_FATAL_THROW(std::runtime_error) << "<MinusJacobian> not implemented.";
    return true;
  }

  // Size of x.
  int AmbientSize() const override {
    PYBIND11_OVERLOAD_PURE_NAME(
        int,             /* Return type */
        ceres::Manifold, /* Parent class */
        "ambient_size",  /* Name of python function */
        AmbientSize      /* Name of function in C++ (must match Python name) */
    );
  }

  // Size of delta.
  int TangentSize() const override {
    PYBIND11_OVERLOAD_PURE_NAME(
        int,             /* Return type */
        ceres::Manifold, /* Parent class */
        "tangent_size",  /* Name of python function */
        TangentSize      /* Name of function in C++ (must match Python name) */
    );
  }
};

void BindManifold(py::module& m) {
  py::class_<ceres::Manifold, PyManifold /* <--- trampoline*/>(m, "Manifold")
      .def(py::init<>())
      .def("ambient_size", &ceres::Manifold::AmbientSize)
      .def("tangent_size", &ceres::Manifold::TangentSize);

  py::class_<ceres::EuclideanManifold<ceres::DYNAMIC>, ceres::Manifold>(
      m, "EuclideanManifold")
      .def(py::init<int>());
  py::class_<ceres::SubsetManifold, ceres::Manifold>(m, "SubsetManifold")
      .def(py::init<int, const std::vector<int>&>());
  py::class_<ceres::QuaternionManifold, ceres::Manifold>(m,
                                                         "QuaternionManifold")
      .def(py::init<>());
  py::class_<ceres::EigenQuaternionManifold, ceres::Manifold>(
      m, "EigenQuaternionManifold")
      .def(py::init<>());
  py::class_<ceres::SphereManifold<ceres::DYNAMIC>, ceres::Manifold>(
      m, "SphereManifold")
      .def(py::init<int>());
}
