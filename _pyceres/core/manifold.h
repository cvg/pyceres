// Ceres Solver Python Bindings
// Copyright Nikolaus Mitchell. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the copyright holder nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: nikolausmitchell@gmail.com (Nikolaus Mitchell)
// Edited by: philipp.lindenberger@math.ethz.ch (Philipp Lindenberger)

#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <ceres/ceres.h>

#include "_pyceres/helpers.h"
#include "_pyceres/log_exceptions.h"

class PyManifold : public ceres::Manifold {
  /* Inherit the constructors */
  using ceres::Manifold::Manifold;
  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
    THROW_EXCEPTION(std::runtime_error, "<Plus> not implemented.");
    return true;
  }

  bool PlusJacobian(const double* x, double* jacobian) const override {
    THROW_EXCEPTION(std::runtime_error, "<PlusJacobian> not implemented.");
  }

  bool Minus(const double* y, const double* x, double* y_minus_x) const override {
    THROW_EXCEPTION(std::runtime_error, "<Minus> not implemented.");
    return true;
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    THROW_EXCEPTION(std::runtime_error, "<MinusJacobian> not implemented.");
  }

  // Size of x.
  int AmbientSize() const override {
    PYBIND11_OVERLOAD_PURE_NAME(
        int,                          /* Return type */
        ceres::Manifold, /* Parent class */
        "ambient_size",                /* Name of python function */
        AmbientSize /* Name of function in C++ (must match Python name) */
    );
  }

  // Size of delta.
  int TangentSize() const override {
    PYBIND11_OVERLOAD_PURE_NAME(
        int,                          /* Return type */
        ceres::Manifold, /* Parent class */
        "tangent_size",                 /* Name of python function */
        TangentSize /* Name of function in C++ (must match Python name) */
    );
  }
};

using namespace ceres;

void init_manifold(py::module& m) {
  py::class_<Manifold, PyManifold /* <--- trampoline*/>(
      m, "Manifold")
      .def(py::init<>())
      .def("ambient_size", &Manifold::AmbientSize)
      .def("tangent_size", &Manifold::TangentSize);

  py::class_<EuclideanManifold<DYNAMIC>, Manifold>(m,
      "EuclideanManifold")
      .def(py::init<int>());
  py::class_<SubsetManifold, Manifold>(m,
      "SubsetManifold")
      .def(py::init<int, const std::vector<int>&>());
  py::class_<QuaternionManifold, Manifold>(
      m, "QuaternionManifold")
      .def(py::init<>());
  py::class_<EigenQuaternionManifold, Manifold>(
      m, "EigenQuaternionManifold")
      .def(py::init<>());
  py::class_<SphereManifold<DYNAMIC>, Manifold>(
      m, "SphereManifold")
      .def(py::init<int>());
}
