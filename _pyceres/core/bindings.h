#pragma once

#include "_pyceres/core/callbacks.h"
#include "_pyceres/core/cost_functions.h"
#include "_pyceres/core/covariance.h"
#include "_pyceres/core/loss_functions.h"
#include "_pyceres/core/manifold.h"
#include "_pyceres/core/problem.h"
#include "_pyceres/core/solver.h"
#include "_pyceres/core/types.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_core(py::module& m) {
  init_types(m);
  init_callbacks(m);
  init_covariance(m);
  init_solver(m);
  init_loss_functions(m);
  init_cost_functions(m);
  init_manifold(m);
  init_problem(m);
}
