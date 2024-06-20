#pragma once

#include "_pyceres/core/callbacks.h"
#include "_pyceres/core/cost_functions.h"
#include "_pyceres/core/covariance.h"
#include "_pyceres/core/crs_matrix.h"
#include "_pyceres/core/loss_functions.h"
#include "_pyceres/core/manifold.h"
#include "_pyceres/core/problem.h"
#include "_pyceres/core/solver.h"
#include "_pyceres/core/types.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;

void BindCore(py::module& m) {
  BindTypes(m);
  BindCallbacks(m);
  BindCovariance(m);
  BindCRSMatrix(m);
  BindSolver(m);
  BindLossFunctions(m);
  BindCostFunctions(m);
  BindManifold(m);
  BindProblem(m);
}
