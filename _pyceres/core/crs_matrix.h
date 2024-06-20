#pragma once

#include "_pyceres/core/wrappers.h"
#include "_pyceres/helpers.h"
#include "_pyceres/logging.h"

#include <Eigen/Sparse>
#include <ceres/ceres.h>
#include <ceres/crs_matrix.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace {
py::tuple ConvertCRSToPyTuple(const ceres::CRSMatrix& crsMatrix) {
  std::vector<int> rows, cols;
  std::vector<double> values;

  for (int row = 0; row < crsMatrix.num_rows; ++row) {
    for (int k = crsMatrix.rows[row]; k < crsMatrix.rows[row + 1]; ++k) {
      rows.push_back(row);
      cols.push_back(crsMatrix.cols[k]);
      values.push_back(crsMatrix.values[k]);
    }
  }

  // return as a tuple
  return py::make_tuple(rows, cols, values);
}
}  // namespace

void BindCRSMatrix(py::module& m) {
  using CRSMatrix = ceres::CRSMatrix;
  py::class_<CRSMatrix> PyCRSMatrix(m, "CRSMatrix");
  PyCRSMatrix.def(py::init<>())
      .def_readonly("num_rows", &CRSMatrix::num_rows)
      .def_readonly("num_cols", &CRSMatrix::num_cols)
      .def_readonly("rows", &CRSMatrix::rows)
      .def_readonly("cols", &CRSMatrix::cols)
      .def_readonly("values", &CRSMatrix::values)
      .def("to_tuple",
           [](CRSMatrix& self) { return ConvertCRSToPyTuple(self); });
}
