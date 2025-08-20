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
  const size_t n_values = crsMatrix.values.size();
  py::array_t<int> rows(n_values), cols(n_values);
  py::array_t<double> values(n_values);

  int* const rows_data = static_cast<int*>(rows.request().ptr);
  int* const cols_data = static_cast<int*>(cols.request().ptr);
  double* const values_data = static_cast<double*>(values.request().ptr);

  int counter = 0;
  for (int row = 0; row < crsMatrix.num_rows; ++row) {
    for (int k = crsMatrix.rows[row]; k < crsMatrix.rows[row + 1]; ++k) {
      rows_data[counter] = row;
      cols_data[counter] = crsMatrix.cols[k];
      values_data[counter] = crsMatrix.values[k];
      counter++;
    }
  }

  // return as a tuple
  return py::make_tuple(rows, cols, values);
}
}  // namespace

void BindCRSMatrix(py::module& m) {
  using CRSMatrix = ceres::CRSMatrix;
  py::classh<CRSMatrix> PyCRSMatrix(m, "CRSMatrix");
  PyCRSMatrix.def(py::init<>())
      .def_readonly("num_rows", &CRSMatrix::num_rows)
      .def_readonly("num_cols", &CRSMatrix::num_cols)
      .def_readonly("rows", &CRSMatrix::rows)
      .def_readonly("cols", &CRSMatrix::cols)
      .def_readonly("values", &CRSMatrix::values)
      .def("to_tuple", &ConvertCRSToPyTuple);
}
