#pragma once

#include "_pyceres/helpers.h"

#include <ceres/ceres.h>
#include <ceres/normal_prior.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_factors(py::module& m) {
  m.def(
      "NormalPrior",
      [](const Eigen::VectorXd& mean,
         const Eigen::Matrix<double, -1, -1>& covariance) {
        THROW_CHECK_EQ(covariance.cols(), mean.size());
        THROW_CHECK_EQ(covariance.cols(), covariance.rows());
        return new ceres::NormalPrior(covariance.inverse().llt().matrixL(),
                                      mean);
      },
      py::arg("mean"),
      py::arg("covariance"));
}
