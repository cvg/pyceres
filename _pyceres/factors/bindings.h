#pragma once

#include "_pyceres/helpers.h"

#include <ceres/ceres.h>
#include <ceres/normal_prior.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

inline Eigen::MatrixXd SqrtInformation(const Eigen::MatrixXd& covariance) {
  return covariance.inverse().llt().matrixL();
}

class NormalError {
 public:
  explicit NormalError(const Eigen::MatrixXd& covariance)
      : sqrt_information_(SqrtInformation(covariance)),
        dimension_(covariance.rows()) {
    THROW_CHECK_EQ(covariance.rows(), covariance.cols());
  }

  static ceres::CostFunction* Create(const Eigen::MatrixXd& covariance) {
    auto* cost_function = new ceres::DynamicAutoDiffCostFunction<NormalError>(
        new NormalError(covariance));
    const int dimension = covariance.rows();
    cost_function->AddParameterBlock(dimension);
    cost_function->AddParameterBlock(dimension);
    cost_function->SetNumResiduals(dimension);
    return cost_function;
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals_ptr) const {
    for (int i = 0; i < dimension_; ++i) {
      residuals_ptr[i] = parameters[0][i] - parameters[1][i];
    }
    Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> residuals(residuals_ptr,
                                                              dimension_);
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());
    return true;
  }

 private:
  const Eigen::MatrixXd sqrt_information_;
  const int dimension_;
};

void BindFactors(py::module& m) {
  m.def(
      "NormalPrior",
      [](const Eigen::VectorXd& mean,
         const Eigen::MatrixXd& covariance) -> ceres::CostFunction* {
        THROW_CHECK_EQ(covariance.cols(), mean.size());
        THROW_CHECK_EQ(covariance.cols(), covariance.rows());
        return new ceres::NormalPrior(SqrtInformation(covariance), mean);
      },
      py::arg("mean"),
      py::arg("covariance"));

  m.def("NormalError", &NormalError::Create, py::arg("covariance"));
}
