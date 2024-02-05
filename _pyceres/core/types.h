#pragma once

#include "_pyceres/helpers.h"
#include "_pyceres/logging.h"

#include <ceres/ceres.h>
#include <ceres/normal_prior.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void BindTypes(py::module& m) {
  auto ownt = py::enum_<ceres::Ownership>(m, "Ownership")
                  .value("DO_NOT_TAKE_OWNERSHIP",
                         ceres::Ownership::DO_NOT_TAKE_OWNERSHIP)
                  .value("TAKE_OWNERSHIP", ceres::Ownership::TAKE_OWNERSHIP)
                  .export_values();
  AddStringToEnumConstructor(ownt);

  auto mt = py::enum_<ceres::MinimizerType>(m, "MinimizerType")
                .value("LINE_SEARCH", ceres::MinimizerType::LINE_SEARCH)
                .value("TRUST_REGION", ceres::MinimizerType::TRUST_REGION);
  AddStringToEnumConstructor(mt);

  auto linesearcht = py::enum_<ceres::LineSearchType>(m, "LineSearchType")
                         .value("ARMIJO", ceres::LineSearchType::ARMIJO)
                         .value("WOLFE", ceres::LineSearchType::WOLFE);
  AddStringToEnumConstructor(linesearcht);

  auto lsdt =
      py::enum_<ceres::LineSearchDirectionType>(m, "LineSearchDirectionType")
          .value("BFGS", ceres::LineSearchDirectionType::BFGS)
          .value("LBFGS", ceres::LineSearchDirectionType::LBFGS)
          .value("NONLINEAR_CONJUGATE_GRADIENT",
                 ceres::LineSearchDirectionType::NONLINEAR_CONJUGATE_GRADIENT)
          .value("STEEPEST_DESCENT",
                 ceres::LineSearchDirectionType::STEEPEST_DESCENT);
  AddStringToEnumConstructor(lsdt);

  auto lsit =
      py::enum_<ceres::LineSearchInterpolationType>(
          m, "LineSearchInterpolationType")
          .value("BISECTION", ceres::LineSearchInterpolationType::BISECTION)
          .value("CUBIC", ceres::LineSearchInterpolationType::CUBIC)
          .value("QUADRATIC", ceres::LineSearchInterpolationType::QUADRATIC);
  AddStringToEnumConstructor(lsit);

  auto ncgt =
      py::enum_<ceres::NonlinearConjugateGradientType>(
          m, "NonlinearConjugateGradientType")
          .value("FLETCHER_REEVES",
                 ceres::NonlinearConjugateGradientType::FLETCHER_REEVES)
          .value("HESTENES_STIEFEL",
                 ceres::NonlinearConjugateGradientType::HESTENES_STIEFEL)
          .value("POLAK_RIBIERE",
                 ceres::NonlinearConjugateGradientType::POLAK_RIBIERE);
  AddStringToEnumConstructor(ncgt);

  auto linsolt =
      py::enum_<ceres::LinearSolverType>(m, "LinearSolverType")
          .value("DENSE_NORMAL_CHOLESKY",
                 ceres::LinearSolverType::DENSE_NORMAL_CHOLESKY)
          .value("DENSE_QR", ceres::LinearSolverType::DENSE_QR)
          .value("SPARSE_NORMAL_CHOLESKY",
                 ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY)
          .value("DENSE_SCHUR", ceres::LinearSolverType::DENSE_SCHUR)
          .value("SPARSE_SCHUR", ceres::LinearSolverType::SPARSE_SCHUR)
          .value("ITERATIVE_SCHUR", ceres::LinearSolverType::ITERATIVE_SCHUR)
          .value("CGNR", ceres::LinearSolverType::CGNR);
  AddStringToEnumConstructor(linsolt);

  auto dogt =
      py::enum_<ceres::DoglegType>(m, "DoglegType")
          .value("TRADITIONAL_DOGLEG", ceres::DoglegType::TRADITIONAL_DOGLEG)
          .value("SUBSPACE_DOGLEG", ceres::DoglegType::SUBSPACE_DOGLEG);
  AddStringToEnumConstructor(dogt);

  auto trst =
      py::enum_<ceres::TrustRegionStrategyType>(m, "TrustRegionStrategyType")
          .value("LEVENBERG_MARQUARDT",
                 ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT)
          .value("DOGLEG", ceres::TrustRegionStrategyType::DOGLEG);
  AddStringToEnumConstructor(trst);

  auto prt =
      py::enum_<ceres::PreconditionerType>(m, "PreconditionerType")
          .value("IDENTITY", ceres::PreconditionerType::IDENTITY)
          .value("JACOBI", ceres::PreconditionerType::JACOBI)
          .value("SCHUR_JACOBI", ceres::PreconditionerType::SCHUR_JACOBI)
          .value("CLUSTER_JACOBI", ceres::PreconditionerType::CLUSTER_JACOBI)
          .value("CLUSTER_TRIDIAGONAL",
                 ceres::PreconditionerType::CLUSTER_TRIDIAGONAL);
  // .value("SUBSET", ceres::PreconditionerType::SUBSET);
  AddStringToEnumConstructor(prt);

  auto vct =
      py::enum_<ceres::VisibilityClusteringType>(m, "VisibilityClusteringType")
          .value("CANONICAL_VIEWS",
                 ceres::VisibilityClusteringType::CANONICAL_VIEWS)
          .value("SINGLE_LINKAGE",
                 ceres::VisibilityClusteringType::SINGLE_LINKAGE);
  AddStringToEnumConstructor(vct);

  auto dlalt =
      py::enum_<ceres::DenseLinearAlgebraLibraryType>(
          m, "DenseLinearAlgebraLibraryType")
          .value("EIGEN", ceres::DenseLinearAlgebraLibraryType::EIGEN)
          .value("LAPACK", ceres::DenseLinearAlgebraLibraryType::LAPACK)
          .value("CUDA", ceres::DenseLinearAlgebraLibraryType::CUDA);
  AddStringToEnumConstructor(dlalt);

  auto slalt =
      py::enum_<ceres::SparseLinearAlgebraLibraryType>(
          m, "SparseLinearAlgebraLibraryType")
          .value("SUITE_SPARSE",
                 ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE)
          .value("EIGEN_SPARSE",
                 ceres::SparseLinearAlgebraLibraryType::EIGEN_SPARSE)
          .value("ACCELERATE_SPARSE",
                 ceres::SparseLinearAlgebraLibraryType::ACCELERATE_SPARSE)
          //   .value("CUDA_SPARSE",
          //           ceres::SparseLinearAlgebraLibraryType::CUDA_SPARSE)
          .value("NO_SPARSE", ceres::SparseLinearAlgebraLibraryType::NO_SPARSE);
  AddStringToEnumConstructor(slalt);

  auto logt = py::enum_<ceres::LoggingType>(m, "LoggingType")
                  .value("SILENT", ceres::LoggingType::SILENT)
                  .value("PER_MINIMIZER_ITERATION",
                         ceres::LoggingType::PER_MINIMIZER_ITERATION);
  AddStringToEnumConstructor(logt);

  auto cat =
      py::enum_<ceres::CovarianceAlgorithmType>(m, "CovarianceAlgorithmType")
          .value("DENSE_SVD", ceres::CovarianceAlgorithmType::DENSE_SVD)
          .value("SPARSE_QR", ceres::CovarianceAlgorithmType::SPARSE_QR);
  AddStringToEnumConstructor(cat);

  auto cbrt =
      py::enum_<ceres::CallbackReturnType>(m, "CallbackReturnType")
          .value("SOLVER_CONTINUE", ceres::CallbackReturnType::SOLVER_CONTINUE)
          .value("SOLVER_ABORT", ceres::CallbackReturnType::SOLVER_ABORT)
          .value("SOLVER_TERMINATE_SUCCESSFULLY",
                 ceres::CallbackReturnType::SOLVER_TERMINATE_SUCCESSFULLY);
  AddStringToEnumConstructor(cbrt);

  auto dft = py::enum_<ceres::DumpFormatType>(m, "DumpFormatType")
                 .value("CONSOLE", ceres::DumpFormatType::CONSOLE)
                 .value("TEXTFILE", ceres::DumpFormatType::TEXTFILE);
  AddStringToEnumConstructor(dft);

  auto termt =
      py::enum_<ceres::TerminationType>(m, "TerminationType")
          .value("CONVERGENCE", ceres::TerminationType::CONVERGENCE)
          .value("NO_CONVERGENCE", ceres::TerminationType::NO_CONVERGENCE)
          .value("FAILURE", ceres::TerminationType::FAILURE)
          .value("USER_SUCCESS", ceres::TerminationType::USER_SUCCESS)
          .value("USER_FAILURE", ceres::TerminationType::USER_FAILURE);
  AddStringToEnumConstructor(termt);
}
