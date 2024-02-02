#pragma once

#include "_pyceres/helpers.h"
#include "_pyceres/log_exceptions.h"

#include <ceres/ceres.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

void BindCovariance(py::module& m) {
  using c_options = ceres::Covariance::Options;
  auto co = py::class_<ceres::Covariance::Options>(m, "CovarianceOptions")
                .def(py::init<>())
                .def_property(
                    "num_threads",
                    [](const c_options& self) { return self.num_threads; },
                    [](c_options& self, int n_threads) {
                      int effective_n_threads =
                          GetEffectiveNumThreads(n_threads);
                      self.num_threads = effective_n_threads;
                    })
                .def_readwrite("sparse_linear_algebra_library_type",
                               &c_options::sparse_linear_algebra_library_type)
                .def_readwrite("algorithm_type", &c_options::algorithm_type)
                .def_readwrite("min_reciprocal_condition_number",
                               &c_options::min_reciprocal_condition_number)
                .def_readwrite("null_space_rank", &c_options::null_space_rank)
                .def_readwrite("apply_loss_function",
                               &c_options::apply_loss_function);
  make_dataclass(co);

  py::class_<ceres::Covariance>(m, "Covariance")
      .def(py::init<ceres::Covariance::Options>())
      .def(
          "compute",
          [](ceres::Covariance& self,
             std::vector<std::pair<py::array_t<double>, py::array_t<double>>>&
                 blocks,
             ceres::Problem& problem) {
            std::vector<std::pair<const double*, const double*>> pointer_values;
            for (int i = 0; i < blocks.size(); ++i) {
              const double* ptr1 =
                  static_cast<double*>(blocks[i].first.request().ptr);
              const double* ptr2 =
                  static_cast<double*>(blocks[i].second.request().ptr);
              THROW_CHECK(problem.HasParameterBlock(ptr1));
              THROW_CHECK(problem.HasParameterBlock(ptr2));
              pointer_values.emplace_back(ptr1, ptr2);
            }
            py::gil_scoped_release release;
            return self.Compute(pointer_values, &problem);
          },
          py::arg("blocks").noconvert(),
          py::arg("problem"),
          py::keep_alive<1, 2>(),
          py::keep_alive<1, 3>())
      .def(
          "get_covariance_block",
          [](ceres::Covariance& self,
             py::array_t<double>& block1,
             py::array_t<double>& block2) -> py::object {
            // Get shape of parameter 1
            py::buffer_info info1 = block1.request();
            ssize_t dim1 = 1;
            for (ssize_t s : info1.shape) {
              dim1 *= s;
            }
            // Get shape of parameter 2
            py::buffer_info info2 = block2.request();
            ssize_t dim2 = 1;
            for (ssize_t s : info2.shape) {
              dim2 *= s;
            }
            Eigen::Matrix<double, -1, -1, Eigen::RowMajor> covariance(dim1,
                                                                      dim2);
            bool success =
                self.GetCovarianceBlock(static_cast<double*>(info1.ptr),
                                        static_cast<double*>(info2.ptr),
                                        covariance.data());
            return success ? py::cast(covariance) : py::none();
          },
          py::arg("block1").noconvert(),
          py::arg("block2").noconvert())
      .def(
          "get_covariance_block_tangent_space",
          [](ceres::Covariance& self,
             py::array_t<double>& block1,
             py::array_t<double>& block2,
             const ceres::Problem& problem) -> py::object {
            // Get buffer and raw pointer to block 1
            py::buffer_info info1 = block1.request();
            const double* ptr1 = static_cast<double*>(info1.ptr);
            // Use size of tangent space if local parameterization, otherwise
            // get param shape
            const ceres::Manifold* param1 = problem.GetManifold(ptr1);
            ssize_t dim1 = 1;
            if (param1 != nullptr) {
              dim1 = param1->TangentSize();
            } else {
              for (ssize_t s : info1.shape) {
                dim1 *= s;
              }
            }
            // Identically for block 2
            py::buffer_info info2 = block2.request();
            const double* ptr2 = static_cast<double*>(info2.ptr);
            const ceres::Manifold* param2 = problem.GetManifold(ptr2);
            ssize_t dim2 = 1;
            if (param2 != nullptr) {
              dim2 = param2->TangentSize();
            } else {
              for (ssize_t s : info2.shape) {
                dim2 *= s;
              }
            }
            Eigen::Matrix<double, -1, -1, Eigen::RowMajor> covariance(dim1,
                                                                      dim2);
            bool success = self.GetCovarianceBlockInTangentSpace(
                ptr1, ptr2, covariance.data());
            return success ? py::cast(covariance) : py::none();
          },
          py::arg("block1").noconvert(),
          py::arg("block2").noconvert(),
          py::arg("problem"));
}
