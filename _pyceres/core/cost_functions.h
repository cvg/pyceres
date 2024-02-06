#pragma once

#include "_pyceres/helpers.h"
#include "_pyceres/logging.h"

#include <ceres/ceres.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

// Class which we can use to create a ceres::CostFunction in python.
// This allows use to create python based cost functions.
class PyCostFunction : public ceres::CostFunction {
 public:
  // Inherit the constructors.
  using ceres::CostFunction::CostFunction;
  using ceres::CostFunction::set_num_residuals;

  bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const override {
    pybind11::gil_scoped_acquire gil;

    // Resize the vectors passed to python to the proper size. And set the
    // pointer values
    if (!cached_flag) {
      parameters_vec.reserve(this->parameter_block_sizes().size());
      jacobians_vec.reserve(this->parameter_block_sizes().size());
      residuals_wrap = py::array_t<double>(num_residuals(), residuals, no_copy);
      for (size_t idx = 0; idx < parameter_block_sizes().size(); ++idx) {
        parameters_vec.emplace_back(py::array_t<double>(
            this->parameter_block_sizes()[idx], parameters[idx], no_copy));
        jacobians_vec.emplace_back(py::array_t<double>(
            this->parameter_block_sizes()[idx] * num_residuals(),
            jacobians[idx],
            no_copy));
      }
      cached_flag = true;
    }

    // Check if the pointers have changed and if they have then change them
    auto info = residuals_wrap.request(true);
    if (info.ptr != residuals) {
      residuals_wrap = py::array_t<double>(num_residuals(), residuals, no_copy);
    }
    info = parameters_vec[0].request(true);
    if (info.ptr != parameters) {
      for (size_t idx = 0; idx < parameters_vec.size(); ++idx) {
        parameters_vec[idx] = py::array_t<double>(
            this->parameter_block_sizes()[idx], parameters[idx], no_copy);
      }
    }
    if (jacobians) {
      info = jacobians_vec[0].request(true);
      if (info.ptr != jacobians) {
        for (size_t idx = 0; idx < jacobians_vec.size(); ++idx) {
          jacobians_vec[idx] = py::array_t<double>(
              this->parameter_block_sizes()[idx] * num_residuals(),
              jacobians[idx],
              no_copy);
        }
      }
    }

    pybind11::function overload = pybind11::get_overload(
        static_cast<const ceres::CostFunction*>(this), "Evaluate");
    if (overload) {
      auto o = overload.operator()<pybind11::return_value_policy::reference>(
          parameters_vec,
          residuals_wrap,
          jacobians ? &jacobians_vec : nullptr);  // nullptr gets cast to None
      return pybind11::detail::cast_safe<bool>(std::move(o));
    }
    pybind11::pybind11_fail("Tried to call pure virtual function \"" PYBIND11_STRINGIFY(
        Ceres::CostFunction) "::"
                             "Evaluate \"");
  }

 private:
  // Vectors used to pass double pointers to python as pybind does not wrap
  // double pointers(**) like Ceres uses.
  // Mutable so they can be modified by the const function.
  mutable std::vector<py::array_t<double>> parameters_vec;
  mutable std::vector<py::array_t<double>> jacobians_vec;
  // Flag used to determine if the vectors need to be resized.
  mutable bool cached_flag = false;
  // Buffer to contain the residuals pointer.
  mutable py::array_t<double> residuals_wrap;
  // Dummy variable for pybind11 to avoid a copy.
  mutable py::str no_copy;
};

void BindCostFunctions(py::module& m) {
  py::class_<ceres::CostFunction, PyCostFunction /* <--- trampoline*/>(
      m, "CostFunction")
      .def(py::init<>())
      .def("num_residuals", &ceres::CostFunction::num_residuals)
      .def("num_parameter_blocks",
           [](ceres::CostFunction& myself) {
             return myself.parameter_block_sizes().size();
           })
      .def("parameter_block_sizes",
           &ceres::CostFunction::parameter_block_sizes,
           py::return_value_policy::reference)
      .def("set_num_residuals", &PyCostFunction::set_num_residuals)
      .def("set_parameter_block_sizes",
           [](ceres::CostFunction& myself, std::vector<int32_t>& sizes) {
             for (auto s : sizes) {
               const_cast<std::vector<int32_t>&>(myself.parameter_block_sizes())
                   .push_back(s);
             }
           })
      .def(
          "evaluate",
          [](ceres::CostFunction& self, const py::args& parameters) {
            THROW_CHECK_EQ(parameters.size(),
                           self.parameter_block_sizes().size());
            std::vector<double*> params(parameters.size());
            Eigen::Matrix<double, -1, -1, Eigen::RowMajor> residuals(
                self.num_residuals(), 1);
            std::vector<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>
                jacobians;
            for (int i = 0; i < parameters.size(); i++) {
              py::buffer_info param_buf =
                  parameters[i]
                      .cast<py::array_t<double, py::array::c_style>>()
                      .request();
              params[i] = static_cast<double*>(param_buf.ptr);
              ssize_t num_dims = 1;
              std::vector<ssize_t> param_shape = param_buf.shape;
              for (int k = 0; k < param_shape.size(); k++) {
                num_dims *= param_shape[k];
              }
              THROW_CHECK_EQ(num_dims, self.parameter_block_sizes()[i]);
              jacobians.push_back(
                  Eigen::Matrix<double, -1, -1, Eigen::RowMajor>(
                      self.num_residuals(), num_dims));
            }
            std::vector<double*> jacobian_ptrs;
            for (int i = 0; i < parameters.size(); i++) {
              jacobian_ptrs.push_back(jacobians[i].data());
            }

            bool success = self.Evaluate(
                params.data(), residuals.data(), jacobian_ptrs.data());
            if (success) {
              return py::make_tuple(residuals, jacobians);
            } else {
              return py::make_tuple(py::none(), py::none());
            }
          },
          py::keep_alive<1, 2>());
}
