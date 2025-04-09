#pragma once

#include "_pyceres/core/wrappers.h"
#include "_pyceres/helpers.h"
#include "_pyceres/logging.h"

#include <ceres/ceres.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace {

// Set residual blocks for Ceres::Problem::EvaluateOptions
void SetResidualBlocks(
    ceres::Problem::EvaluateOptions& self,
    std::vector<ResidualBlockIDWrapper>& residual_block_ids) {
  self.residual_blocks.clear();
  self.residual_blocks.reserve(residual_block_ids.size());
  for (auto it = residual_block_ids.begin(); it != residual_block_ids.end();
       ++it) {
    self.residual_blocks.push_back(it->id);
  }
}

}  // namespace

// Function to create Problem::Options with DO_NOT_TAKE_OWNERSHIP
// This is cause we want Python to manage our memory not Ceres
ceres::Problem::Options CreateProblemOptions() {
  ceres::Problem::Options o;
  o.manifold_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  o.loss_function_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  o.cost_function_ownership = ceres::Ownership::TAKE_OWNERSHIP;
  return o;
}

// Function to create a ceres Problem with the default options that Ceres does
// NOT take ownership. Needed since Python expects to own the memory.
std::unique_ptr<ceres::Problem> CreatePythonProblem() {
  ceres::Problem::Options options = CreateProblemOptions();
  return std::unique_ptr<ceres::Problem>(new ceres::Problem(options));
}

void BindProblem(py::module& m) {
  using options = ceres::Problem::Options;
  py::class_<ceres::Problem::Options>(m, "ProblemOptions")
      .def(py::init(&CreateProblemOptions))  // Ensures default is that
                                             // Python manages memory
      .def_readonly("cost_function_ownership",
                    &options::cost_function_ownership)
      .def_readonly("loss_function_ownership",
                    &options::loss_function_ownership)
      .def_readonly("manifold_ownership", &options::manifold_ownership)
      .def_readwrite("enable_fast_removal", &options::enable_fast_removal)
      .def_readwrite("disable_all_safety_checks",
                     &options::disable_all_safety_checks);

  py::class_<ceres::Problem::EvaluateOptions>(m, "EvaluateOptions")
      .def(py::init<>())
      .def(
          "set_parameter_blocks",
          [](ceres::Problem::EvaluateOptions& self,
             std::vector<py::array_t<double>>& blocks) {
            self.parameter_blocks.clear();
            self.parameter_blocks.reserve(blocks.size());
            for (auto it = blocks.begin(); it != blocks.end(); ++it) {
              py::buffer_info info = it->request();
              self.parameter_blocks.push_back(static_cast<double*>(info.ptr));
            }
          },
          py::arg("parameter_blocks"))
      .def("set_residual_blocks",
           &SetResidualBlocks,
           py::arg("residual_block_ids"))
      .def_readwrite("apply_loss_function",
                     &ceres::Problem::EvaluateOptions::apply_loss_function)
      .def_readwrite("num_threads",
                     &ceres::Problem::EvaluateOptions::num_threads);

  py::class_<ResidualBlockIDWrapper> residual_block_wrapper(m, "ResidualBlock");

  py::class_<ceres::Problem, std::shared_ptr<ceres::Problem>>(m, "Problem")
      .def(py::init(&CreatePythonProblem))
      .def(py::init<ceres::Problem::Options>())
      .def("num_parameter_blocks", &ceres::Problem::NumParameterBlocks)
      .def("num_parameters", &ceres::Problem::NumParameters)
      .def("num_residual_blocks", &ceres::Problem::NumResidualBlocks)
      .def("num_residuals", &ceres::Problem::NumResiduals)
      .def("parameter_block_size", &ceres::Problem::ParameterBlockSize)
      .def(
          "set_parameter_block_constant",
          [](ceres::Problem& self, py::array_t<double>& values) {
            py::buffer_info info = values.request();
            THROW_CHECK(self.HasParameterBlock((double*)info.ptr));
            self.SetParameterBlockConstant((double*)info.ptr);
          },
          py::arg("values").noconvert())
      .def(
          "set_parameter_block_variable",
          [](ceres::Problem& self, py::array_t<double>& values) {
            py::buffer_info info = values.request();
            THROW_CHECK(self.HasParameterBlock((double*)info.ptr));
            self.SetParameterBlockVariable((double*)info.ptr);
          },
          py::arg("values").noconvert())
      .def(
          "is_parameter_block_constant",
          [](ceres::Problem& self, py::array_t<double>& values) {
            py::buffer_info info = values.request();
            THROW_CHECK(self.HasParameterBlock((double*)info.ptr));
            return self.IsParameterBlockConstant((double*)info.ptr);
          },
          py::arg("values").noconvert())
      .def(
          "set_parameter_lower_bound",
          [](ceres::Problem& self,
             py::array_t<double>& values,
             int index,
             double lower_bound) {
            py::buffer_info info = values.request();
            THROW_CHECK(self.HasParameterBlock((double*)info.ptr));
            self.SetParameterLowerBound((double*)info.ptr, index, lower_bound);
          },
          py::arg("values").noconvert(),
          py::arg("index"),
          py::arg("lower_bound"))
      .def(
          "set_parameter_upper_bound",
          [](ceres::Problem& self,
             py::array_t<double>& values,
             int index,
             double upper_bound) {
            py::buffer_info info = values.request();
            THROW_CHECK(self.HasParameterBlock((double*)info.ptr));
            self.SetParameterUpperBound((double*)info.ptr, index, upper_bound);
          },
          py::arg("values").noconvert(),
          py::arg("index"),
          py::arg("upper_bound"))
      .def("get_parameter_lower_bound",
           [](ceres::Problem& self, py::array_t<double>& np_arr, int index) {
             py::buffer_info info = np_arr.request();
             return self.GetParameterLowerBound((double*)info.ptr, index);
           })
      .def("get_parameter_upper_bound",
           [](ceres::Problem& self, py::array_t<double>& np_arr, int index) {
             py::buffer_info info = np_arr.request();
             return self.GetParameterUpperBound((double*)info.ptr, index);
           })
      .def(
          "has_manifold",
          [](ceres::Problem& self, py::array_t<double>& values) {
            py::buffer_info info = values.request();
            THROW_CHECK(self.HasParameterBlock((double*)info.ptr));
            return self.HasManifold((double*)info.ptr);
          },
          py::arg("values").noconvert())
      .def(
          "get_manifold",
          [](ceres::Problem& self, py::array_t<double>& values) {
            py::buffer_info info = values.request();
            THROW_CHECK(self.HasParameterBlock((double*)info.ptr));
            return self.GetManifold((double*)info.ptr);
          },
          py::arg("values").noconvert())
      .def(
          "set_manifold",
          [](ceres::Problem& self,
             py::array_t<double>& values,
             ceres::Manifold* manifold) {
            py::buffer_info info = values.request();
            THROW_CHECK(self.HasParameterBlock((double*)info.ptr));
            ceres::Manifold* paramw = new ManifoldWrapper(manifold);
            self.SetManifold((double*)info.ptr, paramw);
          },
          py::arg("values").noconvert(),
          py::arg("manifold"),
          py::keep_alive<1, 3>())  // LocalParameterization
      .def(
          "parameter_block_size",
          [](ceres::Problem& self, py::array_t<double>& values) {
            py::buffer_info info = values.request();
            THROW_CHECK(self.HasParameterBlock((double*)info.ptr));
            return self.ParameterBlockSize((double*)info.ptr);
          },
          py::arg("values").noconvert())
      .def(
          "parameter_block_tangent_size",
          [](ceres::Problem& self, py::array_t<double>& values) {
            py::buffer_info info = values.request();
            THROW_CHECK(self.HasParameterBlock((double*)info.ptr));
            return self.ParameterBlockTangentSize((double*)info.ptr);
          },
          py::arg("values").noconvert())
      .def(
          "has_parameter_block",
          [](ceres::Problem& self, py::array_t<double>& values) {
            py::buffer_info info = values.request();
            return self.HasParameterBlock((double*)info.ptr);
          },
          py::arg("values").noconvert())
      .def(
          "add_residual_block",
          [](ceres::Problem& self,
             ceres::CostFunction* cost,
             std::shared_ptr<ceres::LossFunction> loss,
             std::vector<py::array_t<double>>& paramv) {
            THROW_CHECK_EQ(paramv.size(), cost->parameter_block_sizes().size());
            std::vector<double*> pointer_values(paramv.size());
            for (int i = 0; i < paramv.size(); ++i) {
              py::buffer_info param_buf = paramv[i].request();
              pointer_values[i] = static_cast<double*>(param_buf.ptr);
              ssize_t num_dims = 1;
              std::vector<ssize_t> param_shape = param_buf.shape;
              for (int k = 0; k < param_shape.size(); k++) {
                num_dims *= param_shape[k];
              }
              THROW_CHECK_EQ(num_dims, cost->parameter_block_sizes()[i]);
            }
            ceres::CostFunction* costw = new CostFunctionWrapper(cost);
            return ResidualBlockIDWrapper(
                self.AddResidualBlock(costw, loss.get(), pointer_values));
          },
          py::arg("cost"),
          py::arg("loss"),
          py::arg("paramv").noconvert(),
          py::keep_alive<1, 2>(),  // Cost Function
          py::keep_alive<1, 3>(),  // Loss Function
          py::keep_alive<1, 4>())  // Parameters
      .def(
          "add_parameter_block",
          [](ceres::Problem& self, py::array_t<double>& values, int size) {
            double* pointer = static_cast<double*>(values.request().ptr);
            self.AddParameterBlock(pointer, size);
          },
          py::arg("values").noconvert(),
          py::arg("size"))
      .def(
          "add_parameter_block",
          [](ceres::Problem& self,
             py::array_t<double>& values,
             int size,
             ceres::Manifold* manifold) {
            double* pointer = static_cast<double*>(values.request().ptr);
            self.AddParameterBlock(pointer, size, manifold);
          },
          py::arg("values").noconvert(),
          py::arg("size"),
          py::arg("manifold"),
          py::keep_alive<1, 4>()  // LocalParameterization
          )
      .def(
          "remove_parameter_block",
          [](ceres::Problem& self, py::array_t<double>& values) {
            double* pointer = static_cast<double*>(values.request().ptr);
            THROW_CHECK(self.HasParameterBlock(pointer));
            self.RemoveParameterBlock(pointer);
          },
          py::arg("values").noconvert())
      .def("remove_residual_block",
           [](ceres::Problem& self, ResidualBlockIDWrapper& residual_block_id) {
             self.RemoveResidualBlock(residual_block_id.id);
           })
      .def(
          "evaluate_residuals",
          [](ceres::Problem& self,
             const ceres::Problem::EvaluateOptions& options) {
            std::vector<double> residuals;
            self.Evaluate(options, nullptr, &residuals, nullptr, nullptr);
            return residuals;
          },
          py::arg_v("options",
                    ceres::Problem::EvaluateOptions(),
                    "EvaluateOptions()"))
      .def(
          "evaluate_residuals",
          [](ceres::Problem& self,
             std::vector<ResidualBlockIDWrapper>& residual_block_ids) {
            ceres::Problem::EvaluateOptions eval_options =
                ceres::Problem::EvaluateOptions();
            SetResidualBlocks(eval_options, residual_block_ids);
            std::vector<double> residuals;
            self.Evaluate(eval_options, nullptr, &residuals, nullptr, nullptr);
            return residuals;
          },
          py::arg("residual_block_ids"))
      .def(
          "evaluate_jacobian",
          [](ceres::Problem& self,
             const ceres::Problem::EvaluateOptions& options) {
            ceres::CRSMatrix jacobian;
            self.Evaluate(options, nullptr, nullptr, nullptr, &jacobian);
            return jacobian;
          },
          py::arg_v("options",
                    ceres::Problem::EvaluateOptions(),
                    "EvaluateOptions()"));
}
