#pragma once

#include "_pyceres/helpers.h"
#include "_pyceres/logging.h"

#include <string>

#include <ceres/ceres.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// Trampoline class so that we can create a LossFunction in Python.
class PyLossFunction : public ceres::LossFunction {
 public:
  /* Inherit the constructors */
  using ceres::LossFunction::LossFunction;

  void Evaluate(double sq_norm, double out[3]) const override {
    py::gil_scoped_acquire gil;

    py::array_t<double> out_arr(3, out, no_copy);
    py::function overload = py::get_overload(
        static_cast<const ceres::LossFunction*>(this), "Evaluate");
    if (overload) {
      overload.operator()(sq_norm, out_arr);
    } else {
      LOG_FATAL_THROW(std::runtime_error) << "<Evaluate> not implemented.";
    }
  }

 private:
  mutable py::str no_copy;  // Dummy variable for pybind11 to avoid copy
};

ceres::LossFunction* CreateLossFunction(std::string loss_name,
                                        std::vector<double> scales) {
  ceres::LossFunction* loss_function = nullptr;
  if (loss_name == "trivial") {
    loss_function = new ceres::TrivialLoss();
  } else if (loss_name == "soft_l1") {
    loss_function = new ceres::SoftLOneLoss(scales.at(0));
  } else if (loss_name == "cauchy") {
    loss_function = new ceres::CauchyLoss(scales.at(0));
  } else if (loss_name == "tolerant") {
    loss_function = new ceres::TolerantLoss(scales.at(0), scales.at(1));
  } else if (loss_name == "huber") {
    loss_function = new ceres::HuberLoss(scales.at(0));
  } else if (loss_name == "arctan") {
    loss_function = new ceres::ArctanLoss(scales.at(0));
  } else {
    std::string failure_message = "Unknown loss_name " + loss_name;
    throw py::index_error(failure_message.c_str());
  }
  return loss_function;
}

ceres::LossFunction* CreateScaledLossFunction(std::string loss_name,
                                              std::vector<double> scales,
                                              double magnitude) {
  return new ceres::ScaledLoss(
      CreateLossFunction(loss_name, scales), magnitude, ceres::TAKE_OWNERSHIP);
}

std::shared_ptr<ceres::LossFunction> CreateLossFunctionFromDict(py::dict dict) {
  THROW_CHECK(dict.contains("name"));
  std::string loss_name = dict["name"].cast<std::string>();

  if (loss_name != std::string("trivial")) {
    THROW_CHECK(dict.contains("params"));
  }
  if (dict.contains("magnitude")) {
    return std::shared_ptr<ceres::LossFunction>(
        CreateScaledLossFunction(dict["name"].cast<std::string>(),
                                 dict["params"].cast<std::vector<double>>(),
                                 dict["magnitude"].cast<double>()));
  } else {
    return std::shared_ptr<ceres::LossFunction>(
        CreateLossFunction(dict["name"].cast<std::string>(),
                           dict["params"].cast<std::vector<double>>()));
  }
}

void BindLossFunctions(py::module& m) {
  py::class_<ceres::LossFunction,
             PyLossFunction /*<--- trampoline*/,
             std::shared_ptr<ceres::LossFunction>>(m, "LossFunction")
      .def(py::init<>())
      .def(py::init(&CreateLossFunctionFromDict))
      .def("evaluate", [](ceres::LossFunction& self, float v) {
        Eigen::Vector3d rho;
        self.Evaluate(v, rho.data());
        return rho;
      });
  py::implicitly_convertible<py::dict, ceres::LossFunction>();

  py::class_<ceres::TrivialLoss,
             ceres::LossFunction,
             std::shared_ptr<ceres::TrivialLoss>>(m, "TrivialLoss")
      .def(py::init<>());

  py::class_<ceres::HuberLoss,
             ceres::LossFunction,
             std::shared_ptr<ceres::HuberLoss>>(m, "HuberLoss")
      .def(py::init<double>());

  py::class_<ceres::SoftLOneLoss,
             ceres::LossFunction,
             std::shared_ptr<ceres::SoftLOneLoss>>(m, "SoftLOneLoss")
      .def(py::init<double>());

  py::class_<ceres::CauchyLoss,
             ceres::LossFunction,
             std::shared_ptr<ceres::CauchyLoss>>(m, "CauchyLoss")
      .def(py::init<double>());
}
