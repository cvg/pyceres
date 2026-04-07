#pragma once

#include "_pyceres/helpers.h"
#include "_pyceres/logging.h"

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

class PyProductManifold : public ceres::Manifold {
 public:
  explicit PyProductManifold(const std::vector<py::object>& manifolds) {
    for (const py::object& obj : manifolds) {
      ceres::Manifold* m = obj.cast<ceres::Manifold*>();
      THROW_CHECK_NOTNULL(m);
      py_refs_.push_back(obj);
      manifolds_.push_back(m);
      ambient_offsets_.push_back(ambient_size_);
      tangent_offsets_.push_back(tangent_size_);
      ambient_size_ += m->AmbientSize();
      tangent_size_ += m->TangentSize();
    }
  }

  int AmbientSize() const override { return ambient_size_; }
  int TangentSize() const override { return tangent_size_; }

  bool Plus(const double* x,
            const double* delta,
            double* x_plus_delta) const override {
    for (size_t i = 0; i < manifolds_.size(); ++i) {
      if (!manifolds_[i]->Plus(x + ambient_offsets_[i],
                               delta + tangent_offsets_[i],
                               x_plus_delta + ambient_offsets_[i])) {
        return false;
      }
    }
    return true;
  }

  bool Minus(const double* y,
             const double* x,
             double* y_minus_x) const override {
    for (size_t i = 0; i < manifolds_.size(); ++i) {
      if (!manifolds_[i]->Minus(y + ambient_offsets_[i],
                                x + ambient_offsets_[i],
                                y_minus_x + tangent_offsets_[i])) {
        return false;
      }
    }
    return true;
  }

  bool PlusJacobian(const double* x, double* jacobian_ptr) const override {
    using RowMajorMat =
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<RowMajorMat> J(jacobian_ptr, ambient_size_, tangent_size_);
    J.setZero();
    for (size_t i = 0; i < manifolds_.size(); ++i) {
      const int as = manifolds_[i]->AmbientSize();
      const int ts = manifolds_[i]->TangentSize();
      RowMajorMat block(as, ts);
      if (!manifolds_[i]->PlusJacobian(x + ambient_offsets_[i], block.data())) {
        return false;
      }
      J.block(ambient_offsets_[i], tangent_offsets_[i], as, ts) = block;
    }
    return true;
  }

  bool MinusJacobian(const double* x, double* jacobian_ptr) const override {
    using RowMajorMat =
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<RowMajorMat> J(jacobian_ptr, tangent_size_, ambient_size_);
    J.setZero();
    for (size_t i = 0; i < manifolds_.size(); ++i) {
      const int as = manifolds_[i]->AmbientSize();
      const int ts = manifolds_[i]->TangentSize();
      RowMajorMat block(ts, as);
      if (!manifolds_[i]->MinusJacobian(x + ambient_offsets_[i],
                                        block.data())) {
        return false;
      }
      J.block(tangent_offsets_[i], ambient_offsets_[i], ts, as) = block;
    }
    return true;
  }

 private:
  std::vector<py::object> py_refs_;
  std::vector<ceres::Manifold*> manifolds_;
  std::vector<int> ambient_offsets_;
  std::vector<int> tangent_offsets_;
  int ambient_size_ = 0;
  int tangent_size_ = 0;
};

class PyManifold : public ceres::Manifold, py::trampoline_self_life_support {
  /* Inherit the constructors */
  using ceres::Manifold::Manifold;
  bool Plus(const double* x,
            const double* delta,
            double* x_plus_delta) const override {
    LOG_FATAL_THROW(std::runtime_error) << "<Plus> not implemented.";
    return true;
  }

  bool PlusJacobian(const double* x, double* jacobian) const override {
    LOG_FATAL_THROW(std::runtime_error) << "<PlusJacobian> not implemented.";
    return true;
  }

  bool Minus(const double* y,
             const double* x,
             double* y_minus_x) const override {
    LOG_FATAL_THROW(std::runtime_error) << "<Minus> not implemented.";
    return true;
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    LOG_FATAL_THROW(std::runtime_error) << "<MinusJacobian> not implemented.";
    return true;
  }

  // Size of x.
  int AmbientSize() const override {
    PYBIND11_OVERLOAD_PURE_NAME(
        int,             /* Return type */
        ceres::Manifold, /* Parent class */
        "ambient_size",  /* Name of python function */
        AmbientSize      /* Name of function in C++ (must match Python name) */
    );
  }

  // Size of delta.
  int TangentSize() const override {
    PYBIND11_OVERLOAD_PURE_NAME(
        int,             /* Return type */
        ceres::Manifold, /* Parent class */
        "tangent_size",  /* Name of python function */
        TangentSize      /* Name of function in C++ (must match Python name) */
    );
  }
};

void BindManifold(py::module& m) {
  py::classh<ceres::Manifold, PyManifold /* <--- trampoline*/>(m, "Manifold")
      .def(py::init<>())
      .def("ambient_size", &ceres::Manifold::AmbientSize)
      .def("tangent_size", &ceres::Manifold::TangentSize);

  py::classh<ceres::EuclideanManifold<ceres::DYNAMIC>, ceres::Manifold>(
      m, "EuclideanManifold")
      .def(py::init<int>());
  py::classh<ceres::SubsetManifold, ceres::Manifold>(m, "SubsetManifold")
      .def(py::init<int, const std::vector<int>&>());
  py::classh<ceres::QuaternionManifold, ceres::Manifold>(m,
                                                         "QuaternionManifold")
      .def(py::init<>());
  py::classh<ceres::EigenQuaternionManifold, ceres::Manifold>(
      m, "EigenQuaternionManifold")
      .def(py::init<>());
  py::classh<ceres::SphereManifold<ceres::DYNAMIC>, ceres::Manifold>(
      m, "SphereManifold")
      .def(py::init<int>());
  py::classh<PyProductManifold, ceres::Manifold>(m, "ProductManifold")
      .def(py::init([](py::args args) {
             std::vector<py::object> manifolds;
             manifolds.reserve(args.size());
             for (auto h : args) {
               manifolds.push_back(py::reinterpret_borrow<py::object>(h));
             }
             if (manifolds.size() < 2) {
               throw std::invalid_argument(
                   "ProductManifold requires at least 2 manifolds");
             }
             return std::make_shared<PyProductManifold>(manifolds);
           }),
           "Construct a product manifold from 2 or more Manifold instances.");
}
