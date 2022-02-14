#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

#include "factors/bundle.cc"
#include "factors/common.cc"
#include "factors/pose_graph.cc"
#include "helpers.h"

void bind_factors(py::module& m) {
  // clang-format off
  m.def("BundleAdjustmentCost", &CreateBundleAdjustmentCost,
        py::arg("camera_model_id"),
        py::arg("point2D"),
        py::arg("stddev") = 1.0);
  m.def("BundleAdjustmentCost", &CreateBundleAdjustmentConstantPoseCost,
        py::arg("camera_model_id"),
        py::arg("point2D"),
        py::arg("qvec"),
        py::arg("tvec"),
        py::arg("stddev") = 1.0);
  m.def("BundleAdjustmentRigCost", &CreateBundleAdjustmentRigCost,
        py::arg("camera_model_id"),
        py::arg("point2D"),
        py::arg("stddev") = 1.0);
  m.def("BundleAdjustmentRigCost", &CreateBundleAdjustmentConstantRigCost,
        py::arg("camera_model_id"),
        py::arg("point2D"),
        py::arg("rel_qvec"),
        py::arg("rel_tvec"),
        py::arg("stddev") = 1.0);

  m.def("PoseGraphRelativeCost", &PoseGraphRelativeCost::Create,
        py::arg("qvec_j_i"),
        py::arg("tvec_j_i"),
        py::arg("covariance"));
  m.def("PoseGraphAbsoluteCost", &PoseGraphAbsoluteCost::Create,
        py::arg("qvec_i_w"),
        py::arg("tvec_i_w"),
        py::arg("covariance"));

  m.def("NormalPrior", &CreateNormalPrior,
        py::arg("mean"),
        py::arg("covariance"));
  // clang-format on
}
