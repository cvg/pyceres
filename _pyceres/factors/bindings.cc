#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

#include "factors/bundle.cc"
#include "factors/common.cc"
#include "factors/pose_graph.cc"
#include "helpers.h"

using namespace colmap;

void bind_factors(py::module& m) {
  m.def("BundleAdjustmentCost",
        &CreateCostFunction<ReprojErrorCostFunction, const Eigen::Vector2d&>,
        py::arg("camera_model_id"), py::arg("point2D"));
  m.def("BundleAdjustmentCost",
        &CreateCostFunction<ReprojErrorCostFunctionWithNoise, const Eigen::Vector2d&,
                            const double>,
        py::arg("camera_model_id"), py::arg("point2D"), py::arg("stddev"));
  m.def("BundleAdjustmentCost",
        &CreateCostFunction<ReprojErrorConstantPoseCostFunction, const Rigid3d&,
                            const Eigen::Vector2d&>,
        py::arg("camera_model_id"), py::arg("cam_from_world"), py::arg("point2D"));
  m.def("BundleAdjustmentCost",
        &CreateCostFunction<ReprojErrorConstantPoseCostFunctionWithNoise, const Rigid3d&,
                            const Eigen::Vector2d&, const double>,
        py::arg("camera_model_id"), py::arg("cam_from_world"), py::arg("point2D"),
        py::arg("stddev"));

  m.def("BundleAdjustmentRigCost",
        &CreateCostFunction<RigReprojErrorCostFunction, const Eigen::Vector2d&>,
        py::arg("camera_model_id"), py::arg("point2D"));
  m.def("BundleAdjustmentRigCost",
        &CreateCostFunction<RigReprojErrorCostFunctionWithNoise, const Eigen::Vector2d&,
                            const double>,
        py::arg("camera_model_id"), py::arg("point2D"), py::arg("stddev"));
  m.def("BundleAdjustmentRigCost",
        &CreateCostFunction<RigReprojErrorConstantRigCostFunction, const Rigid3d&,
                            const Eigen::Vector2d&>,
        py::arg("camera_model_id"), py::arg("cam_from_rig"), py::arg("point2D"));
  m.def("BundleAdjustmentRigCost",
        &CreateCostFunction<RigReprojErrorConstantRigCostFunctionWithNoise,
                            const Rigid3d&, const Eigen::Vector2d&, const double>,
        py::arg("camera_model_id"), py::arg("cam_from_rig"), py::arg("point2D"),
        py::arg("stddev"));

  m.def("PoseGraphRelativeCost", &PoseGraphRelativeCost::Create, py::arg("qvec_j_i"),
        py::arg("tvec_j_i"), py::arg("covariance"));
  m.def("PoseGraphAbsoluteCost", &PoseGraphAbsoluteCost::Create, py::arg("qvec_i_w"),
        py::arg("tvec_i_w"), py::arg("covariance"));

  m.def("NormalPrior", &CreateNormalPrior, py::arg("mean"), py::arg("covariance"));
}
