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
  m.def("ReprojErrorCost",
        &CreateCostFunction<ReprojErrorCostFunction, const Eigen::Vector2d&>,
        py::arg("camera_model_id"), py::arg("point2D"));
  m.def("ReprojErrorCost",
        &CreateCostFunction<ReprojErrorCostFunctionWithNoise, const Eigen::Vector2d&,
                            const double>,
        py::arg("camera_model_id"), py::arg("point2D"), py::arg("stddev"));
  m.def("ReprojErrorCost",
        &CreateCostFunction<ReprojErrorConstantPoseCostFunction, const Rigid3d&,
                            const Eigen::Vector2d&>,
        py::arg("camera_model_id"), py::arg("cam_from_world"), py::arg("point2D"));
  m.def("ReprojErrorCost",
        &CreateCostFunction<ReprojErrorConstantPoseCostFunctionWithNoise, const Rigid3d&,
                            const Eigen::Vector2d&, const double>,
        py::arg("camera_model_id"), py::arg("cam_from_world"), py::arg("point2D"),
        py::arg("stddev"));

  m.def("RigReprojErrorCost",
        &CreateCostFunction<RigReprojErrorCostFunction, const Eigen::Vector2d&>,
        py::arg("camera_model_id"), py::arg("point2D"));
  m.def("RigReprojErrorCost",
        &CreateCostFunction<RigReprojErrorCostFunctionWithNoise, const Eigen::Vector2d&,
                            const double>,
        py::arg("camera_model_id"), py::arg("point2D"), py::arg("stddev"));
  m.def("RigReprojErrorCost",
        &CreateCostFunction<RigReprojErrorConstantRigCostFunction, const Rigid3d&,
                            const Eigen::Vector2d&>,
        py::arg("camera_model_id"), py::arg("cam_from_rig"), py::arg("point2D"));
  m.def("RigReprojErrorCost",
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
