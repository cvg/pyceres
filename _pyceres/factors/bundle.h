#pragma once

#include "_pyceres/log_exceptions.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <colmap/estimators/cost_functions.h>
#include <colmap/geometry/rigid3.h>
#include <colmap/sensor/models.h>

using namespace colmap;

class LinearCostFunction : public ceres::CostFunction {
 public:
  LinearCostFunction(const double s) : s_(s) {
    set_num_residuals(1);
    mutable_parameter_block_sizes()->push_back(1);
  }

  bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const final {
    *residuals = **parameters * s_;
    if (jacobians && *jacobians) {
      **jacobians = s_;
    }
    return true;
  }

 private:
  const double s_;
};

template <class CostFunction, typename... Args>
class CostFunctionIsotropicNoise {
 public:
  static ceres::CostFunction* Create(Args&&... args, const double stddev) {
    THROW_CHECK_GE(stddev, 0.0);
    ceres::CostFunction* cost_function =
        CostFunction::Create(std::forward<Args>(args)...);
    std::vector<ceres::CostFunction*> conditioners(
        cost_function->num_residuals(), new LinearCostFunction(1.0 / stddev));
    return new ceres::ConditionedCostFunction(
        cost_function, conditioners, ceres::TAKE_OWNERSHIP);
  }
};

template <typename CameraModel>
class RigReprojErrorConstantRigCostFunction
    : public ReprojErrorCostFunction<CameraModel> {
  using Parent = ReprojErrorCostFunction<CameraModel>;

 public:
  explicit RigReprojErrorConstantRigCostFunction(const Rigid3d& cam_from_rig,
                                                 const Eigen::Vector2d& point2D)
      : Parent(point2D), cam_from_rig_(cam_from_rig) {}

  static ceres::CostFunction* Create(const Rigid3d& cam_from_rig,
                                     const Eigen::Vector2d& point2D) {
    return (new ceres::AutoDiffCostFunction<
            RigReprojErrorConstantRigCostFunction<CameraModel>,
            2,
            4,
            3,
            3,
            CameraModel::num_params>(
        new RigReprojErrorConstantRigCostFunction(cam_from_rig, point2D)));
  }

  template <typename T>
  bool operator()(const T* const rig_from_world_rotation,
                  const T* const rig_from_world_translation,
                  const T* const point3D,
                  const T* const camera_params,
                  T* residuals) const {
    const Eigen::Quaternion<T> cam_from_world_rotation =
        cam_from_rig_.rotation.cast<T>() *
        EigenQuaternionMap<T>(rig_from_world_rotation);
    const Eigen::Matrix<T, 3, 1> cam_from_world_translation =
        cam_from_rig_.rotation.cast<T>() *
            EigenVector3Map<T>(rig_from_world_translation) +
        cam_from_rig_.translation.cast<T>();
    return Parent::operator()(cam_from_world_rotation.coeffs().data(),
                              cam_from_world_translation.data(),
                              point3D,
                              camera_params,
                              residuals);
  }

 private:
  const Rigid3d& cam_from_rig_;
};

template <typename CameraModel>
using ReprojErrorCostFunctionWithNoise =
    CostFunctionIsotropicNoise<ReprojErrorCostFunction<CameraModel>,
                               const Eigen::Vector2d&>;

template <typename CameraModel>
using ReprojErrorConstantPoseCostFunctionWithNoise =
    CostFunctionIsotropicNoise<ReprojErrorConstantPoseCostFunction<CameraModel>,
                               const Rigid3d&,
                               const Eigen::Vector2d&>;

template <typename CameraModel>
using RigReprojErrorCostFunctionWithNoise =
    CostFunctionIsotropicNoise<RigReprojErrorCostFunction<CameraModel>,
                               const Eigen::Vector2d&>;

template <typename CameraModel>
using RigReprojErrorConstantRigCostFunctionWithNoise =
    CostFunctionIsotropicNoise<
        RigReprojErrorConstantRigCostFunction<CameraModel>,
        const Rigid3d&,
        const Eigen::Vector2d&>;

template <template <typename> class CostFunction, typename... Args>
ceres::CostFunction* CreateCostFunction(const CameraModelId camera_model_id,
                                        Args&&... args) {
  switch (camera_model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                     \
  case CameraModel::model_id:                                              \
    return CostFunction<CameraModel>::Create(std::forward<Args>(args)...); \
    break;
    CAMERA_MODEL_SWITCH_CASES
#undef CAMERA_MODEL_CASE
  }
}
