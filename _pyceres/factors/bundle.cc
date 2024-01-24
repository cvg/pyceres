#include <colmap/estimators/cost_functions.h>

#include <colmap/scene/projection.h>
#include <colmap/sensor/models.h>
#include <colmap/util/types.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace colmap;

template <typename CameraModel>
class ReprojErrorCostFunctionWithNoise : public ReprojErrorCostFunction<CameraModel> {
  using Parent = ReprojErrorCostFunction<CameraModel>;

 public:
  explicit ReprojErrorCostFunctionWithNoise(const Eigen::Vector2d& point2D,
                                            const double stddev)
      : Parent(point2D), scale_(1.0 / stddev) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D,
                                     const double stddev) {
    return (new ceres::AutoDiffCostFunction<ReprojErrorCostFunctionWithNoise<CameraModel>,
                                            2, 4, 3, 3, CameraModel::num_params>(
        new ReprojErrorCostFunctionWithNoise(point2D, stddev)));
  }

  template <typename T>
  bool operator()(const T* const cam_from_world_rotation,
                  const T* const cam_from_world_translation, const T* const point3D,
                  const T* const camera_params, T* residuals) const {
    const bool ret =
        Parent::operator()(cam_from_world_rotation, cam_from_world_translation, point3D,
                           camera_params, residuals);
    residuals[0] *= T(scale_);
    residuals[1] *= T(scale_);
    return ret;
  }

 private:
  const double scale_;
};

template <typename CameraModel>
class ReprojErrorConstantPoseCostFunctionWithNoise
    : public ReprojErrorConstantPoseCostFunction<CameraModel> {
  using Parent = ReprojErrorConstantPoseCostFunction<CameraModel>;

 public:
  explicit ReprojErrorConstantPoseCostFunctionWithNoise(
      // const Eigen::Vector2d& point2D, const double* qvec, const double* tvec)
      const Rigid3d& cam_from_world, const Eigen::Vector2d& point2D, const double stddev)
      : Parent(cam_from_world, point2D), scale_(1.0 / stddev) {}

  static ceres::CostFunction* Create(const Rigid3d& cam_from_world,
                                     const Eigen::Vector2d& point2D,
                                     const double stddev) {
    return (new ceres::AutoDiffCostFunction<
            ReprojErrorConstantPoseCostFunctionWithNoise<CameraModel>, 2, 3,
            CameraModel::num_params>(new ReprojErrorConstantPoseCostFunctionWithNoise(
        cam_from_world, point2D, stddev)));
  }

  template <typename T>
  bool operator()(const T* const point3D, const T* const camera_params,
                  T* residuals) const {
    const bool ret = Parent::operator()(point3D, camera_params, residuals);
    residuals[0] *= T(scale_);
    residuals[1] *= T(scale_);
    return ret;
  }

 private:
  const double scale_;
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
            RigReprojErrorConstantRigCostFunction<CameraModel>, 2, 4, 3, 3,
            CameraModel::num_params>(
        new RigReprojErrorConstantRigCostFunction(cam_from_rig, point2D)));
  }

  template <typename T>
  bool operator()(const T* const rig_from_world_rotation,
                  const T* const rig_from_world_translation, const T* const point3D,
                  const T* const camera_params, T* residuals) const {
    const Eigen::Quaternion<T> cam_from_world_rotation =
        cam_from_rig_.rotation.cast<T>() * EigenQuaternionMap<T>(rig_from_world_rotation);
    const Eigen::Matrix<T, 3, 1> cam_from_world_translation =
        cam_from_rig_.rotation.cast<T>() *
            EigenVector3Map<T>(rig_from_world_translation) +
        cam_from_rig_.translation.cast<T>();
    return Parent::operator()(cam_from_world_rotation.coeffs().data(),
                              cam_from_world_translation.data(), point3D, camera_params,
                              residuals);
  }

 private:
  const Rigid3d& cam_from_rig_;
};

template <typename CameraModel>
class RigReprojErrorConstantRigCostFunctionWithNoise
    : public RigReprojErrorConstantRigCostFunction<CameraModel> {
  using Parent = RigReprojErrorConstantRigCostFunction<CameraModel>;

 public:
  explicit RigReprojErrorConstantRigCostFunctionWithNoise(const Rigid3d& cam_from_rig,
                                                          const Eigen::Vector2d& point2D,
                                                          const double stddev)
      : Parent(cam_from_rig, point2D), scale_(1.0 / stddev) {}

  static ceres::CostFunction* Create(const Rigid3d& cam_from_rig,
                                     const Eigen::Vector2d& point2D,
                                     const double stddev) {
    return (new ceres::AutoDiffCostFunction<
            RigReprojErrorConstantRigCostFunctionWithNoise<CameraModel>, 2, 4, 3, 3,
            CameraModel::num_params>(new RigReprojErrorConstantRigCostFunctionWithNoise(
        cam_from_rig, point2D, stddev)));
  }

  template <typename T>
  bool operator()(const T* const rig_from_world_rotation,
                  const T* const rig_from_world_translation, const T* const point3D,
                  const T* const camera_params, T* residuals) const {
    const bool ret =
        Parent::operator()(rig_from_world_rotation, rig_from_world_translation, point3D,
                           camera_params, residuals);
    residuals[0] *= T(scale_);
    residuals[1] *= T(scale_);
    return ret;
  }

 private:
  const double scale_;
};

template <typename CameraModel>
class RigReprojErrorCostFunctionWithNoise
    : public RigReprojErrorCostFunction<CameraModel> {
  using Parent = RigReprojErrorCostFunction<CameraModel>;

 public:
  explicit RigReprojErrorCostFunctionWithNoise(const Eigen::Vector2d& point2D,
                                               const double stddev)
      : Parent(point2D), scale_(1.0 / stddev) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D,
                                     const double stddev) {
    return (
        new ceres::AutoDiffCostFunction<RigReprojErrorCostFunctionWithNoise<CameraModel>,
                                        2, 4, 3, 4, 3, 3, CameraModel::num_params>(
            new RigReprojErrorCostFunctionWithNoise(point2D, stddev)));
  }

  template <typename T>
  bool operator()(const T* const cam_from_rig_rotation,
                  const T* const cam_from_rig_translation,
                  const T* const rig_from_world_rotation,
                  const T* const rig_from_world_translation, const T* const point3D,
                  const T* const camera_params, T* residuals) const {
    const bool ret = Parent::operator()(
        cam_from_rig_rotation, cam_from_rig_translation, rig_from_world_rotation,
        rig_from_world_translation, point3D, camera_params, residuals);
    residuals[0] *= T(scale_);
    residuals[1] *= T(scale_);
    return ret;
  }

 private:
  const double scale_;
};

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
