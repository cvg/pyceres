#include <colmap/base/camera_models.h>
#include <colmap/base/projection.h>
#include <colmap/util/types.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

template <typename CameraModel, typename T>
inline void WorldToPixel(const T* camera_params, const T* qvec, const T* tvec,
                         const T* xyz, T* xy) {
  // Rotate and translate.
  T projection[3];
  ceres::QuaternionRotatePoint(qvec, xyz, projection);
  projection[0] += tvec[0];
  projection[1] += tvec[1];
  projection[2] += tvec[2];

  // Project to image plane.
  projection[0] /= projection[2];  // u
  projection[1] /= projection[2];  // v

  // Distort and transform to pixel space.
  CameraModel::WorldToImage(camera_params, projection[0], projection[1], &xy[0], &xy[1]);
}

template <typename T>
inline void ComposePose(const T* qvec1, const T* tvec1, const T* qvec2, const T* tvec2,
                        T* qvec_out, T* tvec_out) {
  ceres::QuaternionProduct(qvec1, qvec2, qvec_out);
  // Concatenate translations.
  ceres::UnitQuaternionRotatePoint(qvec1, tvec2, tvec_out);
  tvec_out[0] += tvec1[0];
  tvec_out[1] += tvec1[1];
  tvec_out[2] += tvec1[2];
}

template <typename CameraModel>
class BundleAdjustmentCost {
 public:
  explicit BundleAdjustmentCost(const Eigen::Vector2d& point2D, const double stddev)
      : observed_x_(point2D(0)), observed_y_(point2D(1)), scale_(1.0 / stddev) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D,
                                     const double stddev) {
    return (new ceres::AutoDiffCostFunction<BundleAdjustmentCost<CameraModel>, 2, 4, 3, 3,
                                            CameraModel::kNumParams>(
        new BundleAdjustmentCost(point2D, stddev)));
  }

  template <typename T>
  bool operator()(const T* const qvec, const T* const tvec, const T* const point3D,
                  const T* const camera_params, T* residuals) const {
    WorldToPixel<CameraModel>(camera_params, qvec, tvec, point3D, residuals);
    residuals[0] = T(scale_) * (residuals[0] - T(observed_x_));
    residuals[1] = T(scale_) * (residuals[1] - T(observed_y_));
    return true;
  }

 private:
  const double observed_x_;
  const double observed_y_;
  const double scale_;
};

// TODO: can avoid one memory allocation if not inherited
template <typename CameraModel>
class BundleAdjustmentConstantPoseCost : public BundleAdjustmentCost<CameraModel> {
  using Parent = BundleAdjustmentCost<CameraModel>;

 public:
  explicit BundleAdjustmentConstantPoseCost(
      // const Eigen::Vector2d& point2D, const double* qvec, const double* tvec)
      const Eigen::Vector2d& point2D, const double stddev, const Eigen::Vector4d& qvec,
      const Eigen::Vector3d& tvec)
      : Parent(point2D, stddev), qvec_(qvec), tvec_(tvec) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D, const double stddev,
                                     const Eigen::Vector4d qvec,
                                     const Eigen::Vector3d tvec) {
    return (new ceres::AutoDiffCostFunction<BundleAdjustmentConstantPoseCost<CameraModel>,
                                            2, 3, CameraModel::kNumParams>(
        new BundleAdjustmentConstantPoseCost(point2D, stddev, qvec, tvec)));
  }

  template <typename T>
  bool operator()(const T* const point3D, const T* const camera_params,
                  T* residuals) const {
    const Eigen::Matrix<T, 4, 1> qvec = qvec_.cast<T>();
    const Eigen::Matrix<T, 3, 1> tvec = tvec_.cast<T>();
    return Parent::operator()(qvec.data(), tvec.data(), point3D, camera_params,
                              residuals);
  }

 private:
  const Eigen::Vector4d qvec_;
  const Eigen::Vector3d tvec_;
};

template <typename CameraModel>
class BundleAdjustmentConstantRigCost : public BundleAdjustmentCost<CameraModel> {
  using Parent = BundleAdjustmentCost<CameraModel>;

 public:
  explicit BundleAdjustmentConstantRigCost(const Eigen::Vector2d& point2D,
                                           const double stddev,
                                           const Eigen::Vector4d& rel_qvec,
                                           const Eigen::Vector3d& rel_tvec)
      : Parent(point2D, stddev), rel_qvec_(rel_qvec), rel_tvec_(rel_tvec) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D, const double stddev,
                                     const Eigen::Vector4d rel_qvec,
                                     const Eigen::Vector3d rel_tvec) {
    return (new ceres::AutoDiffCostFunction<BundleAdjustmentConstantRigCost<CameraModel>,
                                            2, 4, 3, 3, CameraModel::kNumParams>(
        new BundleAdjustmentConstantRigCost(point2D, stddev, rel_qvec, rel_tvec)));
  }

  template <typename T>
  bool operator()(const T* const rig_qvec, const T* const rig_tvec,
                  const T* const point3D, const T* const camera_params,
                  T* residuals) const {
    const Eigen::Matrix<T, 4, 1> rel_qvec = rel_qvec_.cast<T>();
    const Eigen::Matrix<T, 3, 1> rel_tvec = rel_tvec_.cast<T>();
    T qvec[4], tvec[3];
    ComposePose(rel_qvec.data(), rel_tvec.data(), rig_qvec, rig_tvec, qvec, tvec);
    return Parent::operator()(qvec, tvec, point3D, camera_params, residuals);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const Eigen::Vector4d rel_qvec_;
  const Eigen::Vector3d rel_tvec_;
};

template <typename CameraModel>
class BundleAdjustmentRigCost : public BundleAdjustmentCost<CameraModel> {
  using Parent = BundleAdjustmentCost<CameraModel>;

 public:
  explicit BundleAdjustmentRigCost(const Eigen::Vector2d& point2D, const double stddev)
      : Parent(point2D, stddev) {}

  static ceres::CostFunction* Create(const Eigen::Vector2d& point2D,
                                     const double stddev) {
    return (new ceres::AutoDiffCostFunction<BundleAdjustmentRigCost<CameraModel>, 2, 4, 3,
                                            4, 3, 3, CameraModel::kNumParams>(
        new BundleAdjustmentRigCost(point2D, stddev)));
  }

  template <typename T>
  bool operator()(const T* const rig_qvec, const T* const rig_tvec,
                  const T* const rel_qvec, const T* const rel_tvec,
                  const T* const point3D, const T* const camera_params,
                  T* residuals) const {
    T qvec[4], tvec[3];
    ComposePose(rel_qvec, rel_tvec, rig_qvec, rig_tvec, qvec, tvec);
    return Parent::operator()(qvec, tvec, point3D, camera_params, residuals);
  }
};

ceres::CostFunction* CreateBundleAdjustmentCost(int camera_model_id,
                                                const Eigen::Vector2d& point2D,
                                                const double stddev) {
  switch (camera_model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
  case colmap::CameraModel::kModelId:                                          \
    return BundleAdjustmentCost<colmap::CameraModel>::Create(point2D, stddev); \
    break;
    CAMERA_MODEL_SWITCH_CASES
#undef CAMERA_MODEL_CASE
  }
}

ceres::CostFunction* CreateBundleAdjustmentConstantPoseCost(
    int camera_model_id, const Eigen::Vector2d& point2D, const Eigen::Vector4d& qvec,
    const Eigen::Vector3d& tvec, const double stddev) {
  switch (camera_model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                    \
  case colmap::CameraModel::kModelId:                                     \
    return BundleAdjustmentConstantPoseCost<colmap::CameraModel>::Create( \
        point2D, stddev, qvec, tvec);                                     \
    break;
    CAMERA_MODEL_SWITCH_CASES
#undef CAMERA_MODEL_CASE
  }
}

ceres::CostFunction* CreateBundleAdjustmentConstantRigCost(
    int camera_model_id, const Eigen::Vector2d& point2D, const Eigen::Vector4d& rel_qvec,
    const Eigen::Vector3d& rel_tvec, const double stddev) {
  switch (camera_model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                   \
  case colmap::CameraModel::kModelId:                                    \
    return BundleAdjustmentConstantRigCost<colmap::CameraModel>::Create( \
        point2D, stddev, rel_qvec, rel_tvec);                            \
    break;
    CAMERA_MODEL_SWITCH_CASES
#undef CAMERA_MODEL_CASE
  }
}

ceres::CostFunction* CreateBundleAdjustmentRigCost(int camera_model_id,
                                                   const Eigen::Vector2d& point2D,
                                                   const double stddev) {
  switch (camera_model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                            \
  case colmap::CameraModel::kModelId:                                             \
    return BundleAdjustmentRigCost<colmap::CameraModel>::Create(point2D, stddev); \
    break;
    CAMERA_MODEL_SWITCH_CASES
#undef CAMERA_MODEL_CASE
  }
}
