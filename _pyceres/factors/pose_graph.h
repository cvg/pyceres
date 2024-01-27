#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <colmap/geometry/rigid3.h>

using namespace colmap;

using Matrix6d = Eigen::Matrix<double, 6, 6>;

template <typename T>
inline void EigenQuaternionToAngleAxis(const T* eigen_quaternion,
                                       T* angle_axis) {
  const T quaternion[4] = {eigen_quaternion[3],
                           eigen_quaternion[0],
                           eigen_quaternion[1],
                           eigen_quaternion[2]};
  ceres::QuaternionToAngleAxis(quaternion, angle_axis);
}

/* Adapted from:
 * https://github.com/ceres-solver/ceres-solver/blob/master/examples/slam/pose_graph_3d/pose_graph_3d_error_term.h
 * https://github.com/ceres-solver/ceres-solver/blob/master/internal/ceres/autodiff_benchmarks/relative_pose_error.h
 * Use right-hand multiplication like in GTSAM.
 * The covariance is expressed in the frame j.
 */
struct PoseGraphRelativeCost {
 public:
  explicit PoseGraphRelativeCost(const Rigid3d& j_from_i,
                                 const Matrix6d covariance)
      : j_from_i_(j_from_i),
        sqrt_information_(covariance.inverse().llt().matrixL()) {}

  static ceres::CostFunction* Create(const Rigid3d& j_from_i,
                                     const Matrix6d covariance) {
    return (
        new ceres::AutoDiffCostFunction<PoseGraphRelativeCost, 6, 4, 3, 4, 3>(
            new PoseGraphRelativeCost(j_from_i, covariance)));
  }

  template <typename T>
  bool operator()(const T* const i_from_world_q,
                  const T* const i_from_world_t,
                  const T* const j_from_world_q,
                  const T* const j_from_world_t,
                  T* residuals_ptr) const {
    const Eigen::Quaternion<T> i_from_j_q =
        EigenQuaternionMap<T>(i_from_world_q) *
        EigenQuaternionMap<T>(j_from_world_q).inverse();
    Eigen::Matrix<T, 3, 1> i_from_j_t =
        EigenVector3Map<T>(i_from_world_t) -
        i_from_j_q * EigenVector3Map<T>(j_from_world_t);

    const Eigen::Quaternion<T> mes_from_est_q =
        j_from_i_.rotation.cast<T>() * i_from_j_q;
    EigenQuaternionToAngleAxis(mes_from_est_q.coeffs().data(), residuals_ptr);

    Eigen::Map<Eigen::Matrix<T, 3, 1>> mes_from_est_t(residuals_ptr + 3);
    mes_from_est_t = j_from_i_.translation.cast<T>() +
                     j_from_i_.rotation.cast<T>() * i_from_j_t;

    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());
    return true;
  }

 private:
  const Rigid3d& j_from_i_;
  const Matrix6d sqrt_information_;
};

struct PoseGraphAbsoluteCost {
 public:
  explicit PoseGraphAbsoluteCost(const Rigid3d& cam_from_world,
                                 const Matrix6d& covariance)
      : cam_from_world_(cam_from_world),
        sqrt_information_(covariance.inverse().llt().matrixL()) {}

  static ceres::CostFunction* Create(const Rigid3d& cam_from_world,
                                     const Matrix6d covariance) {
    return (new ceres::AutoDiffCostFunction<PoseGraphAbsoluteCost, 6, 4, 3>(
        new PoseGraphAbsoluteCost(cam_from_world, covariance)));
  }

  template <typename T>
  bool operator()(const T* const cam_from_world_q,
                  const T* const cam_from_world_t,
                  T* residuals_ptr) const {
    const Eigen::Quaternion<T> mes_from_est_q =
        cam_from_world_.rotation.cast<T>() *
        EigenQuaternionMap<T>(cam_from_world_q).inverse();
    EigenQuaternionToAngleAxis(mes_from_est_q.coeffs().data(), residuals_ptr);

    Eigen::Map<Eigen::Matrix<T, 3, 1>> mes_from_est_t(residuals_ptr + 3);
    mes_from_est_t = cam_from_world_.translation.cast<T>() -
                     mes_from_est_q * EigenVector3Map<T>(cam_from_world_t);

    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());
    return true;
  }

 private:
  const Rigid3d& cam_from_world_;
  const Matrix6d sqrt_information_;
};
