#include <ceres/ceres.h>
#include <ceres/normal_prior.h>

#include "helpers.h"

ceres::CostFunction* CreateNormalPrior(const Eigen::VectorXd& mean,
                                       const Eigen::Matrix<double, -1, -1>& covariance) {
    THROW_CHECK_EQ(covariance.cols(), mean.size());
    THROW_CHECK_EQ(covariance.cols(), covariance.rows());
    return new ceres::NormalPrior(covariance.inverse().llt().matrixL(), mean);
}
