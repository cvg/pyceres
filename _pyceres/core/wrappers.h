#include <ceres/ceres.h>

// Wrapper around ceres ResidualBlockID. In Ceres a ResidualBlockId is
// actually just a pointer to internal::ResidualBlock. However, since Ceres
// uses a forward declaration we don't actually have the type definition.
// (Ceres doesn't make it part of its public API). Since pybind11 needs a type
// we use this class instead which simply holds the pointer.
struct ResidualBlockIDWrapper {
 public:
  ResidualBlockIDWrapper(const ceres::ResidualBlockId& id) : id(id) {}
  const ceres::ResidualBlockId id;
};

// Wrapper around cost functions. Solely for lifetime management.
class CostFunctionWrapper : public ceres::CostFunction {
 public:
  explicit CostFunctionWrapper(ceres::CostFunction* real_cost_function)
      : cost_function_(real_cost_function) {
    this->set_num_residuals(cost_function_->num_residuals());
    *(this->mutable_parameter_block_sizes()) =
        cost_function_->parameter_block_sizes();
  }

  bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const override {
    return cost_function_->Evaluate(parameters, residuals, jacobians);
  }

 private:
  ceres::CostFunction* cost_function_;
};

// Wrapper around manifolds. Solely for lifetime management.
class ManifoldWrapper : public ceres::Manifold {
 public:
  explicit ManifoldWrapper(ceres::Manifold* real_manifold)
      : manifold_(real_manifold) {}
  virtual ~ManifoldWrapper() {}

  // Generalization of the addition operation,
  //
  //   x_plus_delta = Plus(x, delta)
  //
  // with the condition that Plus(x, 0) = x.
  bool Plus(const double* x,
            const double* delta,
            double* x_plus_delta) const override {
    return manifold_->Plus(x, delta, x_plus_delta);
  }

  // The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  //
  // jacobian is a row-major AmbientSize() x TangentSize() matrix.
  bool PlusJacobian(const double* x, double* jacobian) const override {
    return manifold_->PlusJacobian(x, jacobian);
  }

  // Generalization of vector subtraction,
  //
  //   y_minus_x = Minus(x, y) = y - x
  bool Minus(const double* y,
             const double* x,
             double* y_minus_x) const override {
    return manifold_->Minus(y, x, y_minus_x);
  }

  // The jacobian of Minus(y, x) w.r.t y at y = x.
  //
  // jacobian is a row-major TangentSize() x AmbientSize() matrix.
  bool MinusJacobian(const double* x, double* jacobian) const override {
    return manifold_->MinusJacobian(x, jacobian);
  }

  // Size of x.
  int AmbientSize() const override { return manifold_->AmbientSize(); }

  // Size of delta.
  int TangentSize() const override { return manifold_->TangentSize(); }

 private:
  ceres::Manifold* manifold_;
};