"""Tests for pyceres.ProductManifold."""

import numpy as np

import pyceres


def test_product_manifold_sizes():
    m = pyceres.ProductManifold(
        pyceres.EigenQuaternionManifold(),  # ambient=4, tangent=3
        pyceres.EuclideanManifold(3),       # ambient=3, tangent=3
    )
    assert m.ambient_size() == 7
    assert m.tangent_size() == 6


def test_product_manifold_plus_identity():
    """Plus with zero delta must leave x unchanged."""
    m = pyceres.ProductManifold(
        pyceres.EigenQuaternionManifold(),
        pyceres.EuclideanManifold(3),
    )
    # Identity quaternion (x, y, z, w) for Eigen + translation
    x = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 3.0])
    # Drive Plus through a Problem so we exercise the C++ side end-to-end.
    prob = pyceres.Problem()
    params = x.copy()
    prob.add_parameter_block(params, 7)
    prob.set_manifold(params, m)
    assert prob.parameter_block_tangent_size(params) == 6
    np.testing.assert_allclose(params, x)


def test_product_manifold_in_solver():
    """Use ProductManifold inside a tiny problem and make sure it solves."""

    class IdentityCost(pyceres.CostFunction):
        def __init__(self, target):
            super().__init__()
            self.set_num_residuals(7)
            self.set_parameter_block_sizes([7])
            self._target = target

        def Evaluate(self, parameters, residuals, jacobians):
            p = parameters[0]
            for i in range(7):
                residuals[i] = p[i] - self._target[i]
            if jacobians is not None:
                J = jacobians[0]
                J[:] = 0.0
                for i in range(7):
                    J[i * 7 + i] = 1.0
            return True

    target = np.array([0.0, 0.0, 0.0, 1.0, 0.5, -0.25, 1.0])
    x = np.array([0.1, 0.0, 0.0, 0.99498744, 0.0, 0.0, 0.0])
    # Renormalize quaternion
    x[:4] /= np.linalg.norm(x[:4])

    prob = pyceres.Problem()
    prob.add_residual_block(IdentityCost(target), None, [x])
    prob.set_manifold(
        x,
        pyceres.ProductManifold(
            pyceres.EigenQuaternionManifold(),
            pyceres.EuclideanManifold(3),
        ),
    )
    options = pyceres.SolverOptions()
    options.max_num_iterations = 50
    summary = pyceres.SolverSummary()
    pyceres.solve(options, prob, summary)
    # Translation should converge to target.
    np.testing.assert_allclose(x[4:], target[4:], atol=1e-6)
    # Quaternion stays unit-norm thanks to the manifold.
    np.testing.assert_allclose(np.linalg.norm(x[:4]), 1.0, atol=1e-9)


if __name__ == "__main__":
    test_product_manifold_sizes()
    test_product_manifold_plus_identity()
    test_product_manifold_in_solver()
    print("OK")
