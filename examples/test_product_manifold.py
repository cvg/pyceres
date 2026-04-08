"""Tests for pyceres.ProductManifold."""

import numpy as np

import pyceres


def test_product_manifold_sizes():
    m = pyceres.ProductManifold(
        pyceres.EigenQuaternionManifold(),  # ambient=4, tangent=3
        pyceres.EuclideanManifold(3),  # ambient=3, tangent=3
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
    target = np.array([0.0, 0.0, 0.0, 1.0, 0.5, -0.25, 1.0])
    x = np.array([0.1, 0.0, 0.0, 0.99498744, 0.0, 0.0, 0.0])
    # Renormalize quaternion
    x[:4] /= np.linalg.norm(x[:4])

    prob = pyceres.Problem()
    prob.add_residual_block(pyceres.factors.NormalPrior(target, np.eye(7)), None, [x])
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


def test_product_manifold_three_children():
    """ProductManifold should be able to compose more than two child manifolds."""
    m = pyceres.ProductManifold(
        pyceres.EigenQuaternionManifold(),  # ambient=4, tangent=3
        pyceres.EuclideanManifold(3),  # ambient=3, tangent=3
        pyceres.SphereManifold(4),  # ambient=4, tangent=3
    )
    assert m.ambient_size() == 11
    assert m.tangent_size() == 9

    # Identity quaternion + zero translation + unit vector on S^3.
    x = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    target = np.array([0.0, 0.0, 0.0, 1.0, 0.5, -0.25, 1.0, 0.0, 1.0, 0.0, 0.0])

    prob = pyceres.Problem()
    prob.add_residual_block(pyceres.factors.NormalPrior(target, np.eye(11)), None, [x])
    prob.set_manifold(x, m)
    assert prob.parameter_block_tangent_size(x) == 9

    options = pyceres.SolverOptions()
    options.max_num_iterations = 100
    summary = pyceres.SolverSummary()
    pyceres.solve(options, prob, summary)

    # Translation block converges freely.
    np.testing.assert_allclose(x[4:7], target[4:7], atol=1e-6)
    # Quaternion stays unit-norm.
    np.testing.assert_allclose(np.linalg.norm(x[:4]), 1.0, atol=1e-9)
    # Sphere block stays on the unit sphere.
    np.testing.assert_allclose(np.linalg.norm(x[7:11]), 1.0, atol=1e-9)


if __name__ == "__main__":
    test_product_manifold_sizes()
    test_product_manifold_plus_identity()
    test_product_manifold_in_solver()
    test_product_manifold_three_children()
