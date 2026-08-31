import gc

import numpy as np
import pytest

import pyceres


def test_loss_function_wrapper_rejects_null_delegate():
    with pytest.raises(ValueError):
        pyceres.LossFunctionWrapper(None)


def test_loss_function_wrapper_switches_and_retains_losses():
    loss = pyceres.TrivialLoss()
    wrapper = pyceres.LossFunctionWrapper(loss)
    del loss
    gc.collect()
    assert wrapper.evaluate(4.0) == pytest.approx([4.0, 1.0, 0.0])

    loss = pyceres.CauchyLoss(1.0)
    wrapper.reset(loss)
    del loss
    gc.collect()
    assert wrapper.evaluate(4.0) == pytest.approx([np.log(5.0), 0.2, -0.04])


def test_problem_retains_loss_function_wrapper():
    wrapper = pyceres.LossFunctionWrapper(pyceres.TrivialLoss())
    problem = pyceres.Problem()
    parameter = np.array([2.0])
    cost = pyceres.factors.NormalPrior(np.zeros(1), np.eye(1))
    problem.add_residual_block(cost, wrapper, [parameter])
    del wrapper
    gc.collect()
    assert problem.evaluate_residuals() == pytest.approx([2.0])
