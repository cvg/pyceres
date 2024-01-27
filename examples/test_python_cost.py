import numpy as np

import pyceres


# ref: examples/helloworld_analytic_diff.cc
class HelloworldCostFunction(pyceres.CostFunction):
    def __init__(self):
        pyceres.CostFunction.__init__(self)
        self.set_num_residuals(1)
        self.set_parameter_block_sizes([1])

    def Evaluate(self, parameters, residuals, jacobians):
        x = parameters[0][0]
        residuals[0] = 10.0 - x
        if jacobians is not None:
            jacobians[0][0] = -1.0
        return True


def test_python_cost():
    x = np.array(5.0)
    x_ori = x.copy()
    prob = pyceres.Problem()
    cost = HelloworldCostFunction()
    prob.add_residual_block(cost, None, [x])
    options = pyceres.SolverOptions()
    options.minimizer_progress_to_stdout = True
    summary = pyceres.SolverSummary()
    pyceres.solve(options, prob, summary)
    print(summary.BriefReport())
    print(f"{x_ori} -> {x}")


if __name__ == "__main__":
    test_python_cost()
