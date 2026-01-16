import math
import numpy as np
import pyceres


class F1(pyceres.CostFunction):
    def __init__(self):
        super().__init__()
        self.set_num_residuals(1)
        self.set_parameter_block_sizes([1, 1])
        self.set_use_numeric_diff(True)

    def Evaluate(self, parameters, residuals, jacobians):
        x1 = parameters[0][0]
        x2 = parameters[1][0]
        residuals[0] = x1 + 10.0 * x2
        return True


class F2(pyceres.CostFunction):
    def __init__(self):
        super().__init__()
        self.set_num_residuals(1)
        self.set_parameter_block_sizes([1, 1])
        self.set_use_numeric_diff(True)

    def Evaluate(self, parameters, residuals, jacobians):
        x3 = parameters[0][0]
        x4 = parameters[1][0]
        residuals[0] = math.sqrt(5.0) * (x3 - x4)
        return True


class F3(pyceres.CostFunction):
    def __init__(self):
        super().__init__()
        self.set_num_residuals(1)
        self.set_parameter_block_sizes([1, 1])
        self.set_use_numeric_diff(True)

    def Evaluate(self, parameters, residuals, jacobians):
        x2 = parameters[0][0]
        x3 = parameters[1][0]
        r = x2 - 2.0 * x3
        residuals[0] = r * r
        return True


class F4(pyceres.CostFunction):
    def __init__(self):
        super().__init__()
        self.set_num_residuals(1)
        self.set_parameter_block_sizes([1, 1])
        self.set_use_numeric_diff(True)

    def Evaluate(self, parameters, residuals, jacobians):
        x1 = parameters[0][0]
        x4 = parameters[1][0]
        r = x1 - x4
        residuals[0] = math.sqrt(10.0) * r * r
        return True


def main():
    x1 = np.array([3.0], dtype=np.float64)
    x2 = np.array([-1.0], dtype=np.float64)
    x3 = np.array([0.0], dtype=np.float64)
    x4 = np.array([1.0], dtype=np.float64)

    x0 = (x1.copy(), x2.copy(), x3.copy(), x4.copy())

    problem = pyceres.Problem()

    problem.add_residual_block(F1(), None, [x1, x2])
    problem.add_residual_block(F2(), None, [x3, x4])
    problem.add_residual_block(F3(), None, [x2, x3])
    problem.add_residual_block(F4(), None, [x1, x4])

    options = pyceres.SolverOptions()
    options.linear_solver_type = pyceres.LinearSolverType.DENSE_QR
    options.max_num_iterations = 100
    options.minimizer_progress_to_stdout = True

    summary = pyceres.SolverSummary()
    pyceres.solve(options, problem, summary)

    print(summary.BriefReport())
    print("Initial:", [v[0] for v in x0])
    print("Final:  ", [x1[0], x2[0], x3[0], x4[0]])


if __name__ == "__main__":
    main()
