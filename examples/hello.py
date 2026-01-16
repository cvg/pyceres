import numpy as np
import pyceres


# Cost functor equivalent to C++ AutoDiffCostFunction<CostFunctor, 1, 1>
class CostFunctor(pyceres.CostFunction):
    def __init__(self):
        super().__init__()
        self.set_num_residuals(1)
        self.set_parameter_block_sizes([1])

    def Evaluate(self, parameters, residuals, jacobians):
        x = parameters[0][0]
        residuals[0] = 10.0 - x

        if(jacobians != None):
            jacobians[0][:] = -1
            
        # Let pyceres do numeric differentiation
        return True


def main():
    # Initial value
    initial_x = 30000000
    x = np.array([initial_x], dtype=np.float64)

    # Build problem
    problem = pyceres.Problem()

    cost_function = CostFunctor()
    problem.add_residual_block(
        cost_function,
        None,     # no loss function
        [x]
    )

    # Solver options
    options = pyceres.SolverOptions()
    options.linear_solver_type = pyceres.LinearSolverType.DENSE_QR
    options.minimizer_progress_to_stdout = True

    summary = pyceres.SolverSummary()
    pyceres.solve(options, problem, summary)

    print(summary.BriefReport())
    print(f"x : {initial_x} -> {x[0]}")


if __name__ == "__main__":
    main()
