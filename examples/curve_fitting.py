import numpy as np
import pyceres
import matplotlib.pyplot as plt
import math


class ExpResidual(pyceres.CostFunction):
    def __init__(self, x, y_obs):
        super().__init__()
        self.x = x
        self.y_obs = y_obs
        self.set_num_residuals(1)
        self.set_parameter_block_sizes([1, 1])



    def Evaluate(self, parameters, residuals, jacobians):
        m = parameters[0][0]
        c = parameters[1][0]

        # Model prediction
        y_pred = math.exp(m * self.x + c)

        # Residual: r = y_pred - y_obs
        residuals[0] = y_pred - self.y_obs

        if jacobians is not None:
            # ∂r/∂m = x * exp(m x + c)
            if jacobians[0] is not None:
                jacobians[0][0] = self.x * y_pred

            # ∂r/∂c = exp(m x + c)
            if jacobians[1] is not None:
                jacobians[1][0] = y_pred

        return True



def main():
    # True parameters
    m_true = 0.3
    c_true = 0.1
    sigma = 0.4

    # Generate data
    x = np.linspace(-5, 5, 400)
    y_clean = np.exp(m_true * x + c_true)
    noise = np.random.normal(0.0, sigma, size=x.shape)
    y_noisy = y_clean + noise

    # Initial guesses (what Ceres will optimize)
    m_est = np.array([0.0], dtype=np.float64)
    c_est = np.array([0.0], dtype=np.float64)

    problem = pyceres.Problem()

    # One residual per data point
    for px, py in zip(x, y_noisy):
        problem.add_residual_block(
            ExpResidual(px, py),
            None,
            [m_est, c_est]
        )

    # Solve
    options = pyceres.SolverOptions()
    options.linear_solver_type = pyceres.LinearSolverType.DENSE_NORMAL_CHOLESKY
    options.max_num_iterations = 10000
    options.minimizer_progress_to_stdout = False

    summary = pyceres.SolverSummary()
    pyceres.solve(options, problem, summary)

    print(summary.BriefReport())
    print("Estimated m, c:", m_est[0], c_est[0])

    # Plot results
    y_fit = np.exp(m_est[0] * x + c_est[0])

    plt.figure()
    plt.scatter(x, y_noisy, s=10, alpha=0.5, label="Noisy data")
    plt.plot(x, y_fit, "r", linewidth=2, label="Fitted curve")
    plt.plot(x, y_clean, "k--", label="True curve")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()