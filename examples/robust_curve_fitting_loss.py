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
    
    #Outlier noise
    sigma2 = 10.0
    outlier_prob = 0.1

    # Generate data
    x = np.linspace(-5, 5, 400)
    y_clean = np.exp(m_true * x + c_true)
    noise = np.random.normal(0.0, sigma, size=x.shape)
    outliers = np.random.rand(len(x)) < outlier_prob
    noise[outliers] += np.random.normal(0.0, sigma2, size=np.sum(outliers))


    y_noisy = y_clean + noise

    # Initial guesses (what Ceres will optimize)
    m_est_no_loss = np.array([0.0], dtype=np.float64)
    c_est_no_loss = np.array([0.0], dtype=np.float64)
    
    m_est_loss = np.array([0.0], dtype=np.float64)
    c_est_loss = np.array([0.0], dtype=np.float64)

    problem_no_loss = pyceres.Problem()
    problem_loss = pyceres.Problem()

    # One residual per data point
    for px, py in zip(x, y_noisy):
        problem_no_loss.add_residual_block(
            ExpResidual(px, py),
            None,
            [m_est_no_loss, c_est_no_loss]
        )
        
    for px, py in zip(x, y_noisy):
        problem_loss.add_residual_block(
            ExpResidual(px, py),
            pyceres.CauchyLoss(0.5),
            [m_est_loss, c_est_loss]
        )

    # Solve
    options = pyceres.SolverOptions()
    options.linear_solver_type = pyceres.LinearSolverType.DENSE_NORMAL_CHOLESKY
    options.max_num_iterations = 50
    options.minimizer_progress_to_stdout = False

    summary_no_loss = pyceres.SolverSummary()
    pyceres.solve(options, problem_no_loss, summary_no_loss)
    summary_loss = pyceres.SolverSummary()
    pyceres.solve(options, problem_loss, summary_loss)

    print(summary_no_loss.BriefReport())
    
    print("Estimated m, c:", m_est_no_loss[0], c_est_no_loss[0])
    print("Estimated m, c:", m_est_loss[0], c_est_loss[0])
    
    # Plot results
    y_fit1 = np.exp(m_est_no_loss[0] * x + c_est_no_loss[0])
    y_fit2 = np.exp(m_est_loss[0] * x + c_est_loss[0])

    plt.figure()
    plt.scatter(x, y_noisy, s=10, alpha=0.5, label="Noisy data")
    plt.plot(x, y_fit1, "r", linewidth=2, label="Simple Fitted Curve")
    plt.plot(x, y_fit2, "b", linewidth=2, label="Robust Fitted Curve")
    plt.plot(x, y_clean, "k--", label="True curve")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()