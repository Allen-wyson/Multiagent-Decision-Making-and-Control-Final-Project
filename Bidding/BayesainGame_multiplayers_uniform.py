import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# === Parameters ===
rhos = [0, 0.5, 0.8]    # CRRA risk aversion coefficients (rho < 1)
n_list = [2, 3, 4, 5]      # numbers of bidders to plot separately
grid_points = 500       # resolution for valuation grid
epsilon = 1e-6          # small offset to avoid singularity at v=0

# valuation grid for plotting
v_vals = np.linspace(0, 1, grid_points)

# ODE factory for CRRA first-price auction equilibrium with n bidders:
# db/dv = (n-1) * (v - b(v)) / ((1 - rho) * v),  b(0)=0
def make_bidding_ode(n, rho):
    def ode(v, b):
        return (n - 1) * (v - b) / ((1 - rho) * v)
    return ode

# Loop over each n and plot separately
for n in n_list:
    plt.figure(figsize=(6, 4))
    for rho in rhos:
        ode_fun = make_bidding_ode(n, rho)
        sol = solve_ivp(
            fun=ode_fun,
            t_span=(epsilon, 1.0),
            y0=[0.0],
            t_eval=v_vals[1:], 
            rtol=1e-8,
            atol=1e-8
        )
        b_vals = np.concatenate(([0.0], sol.y[0]))  # include b(0)=0
        plt.plot(v_vals, b_vals, label=f'ρ={rho}')
    # analytic risk-neutral (rho=0) line
    b_theory = (n - 1) / n * v_vals
    plt.plot(v_vals, b_theory, '--', color='gray', label='rho=0 analytic')

    plt.xlabel('Valuation v')
    plt.ylabel('Equilibrium bid b(v)')
    plt.title(f'First-Price Auction Equilibrium, n={n}')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
