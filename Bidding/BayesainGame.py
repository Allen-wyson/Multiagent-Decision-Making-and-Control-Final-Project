import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# === Parameters ===
rhos = [0, 0.5, 0.8]    # CRRA risk aversion coefficients (rho < 1)
grid_points = 500       # resolution for valuation grid
epsilon = 1e-6          # small offset to avoid singularity at v=0

# valuation grid for plotting
v_values = np.linspace(0, 1, grid_points)

# Analytic solution for rho=0
b_theory = v_values / 2

# Function factory: returns ODE for CRRA first-price auction equilibrium
# For two bidders, equilibrium satisfies:
#   db/dv = (v - b(v)) / ((1 - rho) * v),   b(0)=0
def make_bidding_ode(rho):
    def ode(v, b):
        return (v - b) / ((1 - rho) * v)
    return ode

# Solve ODE for each rho and store the solution
solutions = {}
for rho in rhos:
    ode_func = make_bidding_ode(rho)
    sol = solve_ivp(
        fun=ode_func,
        t_span=(epsilon, 1.0),
        y0=[0.0],
        t_eval=v_values[1:],   # skip v=0 to avoid division by zero
        rtol=1e-8,
        atol=1e-8
    )
    # prepend b(0)=0
    b_vals = np.concatenate(([0.0], sol.y[0]))
    solutions[rho] = b_vals

# === Plotting ===
plt.figure(figsize=(8, 6))
for rho, b_vals in solutions.items():
    plt.plot(v_values, b_vals, label=f'rho={rho}')
plt.plot(v_values, b_theory, '--', color='gray', label='analytic rho=0: b(v)=v/2')

plt.xlabel('Valuation v')
plt.ylabel('Equilibrium bid b(v)')
plt.title('First-Price Auction Equilibrium with CRRA Utility (ODE Solution)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
