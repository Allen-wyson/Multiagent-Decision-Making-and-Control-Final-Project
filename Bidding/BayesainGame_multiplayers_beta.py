import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.stats import beta
from scipy.interpolate import interp1d

# === Parameters ===
n_list    = [2, 3, 4, 5]    # number of bidders
rhos      = [0, 0.5, 0.8]   # CRRA risk aversion coefficients
grid_pts  = 500             # resolution for valuation grid
epsilon   = 1e-6            # to avoid singularity at v=0

# valuation grid
v_vals = np.linspace(epsilon, 1, grid_pts)

# Beta(2,2) distribution on [0,1]
dist   = beta(a=2, b=2)
F_vals = dist.cdf(v_vals)
f_vals = dist.pdf(v_vals)

# ODE factory for CRRA, non-uniform IPV:
# db/dv = (n-1) * f(v) * (v - b) / ((1 - rho) * F(v)),  b(0)=0
F_interp = interp1d(v_vals, F_vals, fill_value="extrapolate")
f_interp = interp1d(v_vals, f_vals, fill_value=0)
def make_ode(n, rho):
    def ode(v, b):
        Fv = F_interp(v)
        return (n - 1) * f_interp(v) * (v - b) / ((1 - rho) * Fv)
    return ode

# === Plot Beta(2,2) PDF ===
plt.figure(figsize=(6,4))
plt.plot(v_vals, f_vals, color='blue', lw=2)
plt.xlabel('Valuation v')
plt.ylabel('Density f(v)')
plt.title('Beta(2,2) Value Distribution (PDF)')
plt.grid(True)
plt.tight_layout()
plt.show()

# === Plot equilibrium bid curves for each n ===
for n in n_list:
    plt.figure(figsize=(6,4))
    for rho in rhos:
        sol = solve_ivp(
            make_ode(n, rho),
            (epsilon, 1.0),
            [(n-1)/n * epsilon],   # b(epsilon) ≈ (n-1)/n * epsilon
            t_eval=v_vals,
            rtol=1e-8, atol=1e-8
        )
        plt.plot(sol.t, sol.y[0], label=f'ρ={rho}')
    plt.xlabel('Valuation v')
    plt.ylabel('Equilibrium bid b(v)')
    plt.title(f'First-Price Auction Equilibrium, n={n}, Beta(2,2)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
