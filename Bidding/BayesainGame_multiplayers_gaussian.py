import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.stats import norm
from scipy.interpolate import interp1d

# === Parameters ===
n_list    = [2, 3, 4, 5]
rhos      = [0, 0.5, 0.8]
grid_pts  = 500
epsilon   = 1e-6

# valuation grid
v_vals = np.linspace(epsilon, 1, grid_pts)

# --- Replace Beta(2,2) with a truncated Gaussian ---
mu, sigma = 0.5, 0.15
dist       = norm(loc=mu, scale=sigma)
# compute pdf and cdf on [0,1]
f_raw  = dist.pdf(v_vals)
F_raw  = dist.cdf(v_vals)
# renormalize so ∫₀¹ f = 1, and F(0)=0, F(1)=1
Z      = dist.cdf(1) - dist.cdf(0)
f_vals = f_raw / Z
F_vals = (F_raw - dist.cdf(0)) / Z

# build interpolators
F_interp = interp1d(v_vals, F_vals, fill_value="extrapolate")
f_interp = interp1d(v_vals, f_vals, fill_value=0)

# ODE factory (unchanged)
def make_ode(n, rho):
    def ode(v, b):
        Fv = F_interp(v)
        return (n - 1) * f_interp(v) * (v - b) / ((1 - rho) * Fv)
    return ode

# Plot the Gaussian PDF
plt.figure(figsize=(6,4))
plt.plot(v_vals, f_vals, color='blue', lw=2)
plt.xlabel('Valuation v')
plt.ylabel('Density f(v)')
plt.title('Truncated Normal(0.5,0.15²) Value PDF')
plt.grid(True)
plt.tight_layout()
plt.show()

# Then continue with the same loop as before:
for n in n_list:
    plt.figure(figsize=(6,4))
    for rho in rhos:
        sol = solve_ivp(
            make_ode(n, rho),
            (epsilon, 1.0),
            [(n-1)/n * epsilon],
            t_eval=v_vals,
            rtol=1e-8, atol=1e-8
        )
        plt.plot(sol.t, sol.y[0], label=f'ρ={rho}')
    plt.xlabel('Valuation v')
    plt.ylabel('Equilibrium bid b(v)')
    plt.title(f'First-Price Auction Equilibrium, n={n}, Normal(0.5,0.15²)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
