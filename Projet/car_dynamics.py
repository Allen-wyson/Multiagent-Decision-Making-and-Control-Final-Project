import numpy as np

def bicycle_dynamics_continuous(state, control, L=2.5):
    """Continuous-time bicycle model dynamics."""
    x, y, theta, v = state
    a, delta = control

    dxdt = v * np.cos(theta)
    dydt = v * np.sin(theta)
    dthetadt = v / L * np.tan(delta)
    dvdt = a

    return np.array([dxdt, dydt, dthetadt, dvdt])


def rk4_integration(state, control, dt, dynamics):
    """One RK4 integration step."""
    k1 = dynamics(state, control)
    k2 = dynamics(state + 0.5 * dt * k1, control)
    k3 = dynamics(state + 0.5 * dt * k2, control)
    k4 = dynamics(state + dt * k3, control)

    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


def bicycle_dynamics_discrete(state, control, dt, L=2.5):
    """Bicycle model dynamics discretized using RK4"""
    new_state = rk4_integration(state, control, dt, lambda s, u: bicycle_dynamics_continuous(s, u, L))
    return new_state