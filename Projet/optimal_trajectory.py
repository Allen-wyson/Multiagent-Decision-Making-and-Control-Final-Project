import numpy as np
from scipy.optimize import minimize

def lap_time_cost_with_smoothness(d, s, a_lat_max=9.0, w_smooth=0.5, w_dev=0.1):
    ds = np.gradient(s)
    d2_ds2 = np.gradient(np.gradient(d, s), s)
    curvature = np.abs(d2_ds2)
    curvature = np.clip(curvature, 1e-4, None)

    v_max = np.sqrt(a_lat_max / curvature)
    segment_times = ds / v_max
    lap_time = np.sum(segment_times)

    # Smoothness: penalize fast changes in lateral position
    smoothness = np.sum(np.diff(d, 2)**2)

    # Deviation from centerline (optional)
    deviation = np.sum(d**2)

    total_cost = lap_time + w_smooth * smoothness + w_dev * deviation
    
    return total_cost

def generate_optimal_traj(centerline_frenet, track_width, a_lat_max=9.0):
    """
    Minimize lap time by optimizing lateral offset along centerline.
    """
    N = len(centerline_frenet)
    max_dev = track_width / 2 * 0.95
    np.random.seed(42)
    initial_d = np.random.randn(N)*max_dev
    bounds = [(-max_dev, max_dev)] * N

    res = minimize(
        lap_time_cost_with_smoothness,
        initial_d,
        args=(centerline_frenet, a_lat_max),
        bounds=bounds,
        method='L-BFGS-B'
    )

    return res.x