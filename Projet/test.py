import numpy as np
import matplotlib.pyplot as plt
from racetrack import generate_track
from optimal_trajectory import generate_optimal_traj
from car import Car
from ZS_Controller import zero_sum_best_response

# --- Generate Track ---
track = generate_track()
track.plot()
centerline = np.column_stack((track.xm, track.ym))
headings = track.th
# Remove duplicate (xm, ym) points
_, unique_indices = np.unique(centerline, axis=0, return_index=True)
centerline = centerline[np.sort(unique_indices)]
headings = track.th[np.sort(unique_indices)]
centerline_frenet = s_ref = np.insert(np.cumsum(np.hypot(np.diff(centerline[:, 0]), np.diff(centerline[:, 1]))), 0, 0)

# --- Initialize Cars ---
g = 9.81
car1 = Car([track.xm[0], track.ym[0]+2, headings[0], 5.0], v_max=65.0, a_max=12.0, a_lat_max=1.5*g)
car2 = Car([track.xm[0], track.ym[0]-2, headings[0], 5.0], v_max=75.0, a_max=10.0, a_lat_max=2*g)

# --- Generate Optimal Trajectories ---
d_opt1 = generate_optimal_traj(centerline_frenet, track.width, car1.a_lat_max)
d_opt2 = generate_optimal_traj(centerline_frenet, track.width, car2.a_lat_max)

# --- Simulation ---
s_goal = centerline_frenet[-1]
N = 20
dt = 0.05
T = int(10/dt)
for t in range(T):
    print(f"Step {t+1}/{T}")
    
    # Get current Frenet coordinates
    s1, _ = car1.get_frenet_coords(centerline, headings)
    s2, _ = car2.get_frenet_coords(centerline, headings)

    # Average s position
    s_avg = 0.5 * (s1 + s2)

    # Find the closest index in the centerline
    idx_closest = np.argmin(np.abs(centerline_frenet - s_avg))

    # Move one step forward (saturate to avoid overflow)
    idx_next = min(idx_closest + 1, len(centerline_frenet) - 1)

    # Next reference point in s
    s_next = centerline_frenet[idx_next]
    
    u1, u2 = zero_sum_best_response(
        car1, car2,
        d_opt1, d_opt2, centerline_frenet,
        centerline, headings, s_goal, s_next,
        N=N, dt=dt
    )
    car1.set_control(*u1)
    car2.set_control(*u2)
    car1.update()
    car2.update()

# --- Plotting ---
h1 = np.array(car1.history)
h2 = np.array(car2.history)

plt.figure(figsize=(10, 8))
plt.plot(track.xm, track.ym, 'k--', label='Centerline')
plt.plot(track.xb1, track.yb1, 'g--', label='Inner Boundary')
plt.plot(track.xb2, track.yb2, 'r--', label='Outer Boundary')
plt.plot(h1[:, 0], h1[:, 1], 'b-', label='Car 1')
plt.plot(h2[:, 0], h2[:, 1], 'orange', label='Car 2')
plt.axis('equal')
plt.legend()
plt.title("Zero-Sum Racing Simulation")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.show()