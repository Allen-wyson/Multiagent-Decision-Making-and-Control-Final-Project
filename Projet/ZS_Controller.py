import numpy as np
import cvxpy as cp
from car import Car
from car_dynamics import bicycle_dynamics_discrete
from frenet import get_frenet_coords
from scipy.signal import cont2discrete
from scipy.interpolate import interp1d

def linearize_bicycle_dynamics(x_ref, u_ref, L=2.5):
    _, _, theta, v = x_ref
    a, delta = u_ref

    A = np.zeros((4, 4))
    B = np.zeros((4, 2))

    A[0, 2] = -v * np.sin(theta)
    A[0, 3] = np.cos(theta)
    A[1, 2] = v * np.cos(theta)
    A[1, 3] = np.sin(theta)
    A[2, 3] = 1.0 / L * np.tan(delta)
    B[2, 1] = v / L / (np.cos(delta)**2)
    B[3, 0] = 1.0

    return A, B

def discretize_linear_dynamics(A, B, dt):
    C = np.eye(A.shape[0])
    D = np.zeros(B.shape)
    system = (A, B, C, D)
    A_d, B_d, _, _, _ = cont2discrete(system, dt, method='zoh')
    return A_d, B_d



def solve_zero_sum_qp_P1(
    car1, car2,
    d_opt1, d_opt2, centerline_frenet,
    centerline, headings, s_goal, s_next,
    P2_control_input,
    N=10, dt=0.1, d_g=1.0, d_u=1.0, d_t=0.2, d_next=2.0, d_c=100.0
):
    n, m = 4, 2
    X1 = cp.Variable((n, N+1))
    U1 = cp.Variable((m, N))
    U2 = P2_control_input
    #slack = cp.Variable(N)
    x1_0 = car1.get_state()
    x2_0 = car2.get_state()
    s1_0, d1_0 = car1.get_frenet_coords(centerline, headings)

    constraints = [
        X1[:, 0] == x1_0,
    ]

    cost = 0.0

    # Assume horizon is short enough to  linearize around x_0
    A1, B1 = linearize_bicycle_dynamics(x1_0, [0.0, 0.0])
    A1_d, B1_d = discretize_linear_dynamics(A1, B1, dt)

    # Create interpolators locally 
    d_opt_interp1 = interp1d(centerline_frenet, d_opt1, kind='linear', fill_value='extrapolate')
    d_opt_interp2 = interp1d(centerline_frenet, d_opt2, kind='linear', fill_value='extrapolate')

    # Compute desired lateral offset
    d_goal1 = d_opt_interp1(s1_0)

    # Compute parameters for frenet coordinates linearization of car 1
    idx1 = np.argmin(np.abs(centerline_frenet - s1_0))
    theta_c1 = headings[idx1]
    t_hat1 = np.array([np.cos(theta_c1), np.sin(theta_c1)])
    n_hat1 = np.array([-np.sin(theta_c1), np.cos(theta_c1)])
    p1_0 = x1_0[0:2]

    # Rotation matrix for whole horizon (assume horizon short enough for the cars to keep the same orientation during whole length)
    theta_rel = 0.5 * (x1_0[2] + x2_0[2])  # average heading of the cars
    R = np.array([[np.cos(theta_rel), np.sin(theta_rel)],
                  [-np.sin(theta_rel), np.cos(theta_rel)]])
    
    # Initialize state variables for car 2
    x2_k = x2_0

    for k in range(N):
        
        # Constraints on dynamics
        constraints += [X1[:, k+1] == A1_d @ X1[:, k] + B1_d @ U1[:, k]]

        # Box constraints on speed, acceleration, and steering
        #constraints += [X1[3, k] <= car1.v_max, X1[3, k] >= 0.0]
        #constraints += [U1[0, k] <= car1.a_max, U1[0, k] >= -car1.a_max]
        #constraints += [U1[1, k] <= car1.max_steering_angle, U1[1, k] >= -car1.max_steering_angle]

        # Collision constraints using an ellipse rotated in the direction of the cars (both cars cannot be in the ellipse at the same time)
        delta = X1[0:2, k] - x2_k[0:2]
        rel_rot = R @ delta
        collision_expr = cp.quad_form(rel_rot, np.diag([1/1.0**2, 1/2.0**2]))
        #constraints += [collision_expr + slack[k] >= 1.0]
        #cost += d_c * slack[k]

        # Slack variables always positive
        #constraints += [slack[k] >= 0]

        # Linear approximation of frenet coordinates for X1[0:2,k]
        s1 = s1_0 + t_hat1 @ (X1[0:2, k] - p1_0)
        d1 = d1_0 + n_hat1 @ (X1[0:2, k] - p1_0)
        
        # Use real frenet coordiantes for car 2
        s2, d2 = get_frenet_coords(x2_k[0], x2_k[1], centerline, headings)
        
        # Update optimal trajectory goal
        d_goal2 = d_opt_interp2(s2)

        # Goal cost
        cost += d_g * cp.square(s_goal - s1) - d_g * cp.square(s_goal - s2)
        
        # Optimal trajectory cost
        cost += d_t * cp.square(d_goal1 - d1) - d_t * cp.square(d_goal2 - d2)
        
        # Input cost
        cost += d_u * cp.sum_squares(U1[:, k]) - d_u * cp.sum_squares(U2[:, k])
        
        # Next waypoint cost
        cost += d_next * cp.square(s_next - s1) - d_next * cp.square(s_next - s2)

        # Update state of car 2
        x2_k = bicycle_dynamics_discrete(x2_k, U2[:,k], dt)

    # Linear approximation of frenet coordinates for  X[0:2,N]
    s1_N = s1_0 + t_hat1 @ (X1[0:2, N] - p1_0)
    d1_N = d1_0 + n_hat1 @ (X1[0:2, N] - p1_0)

    # Real frenet coordinates for X2_N
    s2_N, d2_N = get_frenet_coords(x2_k[0], x2_k[1], centerline, headings)

    # Goal cost
    cost += d_g * cp.square(s_goal - s1_N) - d_g * cp.square(s_goal - s2_N)

    # Optimal trajectory cost
    cost += d_t * cp.square(d_goal1 - d1_N) - d_t * cp.square(d_goal2 - d2_N)

    # Next waypoint cost
    cost += d_next * cp.square(s_next - s1_N) - d_next * cp.square(s_next - s2_N)

    # Problem solving
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP)

    # Check if solver succeeded
    if prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        print("[Warning] Car 1 QP solver failed:", prob.status)
        return np.zeros((2, N))

    return U1.value

def solve_zero_sum_qp_P2(
    car1, car2,
    d_opt1, d_opt2, centerline_frenet,
    centerline, headings, s_goal, s_next,
    P1_control_input,
    N=10, dt=0.1, d_g=1.0, d_u=1.0, d_t=0.2, d_next=2.0, d_c=100.0
):
    n, m = 4, 2
    X2 = cp.Variable((n, N+1))
    U2 = cp.Variable((m, N))
    U1 = P1_control_input
    #slack = cp.Variable(N)
    x1_0 = car1.get_state()
    x2_0 = car2.get_state()
    s2_0, d2_0 = car2.get_frenet_coords(centerline, headings)

    constraints = [
        X2[:, 0] == x2_0,
    ]

    cost = 0.0

    # Assume horizon is short enough to  linearize around x_0
    A2, B2 = linearize_bicycle_dynamics(x2_0, [0.0, 0.0])
    A2_d, B2_d = discretize_linear_dynamics(A2, B2, dt)

    # Create interpolators locally 
    d_opt_interp1 = interp1d(centerline_frenet, d_opt1, kind='linear', fill_value='extrapolate')
    d_opt_interp2 = interp1d(centerline_frenet, d_opt2, kind='linear', fill_value='extrapolate')

    # Compute desired lateral offset
    d_goal2 = d_opt_interp2(s2_0)

    # Compute parameters for frenet coordinates linearization of car 2
    idx2 = np.argmin(np.abs(centerline_frenet - s2_0))
    theta_c2 = headings[idx2]
    t_hat2 = np.array([np.cos(theta_c2), np.sin(theta_c2)])
    n_hat2 = np.array([-np.sin(theta_c2), np.cos(theta_c2)])
    p2_0 = x2_0[0:2]

    # Rotation matrix for whole horizon (assume horizon short enough for the cars to keep the same orientation during whole length)
    theta_rel = 0.5 * (x1_0[2] + x2_0[2])  # average heading of the cars
    R = np.array([[np.cos(theta_rel), np.sin(theta_rel)],
                  [-np.sin(theta_rel), np.cos(theta_rel)]])
    
    # Initialize state variables for car 1
    x1_k = x1_0

    for k in range(N):
        
        # Constraints on dynamics
        constraints += [X2[:, k+1] == A2_d @ X2[:, k] + B2_d @ U2[:, k]]

        # Box constraints on speed, acceleration, and steering
        #constraints += [X2[3, k] <= car2.v_max, X2[3, k] >= 0.0]
        #constraints += [U2[0, k] <= car2.a_max, U2[0, k] >= -car2.a_max]
        #constraints += [U2[1, k] <= car2.max_steering_angle, U2[1, k] >= -car2.max_steering_angle]

        # Collision constraints using an ellipse rotated in the direction of the cars (both cars cannot be in the ellipse at the same time)
        delta = x1_k[0:2] - X2[0:2, k]
        rel_rot = R @ delta
        collision_expr = cp.quad_form(rel_rot, np.diag([1/1.0**2, 1/2.0**2]))
        #constraints += [collision_expr + slack[k] >= 1.0]
        #cost += d_c * slack[k]

        # Slack variables always positive
        #constraints += [slack[k] >= 0]

        # Linear approximation of frenet coordinates for X2[0:2,k]
        s2 = s2_0 + t_hat2 @ (X2[0:2, k] - p2_0)
        d2 = d2_0 + n_hat2 @ (X2[0:2, k] - p2_0)
        
        # Use real frenet coordiantes for car 2
        s1, d1 = get_frenet_coords(x1_k[0], x1_k[1], centerline, headings)
        
        # Update optimal trajectory goal
        d_goal1 = d_opt_interp1(s1)

        # Goal cost
        cost += d_g * cp.square(s_goal - s1) - d_g * cp.square(s_goal - s2)
        
        # Optimal trajectory cost
        cost += d_t * cp.square(d_goal1 - d1) - d_t * cp.square(d_goal2 - d2)
        
        # Input cost
        cost += d_u * cp.sum_squares(U1[:, k]) - d_u * cp.sum_squares(U2[:, k])
        
        # Next waypoint cost
        cost += d_next * cp.square(s_next - s1) - d_next * cp.square(s_next - s2)

        # Update state of car 1
        x1_k = bicycle_dynamics_discrete(x1_k, U1[:,k], dt)

    # Linear approximation of frenet coordinates for X2[0:2,N]
    s2_N = s2_0 + t_hat2 @ (X2[0:2, N] - p2_0)
    d2_N = d2_0 + n_hat2 @ (X2[0:2, N] - p2_0)

    # Real frenet coordinates for X2_N
    s1_N, d1_N = get_frenet_coords(x1_k[0], x1_k[1], centerline, headings)

    # Goal cost
    cost += d_g * cp.square(s_goal - s1_N) - d_g * cp.square(s_goal - s2_N)
    
    # Optimal trajectory cost
    cost += d_t * cp.square(d_goal1 - d1_N) - d_t * cp.square(d_goal2 - d2_N)
    
    # Next waypoint cost
    cost += d_next * cp.square(s_next - s1_N) - d_next * cp.square(s_next - s2_N)
    
    # Solve the problem
    prob = cp.Problem(cp.Minimize(-cost), constraints)
    prob.solve(solver=cp.OSQP)

    # Check if solver succeeded
    if prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        print("[Warning] Car 2 QP solver failed:", prob.status)
        return np.zeros((2, N))

    return U2.value

def zero_sum_best_response(
    car1, car2,
    d_opt1, d_opt2, centerline_frenet,
    centerline, headings, s_goal, s_next,
    N=10, dt=0.1, d_g=1.0, d_u=1.0, d_t=20.0, d_next=20.0, d_c=100.0,
    max_iters=5, epsilon=1e-6
):
    np.random.seed(42)
    m = 2  # dimension of control input
    U1 = np.zeros((m, N))
    U2 = np.random.randn(m, N)

    for i in range(max_iters):
        print(f"  Iter {i+1}/{max_iters}")

        # Step 1: Minimize for Car 1 given U2
        U1_new = solve_zero_sum_qp_P1(
            car1, car2,
            d_opt1, d_opt2, centerline_frenet,
            centerline, headings, s_goal, s_next,
            U2, N, dt, d_g, d_u, d_t, d_next, d_c
        )

        # Step 2: Maximize for Car 2 given new U1
        U2_new = solve_zero_sum_qp_P2(
            car1, car2,
            d_opt1, d_opt2, centerline_frenet,
            centerline, headings, s_goal, s_next,
            U1_new, N, dt, d_g, d_u, d_t, d_next, d_c
        )

        # Check if solution converged
        delta_U1 = np.linalg.norm(U1_new - U1)
        delta_U2 = np.linalg.norm(U2_new - U2)
        if delta_U1 < epsilon and delta_U2 < epsilon:
            break

        U1, U2 = U1_new, U2_new

    return U1[:, 0], U2[:, 0]