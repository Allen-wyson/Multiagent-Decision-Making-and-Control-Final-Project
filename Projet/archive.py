def solve_zero_sum_qp(
    car1, car2,
    d_opt1, d_opt2, centerline_frenet,
    centerline, headings, s_goal,
    N=10, dt=0.1, d_g=1.0, d_u=0.1, d_t=1.0, d_c=100.0
):
    n, m = 4, 2
    X1 = cp.Variable((n, N+1))
    U1 = cp.Variable((m, N))
    X2 = cp.Variable((n, N+1))
    U2 = cp.Variable((m, N))
    slack = cp.Variable(N)
    x1_0 = car1.get_state()
    x2_0 = car2.get_state()
    s1_0, d1_0 = car1.get_frenet_coords(centerline, headings)
    s2_0, d2_0 = car2.get_frenet_coords(centerline, headings)

    constraints = [
        X1[:, 0] == x1_0,
        X2[:, 0] == x2_0
    ]

    cost = 0.0

    # Assume horizon is short enough to  linearize around x_0
    A1, B1 = linearize_bicycle_dynamics(x1_0, [0.0, 0.0])
    A2, B2 = linearize_bicycle_dynamics(x2_0, [0.0, 0.0])
    A1_d, B1_d = discretize_linear_dynamics(A1, B1, dt)
    A2_d, B2_d = discretize_linear_dynamics(A2, B2, dt)

    
    # Create interpolators locally 
    d_opt_interp1 = interp1d(centerline_frenet, d_opt1, kind='linear', fill_value='extrapolate')
    d_opt_interp2 = interp1d(centerline_frenet, d_opt2, kind='linear', fill_value='extrapolate')

    # Compute desired lateral offset
    d_goal1 = d_opt_interp1(s1_0)
    d_goal2 = d_opt_interp2(s2_0)

    # Compute parameters for frenet coordinates linearization
    idx1 = np.argmin(np.abs(centerline_frenet - s1_0))
    idx2 = np.argmin(np.abs(centerline_frenet - s2_0))
    theta_c1 = headings[idx1]
    theta_c2 = headings[idx2]
    t_hat1 = np.array([np.cos(theta_c1), np.sin(theta_c1)])
    t_hat2 = np.array([np.cos(theta_c2), np.sin(theta_c2)])
    n_hat1 = np.array([-np.sin(theta_c1), np.cos(theta_c1)])
    n_hat2 = np.array([-np.sin(theta_c2), np.cos(theta_c2)])
    p1_0 = x1_0[0:2]
    p2_0 = x2_0[0:2]

    # Rotation matrix for whole horizon (assume horizon short enough for the cars to keep the same orientation during whole length)
    theta_rel = 0.5 * (x1_0[2] + x2_0[2])  # average heading of the cars
    R = np.array([[np.cos(theta_rel), np.sin(theta_rel)],
                  [-np.sin(theta_rel), np.cos(theta_rel)]])
    
    
    for k in range(N):
        # Constraints on dynamics
        constraints += [X1[:, k+1] == A1_d @ X1[:, k] + B1_d @ U1[:, k]]
        constraints += [X2[:, k+1] == A2_d @ X2[:, k] + B2_d @ U2[:, k]]

        # Box constraints on speed, acceleration, and steering
        constraints += [X1[3, k] <= car1.v_max, X1[3, k] >= 0.0]
        constraints += [X2[3, k] <= car2.v_max, X2[3, k] >= 0.0]

        constraints += [U1[0, k] <= car1.a_max, U1[0, k] >= -car1.a_max]
        constraints += [U2[0, k] <= car2.a_max, U2[0, k] >= -car2.a_max]

        constraints += [U1[1, k] <= car1.max_steering_angle, U1[1, k] >= -car1.max_steering_angle]
        constraints += [U2[1, k] <= car2.max_steering_angle, U2[1, k] >= -car2.max_steering_angle]

        # Collision constraints using an ellipse rotated in the direction of the cars (both cars cannot be in the ellipse at the same time)
        delta = X1[0:2, k] - X2[0:2, k]
        rel_rot = R @ delta
        collision_expr = cp.quad_form(rel_rot, np.diag([1/1.0**2, 1/2.0**2]))
        constraints += [collision_expr + slack[k] >= 1.0]
        cost += d_c * slack[k]

        # Slack variables always positive
        constraints += [slack[k] >= 0]


        # Linear approximation of frenet coordinates for X[0:2,k]
        s1 = s1_0 + t_hat1 @ (X1[0:2, k] - p1_0)
        s2 = s2_0 + t_hat2 @ (X2[0:2, k] - p2_0)
        d1 = d1_0 + n_hat1 @ (X1[0:2, k] - p1_0)
        d2 = d2_0 + n_hat2 @ (X2[0:2, k] - p2_0)

        cost += d_g * cp.square(s_goal - s1) - d_g * cp.square(s_goal - s2)
        cost += d_t * cp.square(d_goal1 - d1) - d_t * cp.square(d_goal2 - d2)
        cost += -d_u * cp.sum_squares(U1[:, k]) + d_u * cp.sum_squares(U2[:, k])

    # Linear approximation of frenet coordinates for  X[0:2,N]
    s1_N = s1_0 + t_hat1 @ (X1[0:2, N] - p1_0)
    s2_N = s2_0 + t_hat2 @ (X2[0:2, N] - p2_0)
    d1_N = d1_0 + n_hat1 @ (X1[0:2, N] - p1_0)
    d2_N = d2_0 + n_hat2 @ (X2[0:2, N] - p2_0)
    cost += d_g * cp.square(s_goal - s1_N) - d_g * cp.square(s_goal - s2_N)
    cost += d_t * cp.square(d_goal1 - d1_N) - d_t * cp.square(d_goal2 - d2_N)
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP)

    return U1[:, 0].value, U2[:, 0].value