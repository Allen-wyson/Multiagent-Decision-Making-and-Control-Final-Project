function x_next = vehicleDynamics(x, u, Ts, L)
%VEHICLEDYNAMICS  Discrete-time bicycle model integration using RK4
%   x_next = vehicleDynamics(x, u, Ts, L)
%   Inputs:
%     x  : current state [x; y; psi; v]
%     u  : control input [delta; a] (steering angle [rad], acceleration [m/s^2])
%     Ts : time step [s]
%     L  : wheelbase [m]
%   Output:
%     x_next : next state after Ts using 4th-order Runge-Kutta

% Define dynamics as nested function
dynamics = @(state, control) [ ...
    state(4) * cos(state(3)); ...  % dx/dt
    state(4) * sin(state(3)); ...  % dy/dt
    state(4) / L * tan(control(1)); ... % dpsi/dt
    control(2) ...                % dv/dt
];

% Retrieve controls
delta = u(1);
a     = u(2);
control = [delta; a];

% RK4 integration
k1 = dynamics(x,          control);
k2 = dynamics(x + 0.5*Ts*k1, control);
k3 = dynamics(x + 0.5*Ts*k2, control);
k4 = dynamics(x + Ts*k3,    control);

x_next = x + Ts/6 * (k1 + 2*k2 + 2*k3 + k4);
end
