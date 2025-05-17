function [U_opt, X_opt] = solveMPC(x0, U_init, s0, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L)
%SOLVEMPC  Receding-horizon MPC using Frenet coordinates and CasADi (MX)
%   [U_opt,X_opt] = solveMPC(x0,U_init,s0,s_vals,ppX,ppY,Ts,N,w_s,w_d,w_u,L)

import casadi.*;

% Prepare track data
d_data  = s_vals;
x_data  = ppval(ppX, d_data);
y_data  = ppval(ppY, d_data);
dx_data = ppval(fnder(ppX), d_data);
dy_data = ppval(fnder(ppY), d_data);

% Create CasADi interpolants (B-spline, MX-compatible)
x_center = interpolant('x_center', 'bspline', {d_data}, x_data);
y_center = interpolant('y_center', 'bspline', {d_data}, y_data);
dx_ds    = interpolant('dx_ds',    'bspline', {d_data}, dx_data);
dy_ds    = interpolant('dy_ds',    'bspline', {d_data}, dy_data);

% Decision variables: controls U, states X, arc-length S (using MX)
U = MX.sym('U', 2, N);
X = MX.sym('X', 4, N+1);
S = MX.sym('S', 1, N+1);
opt_vars = vertcat(reshape(U,2*N,1), reshape(X,4*(N+1),1), S');

% Initialize constraints and objective
g = [];
J = 0;

% Initial constraints: state and arc
g = [g; X(:,1) - x0; S(1) - s0];

% Horizon loop
for k = 1:N
xk = X(:,k);
uk = U(:,k);
sk = S(k);
% RK4 dynamics
k1 = kin(xk,uk,L);
k2 = kin(xk + Ts/2*k1,uk,L);
k3 = kin(xk + Ts/2*k2,uk,L);
k4 = kin(xk + Ts*k3,  uk,L);
x_next = xk + Ts/6*(k1 + 2*k2 + 2*k3 + k4);
g = [g; X(:,k+1) - x_next];
% Frenet frame
xc = x_center(sk);
yc = y_center(sk);
dxr = dx_ds(sk);
dyr = dy_ds(sk);
psi_ref = atan2(dyr,dxr);
% Lateral deviation
d_err = -(xk(1)-xc)*sin(psi_ref) + (xk(2)-yc)*cos(psi_ref);
% Running cost
J = J + w_d*d_err^2 + w_u*(uk(1)^2 + uk(2)^2);
% Arc update
s_next = sk + xk(4)*cos(xk(3)-psi_ref)*Ts;
g = [g; S(k+1) - s_next];
end

% Terminal cost: reward progress
J = J - w_s * S(N+1);

% Concatenate constraints
g = vertcat(g);

% Bounds for variables
delta_max = pi/6; a_max = 3; v_min = 0; v_max = 20;

% Control bounds
lb_u = repmat([-delta_max; -a_max], N,1);
ub_u = repmat([ delta_max;  a_max], N,1);
% State bounds
lb_x = repmat([-inf; -inf; -inf; v_min], N+1,1);
ub_x = repmat([ inf;  inf;  inf; v_max], N+1,1);
% Arc bounds
lb_s = repmat(d_data(1), N+1,1);
ub_s = repmat(d_data(end), N+1,1);

% Combine bounds
lbx = [lb_u; lb_x; lb_s];
ubx = [ub_u; ub_x; ub_s];
% Equality constraints
lbg = zeros(size(g));
ubg = zeros(size(g));

% Setup NLP
nlp = struct('x',opt_vars,'f',J,'g',g);
opts = struct('ipopt',struct('print_level',0));
solver = nlpsol('solver','ipopt',nlp,opts);

% Initial guess
dec0 = [reshape(U_init,2*N,1); repmat(x0, N+1,1); repmat(s0, N+1,1)];

% Solve
res = solver('x0',dec0,'lbx',lbx,'ubx',ubx,'lbg',lbg,'ubg',ubg);
sol = full(res.x);

% Extract solution
U_opt = reshape(sol(1:2*N),2,N);
X_opt = reshape(sol(2*N+1:2*N+4*(N+1)),4,N+1);
end

function dx = kin(x,u,L)
%KIN  Vehicle kinematic derivatives for RK4
dx = [ x(4)*cos(x(3)); x(4)*sin(x(3)); x(4)/L*tan(u(1)); u(2) ];
end

