function [U_opt, X_opt, S_opt, E_coll] = solveMPC_game_collision(...
    x0, U_init, s0, s_vals, ppX, ppY, Ts, N, ...
    w_s, w_d, w_u, w_c, L, S_opp, X_opp, track_width)
%SOLVEMPC_GAME_COLLISION  Zero-sum Frenet MPC + soft collision + track-width constraints
%   [U_opt,X_opt,S_opt,E_coll] = solveMPC_game_collision(...
%       x0,U_init,s0,s_vals,ppX,ppY,Ts,N, ...
%       w_s,w_d,w_u,w_c,L,S_opp,X_opp,track_width)
%   If track_width is not provided, defaults to 5 [m].

import casadi.*

%--- Default track width
if nargin < 16
    track_width = 5.0;
end

%--- Track interpolants
d_data  = s_vals;
x_data  = ppval(ppX, d_data);
y_data  = ppval(ppY, d_data);
dx_data = ppval(fnder(ppX), d_data);
dy_data = ppval(fnder(ppY), d_data);
x_center = interpolant('x_center','bspline',{d_data},x_data);
y_center = interpolant('y_center','bspline',{d_data},y_data);
dx_ds    = interpolant('dx_ds','bspline',{d_data},dx_data);
dy_ds    = interpolant('dy_ds','bspline',{d_data},dy_data);

%--- Decision variables
U      = MX.sym('U', 2, N);       % [steering; acceleration]
X      = MX.sym('X', 4, N+1);     % states [x; y; psi; v]
S      = MX.sym('S', 1, N+1);     % arc-length
E_coll = MX.sym('E_coll', 1, N+1);% collision slack per step
opt_vars = vertcat(reshape(U,2*N,1), reshape(X,4*(N+1),1), S', E_coll');

%--- Initialize constraints, objective, and bounds arrays
g    = []; lbg = []; ubg = [];
J    = 0;

%--- Initial conditions + first slack
% state equality
g   = [g; X(:,1)-x0];          lbg = [lbg; zeros(4,1)]; ubg = [ubg; zeros(4,1)];
% arc equality
g   = [g; S(1)-s0];            lbg = [lbg; 0];           ubg = [ubg; 0];
% slack non-neg
g   = [g; E_coll(1)];          lbg = [lbg; 0];           ubg = [ubg; Inf];
J   = w_c * E_coll(1)^2;

%--- Parameters/limits
R_safe    = 2.0;               % minimum separation to opponent [m]
delta_max = pi/6;              % max steering [rad]
a_max     = 3.0;               % max acceleration [m/s^2]
v_min     = 0; v_max = 20;

%--- Build horizon
for k = 1:N
    xk = X(:,k); uk = U(:,k); sk = S(k);
    % RK4 integration
    k1 = kin(xk, uk, L);
    k2 = kin(xk+Ts/2*k1, uk, L);
    k3 = kin(xk+Ts/2*k2, uk, L);
    k4 = kin(xk+Ts*k3,   uk, L);
    x_next = xk + Ts/6*(k1 + 2*k2 + 2*k3 + k4);
    g = [g; X(:,k+1)-x_next];    lbg = [lbg; zeros(4,1)]; ubg = [ubg; zeros(4,1)];
    % Frenet reference
    xc      = x_center(sk);  yc  = y_center(sk);
    dxr     = dx_ds(sk);     dyr = dy_ds(sk);
    psi_ref = atan2(dyr,dxr);
    % lateral deviation
    d_err   = -(xk(1)-xc)*sin(psi_ref) + (xk(2)-yc)*cos(psi_ref);
    J = J + w_d*d_err^2 + w_u*(uk(1)^2 + uk(2)^2);
    % track-width constraint: |d_err| ≤ track_width/2
    g = [g;  d_err - track_width/2]; lbg=[lbg;-Inf]; ubg=[ubg;0];
    g = [g; -d_err - track_width/2]; lbg=[lbg;-Inf]; ubg=[ubg;0];
    % update arc-length
    s_next = sk + xk(4)*cos(xk(3)-psi_ref)*Ts;
    g = [g; S(k+1)-s_next]; lbg=[lbg;0]; ubg=[ubg;0];
    % collision-soft constraint
    xopp   = X_opp(1:2,k);
    dist2  = (xk(1)-xopp(1))^2 + (xk(2)-xopp(2))^2;
    g = [g; dist2 + E_coll(k+1) - R_safe^2]; lbg=[lbg;0]; ubg=[ubg;Inf];
    g = [g; E_coll(k+1)];               lbg=[lbg;0]; ubg=[ubg;Inf];
    J = J + w_c * E_coll(k+1)^2;
end

%--- Zero-sum terminal cost: lead over opponent
J = J + w_s*(S_opp(end) - S(end));

%--- Variable bounds
lb_u = repmat([-delta_max; -a_max], N,1);
ub_u = repmat([ delta_max;  a_max], N,1);
lb_x = repmat([-Inf; -Inf; -Inf; v_min], N+1,1);
ub_x = repmat([ Inf;  Inf;  Inf; v_max], N+1,1);
lb_s = repmat(d_data(1), N+1,1);
ub_s = repmat(d_data(end), N+1,1);
lb_e = zeros(N+1,1);
ub_e = inf(N+1,1);

lbx = [lb_u; lb_x; lb_s; lb_e];
ubx = [ub_u; ub_x; ub_s; ub_e];

%--- Solve NLP
nlp = struct('x',opt_vars,'f',J,'g',vertcat(g));
opts = struct('ipopt',struct('print_level',0));
solver = nlpsol('solver','ipopt',nlp,opts);

dec0 = [reshape(U_init,2*N,1); repmat(x0, N+1,1); repmat(s0, N+1,1); zeros(N+1,1)];
res = solver('x0',dec0,'lbx',lbx,'ubx',ubx,'lbg',lbg,'ubg',ubg);
sol = full(res.x);

U_opt   = reshape(sol(1:2*N),2,N);
X_opt   = reshape(sol(2*N+1:2*N+4*(N+1)),4,N+1);
idx     = 2*N + 4*(N+1);
S_opt   = sol(idx+1:idx+N+1);
E_coll  = sol(idx+N+2:end);
end

function dx = kin(x,u,L)
% Vehicle kinematic derivatives
dx = [ x(4)*cos(x(3));
       x(4)*sin(x(3));
       x(4)/L * tan(u(1));
       u(2) ];
end
