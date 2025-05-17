function [U_opt, X_opt, E_opt] = solveMPC(x0, U_init, s0, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L)
%SOLVEMPC  Receding-horizon MPC with “arc‐update” slack
%  [U_opt,X_opt,E_opt] = solveMPC(...)

import casadi.*

% slack penalty weight
w_e = 1e3;

% 1) interpolate track
d_data  = s_vals;
x_data  = ppval(ppX, d_data);
y_data  = ppval(ppY, d_data);
dx_data = ppval(fnder(ppX), d_data);
dy_data = ppval(fnder(ppY), d_data);

x_center = interpolant('x_center','bspline',{d_data},x_data);
y_center = interpolant('y_center','bspline',{d_data},y_data);
dx_ds    = interpolant('dx_ds',   'bspline',{d_data},dx_data);
dy_ds    = interpolant('dy_ds',   'bspline',{d_data},dy_data);

% 2) decision variables
U = MX.sym('U',2,N);
X = MX.sym('X',4,N+1);
S = MX.sym('S',1,N+1);
E = MX.sym('E',1,N);           % slack for each arc‐update

vars = vertcat(reshape(U,2*N,1), reshape(X,4*(N+1),1), S', E');

g = [];     % constraints
J = 0;      % objective

% 3) initial constraints
g = [g; X(:,1)-x0; S(1)-s0];

% 4) horizon
for k = 1:N
    xk = X(:,k);
    uk = U(:,k);
    sk = S(k);

    % a) RK4
    k1 = kin(xk,   uk, L);
    k2 = kin(xk+Ts/2*k1, uk, L);
    k3 = kin(xk+Ts/2*k2, uk, L);
    k4 = kin(xk+Ts*k3,   uk, L);
    x_next = xk + Ts/6*(k1+2*k2+2*k3+k4);
    g = [g; X(:,k+1)-x_next];

    % b) Frenet frame
    xc = x_center(sk);
    yc = y_center(sk);
    psi_ref = atan2( dy_ds(sk), dx_ds(sk) );
    d_err   = -(xk(1)-xc)*sin(psi_ref) + (xk(2)-yc)*cos(psi_ref);

    % running cost
    J = J + w_d*d_err^2 + w_u*(uk(1)^2+uk(2)^2);

    % c) arc‐update with slack
    s_next = sk + xk(4)*cos(xk(3)-psi_ref)*Ts;
    g = [g; S(k+1) - s_next - E(k)];

    % penalize slack
    J = J + w_e*E(k)^2;
end

% 5) terminal reward
J = J - w_s*S(N+1);

% 6) bounds
delta_max = pi/6; a_max = 3; v_min = 0; v_max = 20;
lb_u = repmat([-delta_max; -a_max],N,1);
ub_u = repmat([ delta_max;  a_max],N,1);
lb_x = repmat([-inf; -inf; -inf; v_min],N+1,1);
ub_x = repmat([ inf;  inf;  inf; v_max],N+1,1);
lb_s = repmat(d_data(1),N+1,1);
ub_s = repmat(d_data(end),N+1,1);
lb_e = zeros(N,1);             % slack ≥ 0
ub_e = inf(N,1);

lbx = [lb_u; lb_x; lb_s; lb_e];
ubx = [ub_u; ub_x; ub_s; ub_e];

% all constraints are equalities = 0
lbg = zeros(length(g),1);
ubg = zeros(length(g),1);

% 7) NLP solve
nlp = struct('x',vars,'f',J,'g',vertcat(g));
opts = struct('ipopt',struct('print_level',0));
solver = nlpsol('solver','ipopt',nlp,opts);

dec0 = [reshape(U_init,2*N,1);
        repmat(x0,   N+1,1);
        repmat(s0,   N+1,1);
        zeros(N,1)];

res = solver('x0',dec0,'lbx',lbx,'ubx',ubx,'lbg',lbg,'ubg',ubg);
sol = full(res.x);

% 8) unpack
U_opt = reshape(sol(1:2*N),2,N);
X_opt = reshape(sol(2*N+1:2*N+4*(N+1)),4,N+1);
startE = 2*N+4*(N+1)+(N+1)+1;
E_opt = sol(startE:startE+N-1);

end

function dx = kin(x,u,L)
    dx = [ x(4)*cos(x(3));
           x(4)*sin(x(3));
           x(4)/L*tan(u(1));
           u(2) ];
end
