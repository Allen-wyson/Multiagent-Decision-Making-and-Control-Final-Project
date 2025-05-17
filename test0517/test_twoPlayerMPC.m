%% test_twoPlayerMPC.m
clc; clear; close all;
% Two-car receding-horizon MPC with best-response iterations on a curved track

% 1) Build track and Frenet splines
theta = linspace(0,2*pi,100)';  % simple circular loop
R = 50;
centerline = [R*cos(theta), R*sin(theta)];
[s_vals, ppX, ppY] = generateFrenetSpline(centerline);

% 2) MPC parameters
Ts      = 0.1;
T_horiz = 2.0;
N       = round(T_horiz/Ts);
L       = 2.5;
w_s     = 1e3;   % slack penalty weight (large to discourage constraint violation)
w_d     = 5.0;
w_u     = 0.1;
maxIter = 3;      % best-response steps per time

d_min   = 2.0;    % minimum safety distance

% 3) Initialize both cars
% Car1 starts at angle 0, Car2 at pi/2
a1 = 0; a2 = pi/2;
x1 = [R*cos(a1); R*sin(a1); a1+pi/2; 5];
x2 = [R*cos(a2); R*sin(a2); a2+pi/2; 5];

% initial arc positions
s1_pos = 0;              % start at beginning of track
s2_pos = s_vals(end)/4;  % a quarter-loop ahead

% 4) Pre-allocate history and warm-start controls/slacks
steps    = round(10/Ts);
X1_hist  = zeros(4, steps+1);
X2_hist  = zeros(4, steps+1);
X1_hist(:,1) = x1;
X2_hist(:,1) = x2;
U1 = zeros(2,N);
U2 = zeros(2,N);

% warm-start slacks for collision constraints
s1_slack = zeros(1,N);
s2_slack = zeros(1,N);

for k = 1:steps
    % best-response iterations with fallback on solver failures
    for it = 1:maxIter
        % Car 1
        try
            [solverOut, U1, X1, s1_slack] = evalc(...
                'solveMPC(x1, U1, s1_slack, s2_slack, s1_pos, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L, d_min);');
            fprintf('%s', solverOut);
        catch ME
            warning('Car1 MPC failed (%s). Resetting warm start.', ME.message);
            U1 = zeros(size(U1));
            s1_slack = zeros(1,N);
            [solverOut, U1, X1, s1_slack] = evalc(...
                'solveMPC(x1, U1, s1_slack, s2_slack, s1_pos, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L, d_min);');
            fprintf('%s', solverOut);
        end
        % Car 2
        try
            [solverOut, U2, X2, s2_slack] = evalc(...
                'solveMPC(x2, U2, s2_slack, s1_slack, s2_pos, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L, d_min);');
            fprintf('%s', solverOut);
        catch ME
            warning('Car2 MPC failed (%s). Resetting warm start.', ME.message);
            U2 = zeros(size(U2));
            s2_slack = zeros(1,N);
            [solverOut, U2, X2, s2_slack] = evalc(...
                'solveMPC(x2, U2, s2_slack, s1_slack, s2_pos, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L, d_min);');
            fprintf('%s', solverOut);
        end
    end
    % apply first control input
    x1 = vehicleDynamics(x1, U1(:,1), Ts, L);
    x2 = vehicleDynamics(x2, U2(:,1), Ts, L);
    % update and clamp arc positions
    s1_pos = min(max(s1_pos + x1(4)*Ts, s_vals(1)), s_vals(end));
    s2_pos = min(max(s2_pos + x2(4)*Ts, s_vals(1)), s_vals(end));
    % shift warm-start for next step
    U1 = [U1(:,2:end), U1(:,end)];
    U2 = [U2(:,2:end), U2(:,end)];
    s1_slack = [s1_slack(2:end), s1_slack(end)];
    s2_slack = [s2_slack(2:end), s2_slack(end)];

    X1_hist(:,k+1) = x1;
    X2_hist(:,k+1) = x2;
end

% 5) Plot race trajectories
t = (0:steps)*Ts;
figure; hold on; grid on;
plot(X1_hist(1,:), X1_hist(2,:), 'b', 'LineWidth',1.5);
plot(X2_hist(1,:), X2_hist(2,:), 'r', 'LineWidth',1.5);
plot(centerline(:,1), centerline(:,2), 'k--', 'LineWidth',1);
legend('Car 1','Car 2','Track');
title('Two-Player MPC Racing');
axis equal;
