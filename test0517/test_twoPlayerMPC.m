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
w_s     = 1.0; w_d = 5.0; w_u = 0.1;
maxIter = 3;  % best-response steps per time

% 3) Initialize both cars
a1 = 0; a2 = pi/2;
x1 = [R*cos(a1); R*sin(a1); a1+pi/2; 5];
x2 = [R*cos(a2); R*sin(a2); a2+pi/2; 5];
s1 = 0;                    % start at beginning of track
s2 = s_vals(end)/4;        % a quarter-loop ahead

% 4) Pre-allocate history and warm-start controls
steps    = round(10/Ts);
X1_hist  = zeros(4, steps+1);
X2_hist  = zeros(4, steps+1);
X1_hist(:,1) = x1;
X2_hist(:,1) = x2;
U1 = zeros(2,N);
U2 = zeros(2,N);

for k = 1:steps
    % best-response iterations with fallback on solver failures
    for it = 1:maxIter
        % Car 1
        try
            [solverOut, U1, X1, E1] = evalc( ...
                'solveMPC(x1, U1, s1, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L);' );
            fprintf('%s', solverOut);
        catch ME
            warning('Car1 MPC failed (%s). Resetting warm start.', ME.message);
            U1 = zeros(size(U1));
            [solverOut, U1, X1, E1] = evalc( ...
                'solveMPC(x1, U1, s1, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L);' );
            fprintf('%s', solverOut);
        end
        % report any slack violation
        if max(E1) > 1e-6
            fprintf('  [Car1 slack max = %.3e]\n', max(E1));
        end

        % Car 2
        try
            [solverOut, U2, X2, E2] = evalc( ...
                'solveMPC(x2, U2, s2, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L);' );
            fprintf('%s', solverOut);
        catch ME
            warning('Car2 MPC failed (%s). Resetting warm start.', ME.message);
            U2 = zeros(size(U2));
            [solverOut, U2, X2, E2] = evalc( ...
                'solveMPC(x2, U2, s2, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L);' );
            fprintf('%s', solverOut);
        end
        if max(E2) > 1e-6
            fprintf('  [Car2 slack max = %.3e]\n', max(E2));
        end
    end

    % apply first control input
    x1 = vehicleDynamics(x1, U1(:,1), Ts, L);
    x2 = vehicleDynamics(x2, U2(:,1), Ts, L);

    % update and clamp arc positions
    s1 = min(max(s1 + x1(4)*Ts, s_vals(1)), s_vals(end));
    s2 = min(max(s2 + x2(4)*Ts, s_vals(1)), s_vals(end));

    % shift warm-start for next step
    U1 = [U1(:,2:end), U1(:,end)];
    U2 = [U2(:,2:end), U2(:,end)];

    X1_hist(:,k+1) = x1;
    X2_hist(:,k+1) = x2;
end

% 5) Plot race trajectories
t = (0:steps)*Ts;
figure;
subplot(2,1,1);
hold on; grid on;
plot(X1_hist(1,:), X1_hist(2,:), 'b', 'LineWidth',1.5);
plot(X2_hist(1,:), X2_hist(2,:), 'r', 'LineWidth',1.5);
plot(centerline(:,1), centerline(:,2), 'k--', 'LineWidth',1);
legend('Car 1','Car 2','Track','Location','Best');
title('Two-Player MPC Racing Trajectories');
axis equal;

% 6) Plot slacks over time (max per step)
maxSlack1 = zeros(1,steps);
maxSlack2 = zeros(1,steps);
for k = 1:steps
    % you could store E1/E2 each iteration if you like; here we just plot last iter
    maxSlack1(k) = max(E1);
    maxSlack2(k) = max(E2);
end
subplot(2,1,2);
plot(t(1:end-1), maxSlack1, 'b-', t(1:end-1), maxSlack2, 'r-','LineWidth',1.2);
grid on;
xlabel('Time [s]'); ylabel('Max Slack');
legend('Car 1','Car 2','Location','Best');
title('Max Slack Violations Over Time');
