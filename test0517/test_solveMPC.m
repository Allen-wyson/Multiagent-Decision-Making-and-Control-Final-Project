%% test_solveMPC.m
clc; close all; clear;
% Test script for solveMPC: one-car progression on a straight track using CasADi + Frenet MPC

% 1) Define a simple straight track and build Frenet spline
% Generate a dense straight track (>8 points) for robust B-spline interpolation
Xw = linspace(0,100,20)';
centerline = [Xw, zeros(size(Xw))];
[s_vals, ppX, ppY] = generateFrenetSpline(centerline);

% 2) MPC parameters
Ts        = 0.1;            % time step [s]
T_horizon = 2.0;            % horizon [s]
N         = round(T_horizon/Ts);
L         = 2.5;            % wheelbase [m]

% 3) Initial state at start of track
x0 = [0; 0; 0; 5];            % [x; y; psi; v]

% 4) Cost weights
w_s = 1.0;                    % weight on terminal progress (s)
w_d = 1.0;                    % weight on lateral deviation (d)
w_u = 0.1;                    % weight on control effort

% 5) Initial arc-length
s0 = 0;
s0 = 0;

% 5) Initial control guess
U_init = zeros(2, N);

% 6) Run CasADi-based MPC
fprintf('Running CasADi MPC on straight track... ');
tic;
[U_opt, X_opt] = solveMPC(x0, U_init, s0, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L);
toc;
disp('Done.');

% 7) Basic assertion: forward progression
assert(X_opt(1,end) > x0(1) + 1e-3, 'No forward progress detected.');
disp('solveMPC progression test passed.');

% 8) Plot result
figure; hold on; grid on;
plot(X_opt(1,:), X_opt(2,:), 'b-o', 'LineWidth',1.5);
plot(centerline(:,1), centerline(:,2), 'k--', 'LineWidth',1);
title('MPC Trajectory vs. Track');
xlabel('X [m]'); ylabel('Y [m]');
legend('MPC Path','Track Centerline','Location','Best');
axis equal;
