%% test_solveMPC_curve.m
clc; close all; clear;
% Test script for solveMPC: one-car progression on a curved track using CasADi solver

% 1) Define a curved track for plotting (quarter-circle)
radius = 50;  % [m]
theta = linspace(0,pi/2,20)';
centerline = [radius*cos(theta), radius*sin(theta)];

% 2) Build Frenet spline from centerline
[s_vals, ppX, ppY] = generateFrenetSpline(centerline);

% 3) MPC parameters
Ts        = 0.1;               % time step [s]
T_horizon = 3.0;               % horizon [s]
N         = round(T_horizon/Ts);
L         = 2.5;               % wheelbase [m]
w_s       = 1.0;               % weight on terminal progress (s)
w_d       = 20.0;              % weight on lateral deviation (d)
w_u       = 0.1;               % weight on control effort

% 4) Initial state at start of circle (heading tangent to track)
x0_xy = centerline(1,:)';
psi0  = theta(1) + pi/2;       % heading tangent to circle
v0    = 5;                     % [m/s]
x0    = [x0_xy; psi0; v0];

% 5) Initial arc-length
s0 = 0;

% 6) Initial control guess
U_init = zeros(2, N);

% 7) Run CasADi-based MPC
fprintf('Running CasADi MPC on curved track... ');
tic;
[U_opt, X_opt] = solveMPC(x0, U_init, s0, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L);
toc;
disp('Done.');

% 8) Plot results
figure; hold on; grid on;
plot(centerline(:,1), centerline(:,2), 'k--', 'LineWidth',1.5);
plot(X_opt(1,:), X_opt(2,:),        'b-o', 'LineWidth',1.5);
xlabel('X [m]'); ylabel('Y [m]');
axis equal;
title('MPC Path on a Quarter-Circle Track (CasADi + Frenet)');
legend('Track Centerline','MPC Path','Location','Best');
