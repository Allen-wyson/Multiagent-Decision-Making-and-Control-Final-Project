%% racing_simulation.m
% Game-theoretic two-car racing simulation using receding-horizon MPC
%  Author: (your name)
%  Date: (today's date)

%% 1. Clean workspace and add necessary paths
clear; close all; clc;
addpath(genpath('utils'));    % your helper functions
% addpath(genpath('casadi'));   % if you use CasADi for optimization
import casadi.*

%% 2. Simulation parameters
Ts      = 0.1;      % simulation timestep [s]
T_horizon = 2.0;    % MPC horizon length [s]
n_steps = round(T_horizon/Ts);
maxIter = 5;        % best-response iterations per step
Tf      = 20;       % final simulation time [s]
Nsim    = round(Tf/Ts);

%% 3. Vehicle model parameters (bicycle model)
L       = 2.5;      % wheelbase [m]
v0      = 5.0;      % initial speed [m/s]

%% 4. Cost weights
w_s     = 1.0;      % weight for forward progress
w_d     = 10.0;     % weight for lateral error
w_u     = 0.1;      % weight on control effort

%% 5. Track definition: centerline waypoints
centerline = load('track_centerline.mat');   % [Nx2] matrix of (x,y)
% Generate Frenet representation
[spline_s, spline_x, spline_y] = generateFrenetSpline(centerline);

%% 6. Initial states for Car 1 and Car 2
% State vector: [x; y; psi; v]
x1 = [0;  0;  0;  v0];
x2 = [ -2; -1; 0;  v0];   % offset behind and aside

% Pre-allocate state histories
X1 = zeros(4, Nsim+1); X1(:,1) = x1;
X2 = zeros(4, Nsim+1); X2(:,1) = x2;

%% 7. Main simulation loop
for k = 1:Nsim
    t = (k-1)*Ts;

    % 7.1 Initialize trajectory guesses (straight-line)
    U1_guess = repmat([0; 0], 1, n_steps);  % [delta; a]
    U2_guess = repmat([0; 0], 1, n_steps);
    X1_guess = repmat(x1, 1, n_steps+1);
    X2_guess = repmat(x2, 1, n_steps+1);

    % 7.2 Best-response iterations
    for iter = 1:maxIter
        % Car 1 solves MPC given Car 2 guess
        [U1_guess, X1_guess] = solveMPC(x1, X2_guess, U1_guess, spline_s, spline_x, spline_y, Ts, n_steps, w_s, w_d, w_u, L);
        % Car 2 solves MPC given Car 1 updated guess
        [U2_guess, X2_guess] = solveMPC(x2, X1_guess, U2_guess, spline_s, spline_x, spline_y, Ts, n_steps, w_s, w_d, w_u, L);
    end

    % 7.3 Apply first control input
    u1 = U1_guess(:,1);
    u2 = U2_guess(:,1);
    x1 = vehicleDynamics(x1, u1, Ts, L);
    x2 = vehicleDynamics(x2, u2, Ts, L);

    % 7.4 Log states
    X1(:,k+1) = x1;
    X2(:,k+1) = x2;
end

%% 8. Plot results
time = (0:Nsim)*Ts;
figure; hold on; box on;
plot(X1(1,:), X1(2,:),'b','LineWidth',1.5);
plot(X2(1,:), X2(2,:),'r','LineWidth',1.5);
xlabel('X [m]'); ylabel('Y [m]');
title('Two-Car Racing Trajectories');
legend('Car 1','Car 2');

%% Utility function placeholders
% Implement generateFrenetSpline, solveMPC, vehicleDynamics in utils/
