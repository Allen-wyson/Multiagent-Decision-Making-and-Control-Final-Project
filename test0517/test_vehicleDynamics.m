%% test_vehicleDynamics.m
clc; close all; clear;
% Test script for vehicleDynamics using RK4 integration
% 1) Straight-line test: zero steering, constant speed
Ts = 0.01;   % time step [s]
L  = 2.5;     % wheelbase [m]
x0 = [0; 0; 0; 5];  % initial state: x=0,y=0,heading=0,v=5 m/s
u  = [0; 0];       % no steering, no acceleration
N  = 100;
X = zeros(4, N+1);
X(:,1) = x0;
for k = 1:N
    X(:,k+1) = vehicleDynamics(X(:,k), u, Ts, L);
end
% Check final position: should be x = v*Ts*N, y = 0
x_final = X(1,end);
y_final = X(2,end);
expected_x = x0(4)*Ts*N;
assert(abs(x_final-expected_x)<1e-3, 'Straight-line x error too large');
assert(abs(y_final)<1e-6, 'Straight-line y error too large');
disp('Straight-line test passed.');

% 2) Constant steering test: verify heading change matches theory
delta = deg2rad(10);    % 10° steering
u = [delta; 0];         % constant steering
X = zeros(4, N+1);
X(:,1) = x0;
for k = 1:N
    X(:,k+1) = vehicleDynamics(X(:,k), u, Ts, L);
end
% Theoretical heading change: dpsi_dt = v/L*tan(delta)
dpsi_dt = x0(4)/L * tan(delta);
expected_dpsi = dpsi_dt * Ts * N;
psi_final = X(3,end);
assert(abs(psi_final - expected_dpsi) < 1e-3, 'Heading change error too large');
disp('Constant-steering heading test passed.');

% 3) Optional: plot trajectory for visual inspection
figure; plot(X(1,:), X(2,:), 'b-', 'LineWidth',1.5);
axis equal; grid on;
title('Vehicle Dynamics RK4: Constant Steering Trajectory');
xlabel('X [m]'); ylabel('Y [m]');
