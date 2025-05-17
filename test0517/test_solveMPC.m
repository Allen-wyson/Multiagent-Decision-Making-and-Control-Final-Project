clc; close all; clear;
% Test one‐car on straight track

Xw = linspace(0,100,20)';  
centerline = [Xw, zeros(size(Xw))];
[s_vals, ppX, ppY] = generateFrenetSpline(centerline);

Ts        = 0.1;  T_horizon = 2.0;  N = round(T_horizon/Ts);
L         = 2.5;
x0        = [0;0;0;5];
w_s       = 1.0;  w_d = 1.0;  w_u = 0.1;
s0        = 0;
U_init    = zeros(2,N);

fprintf('Running CasADi MPC on straight track... ');
tic;
[U_opt, X_opt, ~] = solveMPC(x0, U_init, s0, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L);
toc;
disp('Done.');

assert(X_opt(1,end) > x0(1) + 1e-3, 'No forward progress.');
disp('solveMPC progression test passed.');

figure; hold on; grid on;
plot(X_opt(1,:),X_opt(2,:),'b-o','LineWidth',1.5);
plot(centerline(:,1),centerline(:,2),'k--','LineWidth',1);
title('MPC Trajectory vs. Track');
xlabel('X [m]'); ylabel('Y [m]');
legend('MPC Path','Track','Location','Best');
axis equal;
