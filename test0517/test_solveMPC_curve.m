clc; close all; clear;
% Test one‐car on curved (quarter‐circle) track

radius = 50;
theta  = linspace(0,pi/2,20)';
centerline = [radius*cos(theta), radius*sin(theta)];
[s_vals, ppX, ppY] = generateFrenetSpline(centerline);

Ts        = 0.1; T_horizon = 3.0; N = round(T_horizon/Ts);  L = 2.5;
w_s = 1.0;  w_d = 20.0;  w_u = 0.1;

x0_xy = centerline(1,:)';
psi0  = theta(1) + pi/2;
x0    = [x0_xy; psi0; 5];
s0    = 0;
U_init = zeros(2,N);

fprintf('Running CasADi MPC on curved track... ');
tic;
[U_opt, X_opt, ~] = solveMPC(x0, U_init, s0, s_vals, ppX, ppY, Ts, N, w_s, w_d, w_u, L);
toc;
disp('Done.');

figure; hold on; grid on;
plot(centerline(:,1),centerline(:,2),'k--','LineWidth',1.5);
plot(X_opt(1,:),X_opt(2,:),'b-o','LineWidth',1.5);
axis equal;
xlabel('X [m]'); ylabel('Y [m]');
title('MPC Path on a Quarter-Circle Track');
legend('Track','MPC Path','Location','Best');
