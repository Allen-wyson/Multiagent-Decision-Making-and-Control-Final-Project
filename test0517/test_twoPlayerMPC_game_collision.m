%% test_twoPlayerMPC_game_collision.m
clc; clear; close all;
% Two-player zero-sum MPC with soft collision & track-width constraints

% build a simple straight track for demo
Xw = linspace(0,50,100)';  
centerline = [Xw, zeros(size(Xw))];
[s_vals, ppX, ppY] = generateFrenetSpline(centerline);

% simulation params
Ts      = 0.1;
T_horiz = 2.0;   N = round(T_horiz/Ts);
L       = 2.5;

% cost weights (tune these!)
w_s = 10.0;   % overtake reward
w_d = 1.0;    % lateral error
w_u = 0.1;    % control effort
w_c = 1.0;    % collision slack

track_width = 5.0;  % total track half-width

% initialize Car1 & Car2
x1 = [0; 0; 0; 5];        s1 = 0;
x2 = [0; -2; 0; 5];       s2 = 0;   % offset in d

steps = 100;
U1 = zeros(2,N);  U2 = zeros(2,N);
X1_hist = zeros(4,steps+1);  X2_hist = X1_hist;
X1_hist(:,1)=x1;  X2_hist(:,1)=x2;
S1_hist = zeros(1,steps+1);  S2_hist=S1_hist;
S1_hist(1)=s1;    S2_hist(1)=s2;

for k = 1:steps
  % --- best-response iterations
  for it = 1:3
    % Car 1 plans vs. Car 2’s last trajectory
    [U1, X1_pred, S1_pred, ~] = solveMPC_game_collision( ...
      x1, U1, s1, s_vals, ppX, ppY, Ts, N, ...
      w_s, w_d, w_u, w_c, L, S2_hist(max(1,k):(max(1,k)+N)), ...
      X2_hist(:,max(1,k):(max(1,k)+N)), track_width);
    % Car 2 plans vs. Car 1’s last trajectory
    [U2, X2_pred, S2_pred, ~] = solveMPC_game_collision( ...
      x2, U2, s2, s_vals, ppX, ppY, Ts, N, ...
      w_s, w_d, w_u, w_c, L, S1_hist(max(1,k):(max(1,k)+N)), ...
      X1_hist(:,max(1,k):(max(1,k)+N)), track_width);
  end

  % --- apply first controls
  x1 = vehicleDynamics(x1, U1(:,1), Ts, L);
  x2 = vehicleDynamics(x2, U2(:,1), Ts, L);

  % --- update arcs
  s1 = min(max(s1 + x1(4)*Ts, s_vals(1)), s_vals(end));
  s2 = min(max(s2 + x2(4)*Ts, s_vals(1)), s_vals(end));

  % --- shift warm-start sequences
  U1 = [U1(:,2:end), U1(:,end)];
  U2 = [U2(:,2:end), U2(:,end)];

  % --- record history
  X1_hist(:,k+1)=x1;  X2_hist(:,k+1)=x2;
  S1_hist(k+1)=s1;    S2_hist(k+1)=s2;
end

% plot results
figure; hold on; grid on;
plot(centerline(:,1),centerline(:,2),'k--','LineWidth',1);
plot(X1_hist(1,:), X1_hist(2,:),'b-o','DisplayName','Car1');
plot(X2_hist(1,:), X2_hist(2,:),'r-s','DisplayName','Car2');
title('Two-Player Zero-Sum MPC with Collision & Track Constraints');
legend('Track','Car1','Car2','Location','Best');
axis equal;
