%% test_solveMPC_game_collision.m
clc; clear; close all;

% 1) Build a straight track
Xw = linspace(0,50,20)';
centerline = [Xw, zeros(size(Xw))];
[s_vals, ppX, ppY] = generateFrenetSpline(centerline);

% 2) Common MPC params
Ts    = 0.1;  T_h = 2.0;  N = round(T_h/Ts);
L     = 2.5;
w_s   = 50.0;  w_d = 1.0;  w_u = 0.1;  w_c = 100.0;  % heavy collision penalty

% 3) Initial states & guesses
x1  = [0;  0; 0; 5];  s1 = 0;  U01 = zeros(2,N);
x2  = [0; -2; 0; 5];  s2 = 0;  U02 = zeros(2,N);

% 4) First plan Car 2 ignoring Car 1
%    Here we “fake” Car1 as stationary at x1, so:
S_opp_for2 = repmat(s1,1,N+1);              % constant arc
X_opp_for2 = repmat(x1(1:2),1,N+1);         % constant XY
[U2, X2, S2] = solveMPC_game_collision( ...
   x2, U02, s2, s_vals, ppX, ppY, Ts, N, ...
   w_s, w_d, w_u, w_c, L, ...
   S_opp_for2, X_opp_for2 );

% 5) Now plan Car 1 against Car 2’s fixed trajectory
[U1, X1, S1, E] = solveMPC_game_collision( ...
   x1, U01, s1, s_vals, ppX, ppY, Ts, N, ...
   w_s, w_d, w_u, w_c, L, ...
   S2, X2 );

% 6) Plot
figure; hold on; grid on;
plot(centerline(:,1),centerline(:,2),'k--','LineWidth',1);
plot(X2(1,:),X2(2,:),'r-o','LineWidth',1.5);
plot(X1(1,:),X1(2,:),'b-o','LineWidth',1.5);
legend('Track','Car 2 (fixed)','Car 1 (collision‐aware)','Location','Best');
axis equal; title('Zero‐Sum MPC with Soft Collision Avoidance');

% 7) Display max slack used
fprintf('Max collision slack = %g m\n', max(E));
