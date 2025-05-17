%% test_generateFrenetSpline.m
clc; close all; clear;
% Test script for generateFrenetSpline
% Define a simple set of waypoints (e.g., an S-curve)
centerline = [0, 0;
              10, 0;
              20, 10;
              30, 5;
              40, 0;
              50, 0];

% Call the spline generator
[s_vals, ppX, ppY] = generateFrenetSpline(centerline);

% Generate dense samples along arc-length
s_dense = linspace(0, s_vals(end), 200);
x_dense = ppval(ppX, s_dense);
y_dense = ppval(ppY, s_dense);

% Plot original waypoints and the fitted spline
figure; hold on; grid on;
plot(centerline(:,1), centerline(:,2), 'ro', 'MarkerFaceColor','r');
plot(x_dense, y_dense, 'b-', 'LineWidth', 1.5);
title('Test of generateFrenetSpline');
legend('Centerline Waypoints', 'Fitted Cubic Spline', 'Location','Best');
xlabel('X [m]'); ylabel('Y [m]');
axis equal;

% Display end-of-test message
disp('generateFrenetSpline test completed successfully.');
