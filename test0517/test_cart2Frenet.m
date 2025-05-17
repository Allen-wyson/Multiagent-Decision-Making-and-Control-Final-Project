%% test_cart2Frenet.m
clc; close all; clear;
% Test script for cart2Frenet
% 1) Build a simple track and its Frenet spline
centerline = [0, 0;
              10, 0;
              20, 5;
              30, 5;
              40, 0;
              50, 0];
[s_vals, ppX, ppY] = generateFrenetSpline(centerline);

% 2) Choose sample s locations and compute true (x,y) on centerline
s_test = linspace(0, s_vals(end), 5);
xy_true = [ppval(ppX, s_test); ppval(ppY, s_test)]';

% 3) Create query points by offsetting laterally by known d values
d_offsets = [-2, -1, 0, 1, 2];
num_pts = numel(s_test) * numel(d_offsets);
xy_query = zeros(num_pts,2);
s_expected = zeros(num_pts,1);
d_expected = zeros(num_pts,1);
idx = 1;
for i = 1:numel(s_test)
    % get tangent vector
    ds = 1e-3;
    dx_ds = (ppval(ppX, s_test(i)+ds) - ppval(ppX, s_test(i)-ds)) / (2*ds);
    dy_ds = (ppval(ppY, s_test(i)+ds) - ppval(ppY, s_test(i)-ds)) / (2*ds);
    t = [dx_ds; dy_ds]; t = t / norm(t);
    n = [-t(2); t(1)];  % outward normal
    for j = 1:numel(d_offsets)
        xy_query(idx,:) = xy_true(i,:) + d_offsets(j)*n';
        s_expected(idx) = s_test(i);
        d_expected(idx) = d_offsets(j);
        idx = idx + 1;
    end
end

% 4) Run cart2Frenet on all query points
[s_proj, d_proj] = cart2Frenet(xy_query(:,1), xy_query(:,2), ppX, ppY, s_vals);

% 5) Display results
TOL = 1e-2;
disp('s_expected vs. s_proj:'); disp([s_expected, s_proj]);
disp('d_expected vs. d_proj:'); disp([d_expected, d_proj]);

% 6) Check errors
s_err = abs(s_proj - s_expected);
d_err = abs(d_proj - d_expected);
assert(all(s_err < TOL), 's projection error exceeds tolerance');
assert(all(d_err < TOL), 'lateral offset error exceeds tolerance');

disp('cart2Frenet test passed successfully.');

% 7) Optional: plot for visual verification
figure; hold on; grid on;
plot(ppval(ppX, s_vals), ppval(ppY, s_vals), 'k-', 'LineWidth',1.5);
scatter(xy_query(:,1), xy_query(:,2), 50, d_proj, 'filled');
colorbar; title('Query Points Colored by Projected d');
xlabel('X [m]'); ylabel('Y [m]'); axis equal;
