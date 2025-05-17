function [s_vals, ppX, ppY] = generateFrenetSpline(centerline)
%GENERATEFRENETSPLINE  Fit a spline to track centerline and parameterize by arc length
%   [s_vals, ppX, ppY] = generateFrenetSpline(centerline)
%   Inputs:
%     centerline : Nx2 matrix of [x, y] waypoint coordinates
%   Outputs:
%     s_vals : Nx1 vector of cumulative arc-length values
%     ppX    : piecewise polynomial (pp) for x(s)
%     ppY    : piecewise polynomial (pp) for y(s)

% Number of points
N = size(centerline,1);

% Extract coordinates
x = centerline(:,1);
y = centerline(:,2);

% Compute incremental distances between waypoints
dx = diff(x);
dy = diff(y);

dists = sqrt(dx.^2 + dy.^2);

% Cumulative arc-length s
s_vals = [0; cumsum(dists)];

% Check for duplicate s (should not happen if track is well-defined)
if any(diff(s_vals)==0)
    warning('Zero-length segment detected in centerline waypoints.');
end

% Fit cubic splines: ppx(s) and ppy(s)
ppX = spline(s_vals, x');   % note: spline takes row vectors for values
ppY = spline(s_vals, y');

% Optionally, you can increase smoothness by using spaps (smoothing spline)
% tol = 1e-3;
% ppX = spaps(s_vals, x', tol);
% ppY = spaps(s_vals, y', tol);

end
