function [s_proj, d] = cart2Frenet(x, y, ppX, ppY, s_vals)
% CART2FRENET  Convert Cartesian coordinates to Frenet (s,d) coordinates
%   [s_proj, d] = cart2Frenet(x, y, ppX, ppY, s_vals)
%   Inputs:
%     x, y     : scalars or vectors of query point(s)
%     ppX, ppY : piecewise-polynomial structs for centerline x(s), y(s)
%     s_vals   : original knot points of the spline (vector)
%   Outputs:
%     s_proj   : projected arc-length along centerline
%     d        : signed lateral offset from centerline

% Ensure column vectors
xq = x(:);
yq = y(:);
Nq = numel(xq);

% Prepare outputs
d = zeros(Nq,1);
s_proj = zeros(Nq,1);

% Sample centerline densely for initial guess
s_min = s_vals(1);
s_max = s_vals(end);
s_dense = linspace(s_min, s_max, 1000);
x_dense = ppval(ppX, s_dense);
y_dense = ppval(ppY, s_dense);

tol = 1e-3;  % tolerance for fminsearch

% Loop over query points
for i = 1:Nq
    % Find nearest dense sample as initial guess
    dxr = x_dense - xq(i);
    dyr = y_dense - yq(i);
    [~, idx0] = min(dxr.^2 + dyr.^2);

    % Objective: squared distance between track point and query
    fun = @(sp) (ppval(ppX,sp)-xq(i)).^2 + (ppval(ppY,sp)-yq(i)).^2;
    opts = optimset('TolX', tol);
    s_opt = fminsearch(fun, s_dense(idx0), opts);

    % Clamp to valid range
    s_proj(i) = min(max(s_opt, s_min), s_max);

    % Compute tangent derivative at s_proj
    dppX = fnder(ppX);
    dppY = fnder(ppY);
    dx_ds = ppval(dppX, s_proj(i));
    dy_ds = ppval(dppY, s_proj(i));
    t = [dx_ds; dy_ds];
    t = t / norm(t);

    % Normal vector (to the left of tangent)
    nvec = [-t(2); t(1)];

    % Lateral offset (signed)
    xt = ppval(ppX, s_proj(i));
    yt = ppval(ppY, s_proj(i));
    vec = [xq(i)-xt; yq(i)-yt];
    d(i) = dot(vec, nvec);
end
end
