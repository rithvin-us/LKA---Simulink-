function [centerPose, curv, width, s, idx] = lka_road_closest_point(px, py, idxPrev, rs, rx, ry, rpsi, rk, rdk, rw)
% LKA_ROAD_CLOSEST_POINT Projection of a point onto the sampled lane centreline.
%
% Syntax:
%   [centerPose, curv, width, s, idx] = lka_road_closest_point(px, py, idxPrev, rs, rx, ry, rpsi, rk, rdk, rw)
%
% Inputs:
%   px, py   - query point (world frame) [m]
%   idxPrev  - segment index found in the previous call (0 = unknown -> global search)
%   rs..rw   - road arrays: station, x, y, heading, curvature, dcurvature, width
%
% Outputs:
%   centerPose - [x y psi] of the foot point on the centreline
%   curv       - [kappa dkappa] at the foot point
%   width      - lane width at the foot point [m]
%   s          - station of the foot point [m]
%   idx        - segment index (start sample) of the foot point
%
% Searches a +-window around idxPrev (vehicle moves < window per step), then
% projects onto the best segment with the proper dot-product formula and
% interpolates every quantity linearly. Code-generation compatible.

    n = numel(rs);
    window = 400;
    if idxPrev < 1
        lo = 1; hi = n - 1;
    else
        lo = max(1, idxPrev - window);
        hi = min(n - 1, idxPrev + window);
    end

    best = lo; bestD = inf; bestU = 0;
    for i = lo:hi
        ax = rx(i); ay = ry(i);
        dx = rx(i+1) - ax; dy = ry(i+1) - ay;
        L2 = dx*dx + dy*dy;
        u = 0;
        if L2 > 0
            u = ((px - ax)*dx + (py - ay)*dy) / L2;
            u = min(max(u, 0), 1);
        end
        ex = ax + u*dx - px; ey = ay + u*dy - py;
        d2 = ex*ex + ey*ey;
        if d2 < bestD
            bestD = d2; best = i; bestU = u;
        end
    end

    i = best; u = bestU;
    centerPose = [rx(i) + u*(rx(i+1) - rx(i)), ...
                  ry(i) + u*(ry(i+1) - ry(i)), ...
                  rpsi(i) + u*(rpsi(i+1) - rpsi(i))];
    curv  = [rk(i) + u*(rk(i+1) - rk(i)), rdk(i) + u*(rdk(i+1) - rdk(i))];
    width = rw(i) + u*(rw(i+1) - rw(i));
    s     = rs(i) + u*(rs(i+1) - rs(i));
    idx   = i;
end
