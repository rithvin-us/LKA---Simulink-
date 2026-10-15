function road = lka_make_lane_road(ref, laneOffset, laneWidth, lineWidth, reverse, name)
% LKA_MAKE_LANE_ROAD Builds the ego-lane description used by the test bench.
%
% Syntax:
%   road = lka_make_lane_road(ref, laneOffset, laneWidth, lineWidth, reverse, name)
%
% Inputs:
%   ref        - reference line from lka_road_from_geometry (s, x, y, hdg, kappa)
%   laneOffset - lateral offset of the ego-lane centre from the reference line,
%                + to the left of the reference direction [m] (scalar or per sample)
%   laneWidth  - ego-lane width [m] (scalar or per sample)
%   lineWidth  - painted line width [m]
%   reverse    - true if the ego lane is driven against the reference direction
%   name       - text label
%
% Output: road struct (column vectors, all along the ego-lane centre in the
% driving direction):
%   s, x, y, psi, kappa, dkappa, width  lane centre and lane width
%   xL, yL, xR, yR                       left / right boundary (line centres)
%   lineWidth, name, length

    n = numel(ref.s);
    d = laneOffset .* ones(n, 1);
    w = laneWidth .* ones(n, 1);

    nx = -sin(ref.hdg);  ny = cos(ref.hdg);           % left normal of the reference line
    xc = ref.x + d .* nx;
    yc = ref.y + d .* ny;
    kc = ref.kappa ./ max(1 - ref.kappa .* d, 0.05);  % parallel-curve curvature
    psi = ref.hdg;

    xLb = xc + 0.5 * w .* nx;  yLb = yc + 0.5 * w .* ny;
    xRb = xc - 0.5 * w .* nx;  yRb = yc - 0.5 * w .* ny;

    if reverse
        xc = flipud(xc); yc = flipud(yc); w = flipud(w);
        psi = flipud(psi) + pi;
        kc = -flipud(kc);
        [xLb, xRb] = deal(flipud(xRb), flipud(xLb));
        [yLb, yRb] = deal(flipud(yRb), flipud(yLb));
    end

    seg = hypot(diff(xc), diff(yc));
    sc = [0; cumsum(seg)];
    psi = unwrap(psi);

    road = struct();
    road.name = name;
    road.s = sc;
    road.x = xc;
    road.y = yc;
    road.psi = psi;
    road.kappa = kc;
    road.dkappa = gradient(kc, sc);
    road.width = w;
    road.xL = xLb; road.yL = yLb;
    road.xR = xRb; road.yR = yRb;
    road.lineWidth = lineWidth;
    road.length = sc(end);
end
