function ref = lka_road_from_geometry(geom, ds)
% LKA_ROAD_FROM_GEOMETRY Samples a planar road reference line from primitives.
%
% Syntax:
%   ref = lka_road_from_geometry(geom, ds)
%
% Inputs:
%   geom - struct array, one element per primitive, with fields
%          s0, x0, y0, hdg0  start station, position and heading
%          length            primitive length [m]
%          k0, k1            curvature at start / end (line: 0,0; arc: k,k;
%                            spiral: linear curvature change, OpenDRIVE <spiral>)
%   ds   - sampling step [m]
%
% Output:
%   ref  - struct with column vectors s, x, y, hdg, kappa
%
% Each primitive is integrated from its own start pose (as OpenDRIVE does), so
% rounding errors never accumulate across primitives. Lines and arcs use the
% closed-form solution, spirals a fine trapezoidal integration of the heading.

    s = []; x = []; y = []; hdg = []; kap = [];
    for g = 1:numel(geom)
        G = geom(g);
        n = max(ceil(G.length / ds), 1);
        u = linspace(0, G.length, n + 1)';
        if g < numel(geom)
            u = u(1:end-1);                       % next primitive supplies the shared point
        end
        dk = (G.k1 - G.k0) / max(G.length, eps);
        k = G.k0 + dk * u;
        th = G.hdg0 + G.k0 * u + 0.5 * dk * u.^2;
        if abs(dk) < 1e-12 && abs(G.k0) < 1e-12
            xx = G.x0 + u * cos(G.hdg0);
            yy = G.y0 + u * sin(G.hdg0);
        elseif abs(dk) < 1e-12
            xx = G.x0 + (sin(th) - sin(G.hdg0)) / G.k0;
            yy = G.y0 - (cos(th) - cos(G.hdg0)) / G.k0;
        else
            uf = linspace(0, G.length, 20 * n + 1)';
            thf = G.hdg0 + G.k0 * uf + 0.5 * dk * uf.^2;
            xf = G.x0 + cumtrapz(uf, cos(thf));
            yf = G.y0 + cumtrapz(uf, sin(thf));
            xx = interp1(uf, xf, u);
            yy = interp1(uf, yf, u);
        end
        s = [s; G.s0 + u];      %#ok<AGROW>
        x = [x; xx];            %#ok<AGROW>
        y = [y; yy];            %#ok<AGROW>
        hdg = [hdg; th];        %#ok<AGROW>
        kap = [kap; k];         %#ok<AGROW>
    end
    ref = struct('s', s, 'x', x, 'y', y, 'hdg', hdg, 'kappa', kap);
end
