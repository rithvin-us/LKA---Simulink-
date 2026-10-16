function [sensor, state, info] = lka_vision_lane_step(bevProb, cam, V, state, dt)
% LKA_VISION_LANE_STEP One camera frame: bird's-eye probability map -> LaneSensorBus.
%
% Syntax:
%   [sensor, state, info] = lka_vision_lane_step(bevProb, cam, V, state, dt)
%
% Per side: sliding-window search around the previous DIRECT fit (warm, +-WarmHalfWidth,
% at most Track.HoldTime old) or around the nominal lane +-width/2 (cold), robust fit,
% plausibility (heading, curvature, side of the vehicle). With both sides found, the
% pair width at the first BEV row must match the width estimate (the weaker side is
% dropped otherwise) and updates it. Only direct fits become the next prior, which
% fixes ADAS_2026 finding M1 (a reconstructed side could lock out the real line).
% One-line operation, coasting and take-over stay in LKA_Controller (lane tracker,
% supervisor), so a missing side is reported with Strength 0.
%
% Inputs:
%   bevProb - transformImage(cam.Bev, probabilityMap)
%   cam     - lka_camera_model
%   V       - LKA.Vision
%   state   - lka_vision_state_init / previous call
%   dt      - frame period [s]
% Outputs:
%   sensor  - LaneSensorBus struct (Left, Right, FrameValid)
%   info    - [nPtsLeft nPtsRight foundLeft foundRight widthEstimate]

    bevProb = double(bevProb);
    names = {'Left', 'Right'};
    sides = [1, -1];
    W = state.Width;
    found = false(1, 2);
    bnd = cell(1, 2);
    nPts = zeros(1, 2);
    for k = 1:2
        prev = state.(names{k});
        prior.Side = sides(k);
        if prev.Valid && prev.Age <= V.Track.HoldTime
            cf = prev.Coeffs;
            prior.Fun = @(x) cf(1) + cf(2) * x + cf(3) * x.^2 + cf(4) * x.^3;
            prior.HalfWidth = V.Search.WarmHalfWidth;
            prior.Cold = false;
        else
            prior.Fun = @(x) sides(k) * W / 2 + 0 * x;
            prior.HalfWidth = V.Search.ColdHalfWidth;
            prior.Cold = true;
        end
        pts = lka_search_boundary(bevProb, cam, prior, V.Search);
        nPts(k) = size(pts, 1);
        [b, ok] = lka_fit_boundary(pts, V.Fit);
        found(k) = ok && abs(b.HeadingAngle) <= V.Track.HeadingMax ...
            && abs(b.Curvature) <= V.Track.CurvatureMax && sides(k) * b.LateralOffset > 0;
        bnd{k} = b;
    end

    if all(found)
        wPair = yAt(bnd{1}, cam.XNear) - yAt(bnd{2}, cam.XNear);
        if wPair < V.Track.WidthMin || wPair > V.Track.WidthMax || abs(wPair - W) > V.Track.WidthTol
            [~, weak] = min([bnd{1}.Strength, bnd{2}.Strength]);
            found(weak) = false;
        else
            state.Width = W + min(1, dt / V.Track.WidthTau) * (wPair - W);
        end
    end

    empty = struct('LateralOffset', 0, 'HeadingAngle', 0, 'Curvature', 0, ...
        'CurvatureDerivative', 0, 'Strength', 0, 'ViewRange', 0);
    sensor = struct('Left', empty, 'Right', empty, 'FrameValid', any(found));
    for k = 1:2
        if found(k)
            b = bnd{k};
            sensor.(names{k}) = b;
            state.(names{k}).Valid = true;
            state.(names{k}).Age = 0;
            state.(names{k}).Coeffs = [b.LateralOffset, tan(b.HeadingAngle), b.Curvature / 2, b.CurvatureDerivative / 6];
        else
            state.(names{k}).Age = state.(names{k}).Age + dt;
            if state.(names{k}).Age > V.Track.HoldTime
                state.(names{k}).Valid = false;
            end
        end
    end
    info = [nPts, double(found), state.Width];
end

function y = yAt(b, x)
    y = b.LateralOffset + tan(b.HeadingAngle) * x + b.Curvature / 2 * x.^2 + b.CurvatureDerivative / 6 * x.^3;
end
