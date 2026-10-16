function [C, report] = lka_autocalibrate_camera(probs, C0, V, options)
% LKA_AUTOCALIBRATE_CAMERA Camera pitch and height from lane-line probability maps.
%
% Syntax:
%   C = lka_autocalibrate_camera(probs, C0, V)
%   [C, report] = lka_autocalibrate_camera(probs, C0, V, LaneWidth=3.5)
%
% For a video from an unknown camera mount (e.g. the organisers' track video). probs is a
% cell array of detector outputs from frames on a straight or gently curved stretch,
% C0 the camera guess (intrinsics must be known or estimated from the field of view).
%   1. Pitch: on a flat road the two lane lines are parallel, so the measured lane width
%      must not change with distance. Choose the pitch that minimises the median
%      |width(far) - width(near)| over the frames (coarse 0.5 deg grid, then 0.1 deg).
%   2. Height: a flat-ground projection scales every distance by the camera height, so
%      Height = C0.Height * LaneWidth / median(measured width). With LaneWidth = NaN the
%      mount height C0.Height is trusted instead and the lane width is measured.
% While the scale is unknown the lane step starts each side from the nearest line and its
% width gate is opened (0.5..10 m), so a narrow lane is not replaced by a line-to-kerb pair.
% The lateral mount offset stays C0.Y (assumed centred), the roll is assumed zero.
% report: PitchGrid, Cost, WidthAtPitch, Frames used, LaneWidth (at the final scale).

    arguments
        probs cell
        C0 (1,1) struct
        V (1,1) struct
        options.LaneWidth (1,1) double = 3.5
        options.PitchRange (1,2) double = [-4 16]
    end
    V.Track.WidthMin = 0.5;
    V.Track.WidthMax = 10;
    V.Search.SeedFirst = true;
    coarse = options.PitchRange(1):0.5:options.PitchRange(2);
    [costC, widthC] = arrayfun(@(p) evaluatePitch(p), coarse);
    [~, i] = min(costC);
    fine = max(options.PitchRange(1), coarse(i) - 0.5):0.1:min(options.PitchRange(2), coarse(i) + 0.5);
    [costF, widthF] = arrayfun(@(p) evaluatePitch(p), fine);
    [~, j] = min(costF);
    if ~isfinite(costF(j))
        error('lka:vision:autocalFailed', 'No pitch gives both lane lines in the frames.');
    end
    C = C0;
    C.PitchDeg = fine(j);
    if isfinite(options.LaneWidth)
        C.Height = C0.Height * options.LaneWidth / widthF(j);
    end
    report = struct('PitchGrid', [coarse fine], 'Cost', [costC costF], ...
        'WidthAtPitch', [widthC widthF], 'Frames', numel(probs), ...
        'LaneWidth', widthF(j) * C.Height / C0.Height);

    function [cost, width] = evaluatePitch(p)
        Cp = C0;
        Cp.PitchDeg = p;
        cost = inf; width = NaN;
        below = deg2rad(p) + atan((Cp.ImageSize(1) - Cp.PrincipalPoint(2)) / Cp.FocalLength(2));
        if below <= deg2rad(1)
            return                                     % bottom row looks above the road
        end
        cam = lka_camera_model(Cp, V.Bev);
        xN = cam.XNear + 2;
        xF = min(cam.XNear + 14, cam.XFar - 1);
        d = nan(numel(probs), 1);
        w = nan(numel(probs), 1);
        for k = 1:numel(probs)
            s = lka_vision_lane_step(lka_bev_warp(cam, probs{k}), cam, V, lka_vision_state_init(V), V.Ts);
            if s.Left.Strength > 0 && s.Right.Strength > 0
                wN = yAt(s.Left, xN) - yAt(s.Right, xN);
                wF = yAt(s.Left, xF) - yAt(s.Right, xF);
                d(k) = abs(wF - wN);
                w(k) = wN;
            end
        end
        if nnz(~isnan(d)) >= max(1, ceil(numel(probs) / 2))
            cost = median(d, 'omitnan');
            width = median(w, 'omitnan');
        end
    end
end

function y = yAt(b, x)
    y = b.LateralOffset + tan(b.HeadingAngle) * x + b.Curvature / 2 * x.^2 + b.CurvatureDerivative / 6 * x.^3;
end
