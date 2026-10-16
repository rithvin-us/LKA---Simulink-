function [img, prob, gt] = lka_render_lane_scene(cam, scene)
% LKA_RENDER_LANE_SCENE Synthetic camera image of a lane with known geometry.
%
% Syntax:
%   [img, prob, gt] = lka_render_lane_scene(cam, scene)
%
% Every ground pixel of the camera (lka_camera_model) is projected to the road and
% tested against the lane lines. Lane frame: s along the lane, l to the left; centre
% line l = Curvature*s^2/2, lines at +-Width/2. The vehicle rear axle sits at
% (s, l) = (0, Offset) with heading Yaw (+ CCW) relative to the lane.
%
% Outputs:
%   img   uint8 RGB, asphalt 70, paint 235
%   prob  single lane-line probability (0.95 paint, 0.02 elsewhere)
%   gt    .Left / .Right ground-truth boundaries in the vehicle frame (rear axle):
%         LateralOffset, HeadingAngle, Curvature, CurvatureDerivative

    C = cam.Params;
    H = C.ImageSize(1); W = C.ImageSize(2);
    [u, v] = meshgrid(1:W, 1:H);
    horizon = C.PrincipalPoint(2) - C.FocalLength(2) * tan(deg2rad(C.PitchDeg));
    ground = v(:) > horizon + 2;
    xy = nan(numel(u), 2);
    xy(ground, :) = imageToVehicle(cam.Sensor, [u(ground) v(ground)]);
    x = xy(:, 1); y = xy(:, 2);
    ok = ground & x > C.X & x < C.X + 80;

    c = cos(scene.Yaw); s = sin(scene.Yaw);
    sLane = x .* c - y .* s;
    rel = scene.Offset + x .* s + y .* c - 0.5 * scene.Curvature * sLane.^2;
    hw = scene.LineWidth / 2;
    onL = abs(rel - scene.Width / 2) < hw;
    onR = abs(rel + scene.Width / 2) < hw;
    dashOn = mod(sLane, scene.DashLen + scene.GapLen) < scene.DashLen;
    if scene.Dashed(1), onL = onL & dashOn; end
    if scene.Dashed(2), onR = onR & dashOn; end
    if scene.Missing(1), onL(:) = false; end
    if scene.Missing(2), onR(:) = false; end
    on = ok & (onL | onR);
    if ~isnan(scene.Distractor)
        on = on | (ok & abs(rel - scene.Distractor) < hw);
    end
    on = reshape(on, H, W);

    gray = uint8(70 * ones(H, W));
    gray(on) = 235;
    img = repmat(gray, 1, 1, 3);
    prob = single(0.02 * ones(H, W));
    prob(on) = 0.95;

    gt.Left = boundary(+1);
    gt.Right = boundary(-1);

    function b = boundary(side)
        sv = linspace(-5, 60, 400)';
        lv = side * scene.Width / 2 + 0.5 * scene.Curvature * sv.^2 - scene.Offset;
        xb = sv * c + lv * s;
        yb = -sv * s + lv * c;
        keep = xb >= 0 & xb <= 40;
        p = polyfit(xb(keep), yb(keep), 3);
        b = struct('LateralOffset', p(4), 'HeadingAngle', atan(p(3)), ...
            'Curvature', 2 * p(2), 'CurvatureDerivative', 6 * p(1));
    end
end
