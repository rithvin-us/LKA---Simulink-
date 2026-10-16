function make_synthetic_vision_dataset(folder, n, V)
% MAKE_SYNTHETIC_VISION_DATASET Random synthetic lane frames in the benchmark format.
%
% Syntax:
%   make_synthetic_vision_dataset(folder, n, V)          V = LKA.Vision
%
% Offsets +-0.6 m, yaw +-4 deg, curvature +-1/50 1/m, random dashed sides, rendered with
% lka_render_lane_scene. Lane-marking label id 1. Seeded (rng 11) for repeatability.
    if ~isfolder(folder), mkdir(folder); end
    cam = lka_camera_model(V.Camera, V.Bev);
    rng(11);
    meta = struct('t', zeros(n, 1), 'pose', zeros(n, 3), 'gtLeft', zeros(n, 4), ...
        'gtRight', zeros(n, 4), 'files', {cell(n, 1)}, 'Ts', V.Ts, 'LaneLabelId', 1);
    for i = 1:n
        s = lka_default_scene();
        s.Offset = 1.2 * (rand - 0.5);
        s.Yaw = deg2rad(8 * (rand - 0.5));
        s.Curvature = (2 * rand - 1) / 50;
        s.Dashed = rand(1, 2) < 0.3;
        [img, prob, gt] = lka_render_lane_scene(cam, s);
        name = sprintf('frame_%05d', i);
        imwrite(img, fullfile(folder, [name '.png']));
        imwrite(uint8(prob > 0.5), fullfile(folder, [name '_label.png']));
        meta.t(i) = (i - 1) * V.Ts;
        meta.gtLeft(i, :) = [gt.Left.LateralOffset gt.Left.HeadingAngle gt.Left.Curvature gt.Left.CurvatureDerivative];
        meta.gtRight(i, :) = [gt.Right.LateralOffset gt.Right.HeadingAngle gt.Right.Curvature gt.Right.CurvatureDerivative];
        meta.files{i} = name;
    end
    save(fullfile(folder, 'meta.mat'), 'meta');
end
