function frame = lka_annotate_frame(img, preview, sensor, hud, cam)
% LKA_ANNOTATE_FRAME Camera frame with detected boundaries, bird's-eye inset and HUD.
%
% Syntax:
%   frame = lka_annotate_frame(img, preview, sensor, hud, cam)
%
% Left boundary green, right boundary blue (only with Strength > 0), bird's-eye
% probability preview (parula, x3) top-right, one HUD line: boundary offsets at the rear
% axle, steering command [deg, + left], LKA state (0 OFF ... 4 TAKEOVER).

    frame = img;
    xs = linspace(cam.XNear, cam.XFar, 40)';
    sides = {'Left', 'Right'};
    colors = {[0 255 0], [0 160 255]};
    for k = 1:2
        b = sensor.(sides{k});
        if b.Strength <= 0
            continue
        end
        y = b.LateralOffset + tan(b.HeadingAngle) * xs + b.Curvature / 2 * xs.^2 + b.CurvatureDerivative / 6 * xs.^3;
        uv = vehicleToImage(cam.Sensor, [xs y]);
        in = all(isfinite(uv), 2) & uv(:, 1) >= 1 & uv(:, 1) <= size(img, 2) & uv(:, 2) >= 1 & uv(:, 2) <= size(img, 1);
        if nnz(in) >= 2
            frame = insertShape(frame, 'Line', reshape(uv(in, :)', 1, []), 'Color', colors{k}, 'LineWidth', 5);
        end
    end

    pv = im2uint8(ind2rgb(im2uint8(mat2gray(preview, [0 1])), parula(256)));
    pv = imresize(pv, 3, 'nearest');
    [ph, pw, ~] = size(pv);
    frame(11:10 + ph, end - pw - 9:end - 10, :) = pv;

    txt = sprintf('L %+.2f m  R %+.2f m  |  steer %+.1f deg  |  LKA state %d', ...
        sensor.Left.LateralOffset * (sensor.Left.Strength > 0), ...
        sensor.Right.LateralOffset * (sensor.Right.Strength > 0), hud.SteerDeg, round(hud.State));
    frame = insertText(frame, [10 10], txt, 'FontSize', 22, 'BoxColor', 'black', ...
        'BoxOpacity', 0.6, 'TextColor', 'white');
end
