function prob = lka_paint_probability(rgb)
% LKA_PAINT_PROBABILITY Probability of bright white / yellow road paint per pixel.
%
% Syntax:
%   prob = lka_paint_probability(rgb)     rgb uint8 H x W x 3 (CPU or gpuArray) -> single H x W
%
% p = logistic((V - boxmean41(V) - 0.12) / 0.03) on pixels that are white (S < 0.25) or
% yellow (hue 0.10-0.19, S > 0.35); V, S, hue from rgb2hsv. The classic detector and the
% paint check of the team CNN (lka_detector_ufld) use it; on a gpuArray it runs on the GPU.
    hsv = rgb2hsv(rgb);
    val = hsv(:, :, 3);
    sat = hsv(:, :, 2);
    hue = hsv(:, :, 1);
    if isgpuarray(val)
        contrast = val - imfilter(val, ones(41, 'like', val) / 41^2, 'replicate');   % imboxfilt has no GPU version
    else
        contrast = val - imboxfilt(val, 41);
    end
    paint = sat < 0.25 | (hue > 0.10 & hue < 0.19 & sat > 0.35);
    prob = single(1 ./ (1 + exp(-(contrast .* paint - 0.12) / 0.03)));
end
