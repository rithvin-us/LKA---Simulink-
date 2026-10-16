function det = lka_detector_classic(~)
% LKA_DETECTOR_CLASSIC Reference lane detector: bright, thin, white/yellow paint.
%
% p = logistic((V - boxmean41(V) - 0.12) / 0.03) on pixels that are white (S < 0.25)
% or yellow (hue 0.10-0.19, S > 0.35); V, S, hue from rgb2hsv. No learning, used as
% the fallback while the ONNX add-on or the team CNN are unavailable and as the
% benchmark baseline.
    det.Name = 'classic (white/yellow local contrast)';
    det.Detect = @detect;

    function prob = detect(rgb)
        hsv = rgb2hsv(rgb);
        val = hsv(:, :, 3);
        sat = hsv(:, :, 2);
        hue = hsv(:, :, 1);
        contrast = val - imboxfilt(val, 41);
        paint = sat < 0.25 | (hue > 0.10 & hue < 0.19 & sat > 0.35);
        prob = single(1 ./ (1 + exp(-(contrast .* paint - 0.12) / 0.03)));
    end
end
