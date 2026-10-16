function det = lka_detector_classic(~)
% LKA_DETECTOR_CLASSIC Reference lane detector: bright, thin, white/yellow paint.
%
% p = logistic((V - boxmean41(V) - 0.12) / 0.03) on pixels that are white (S < 0.25)
% or yellow (hue 0.10-0.19, S > 0.35); V, S, hue from rgb2hsv. No learning, used as
% the fallback while the ONNX add-on or the team CNN are unavailable and as the
% benchmark baseline.
    det.Name = 'classic (white/yellow local contrast)';
    det.Backend = 'matlab';
    det.OnGpu = false;
    det.Pages = 1;
    det.MaxRange = inf;                    % trusted look-ahead [m from the rear axle]
    det.Detect = @detect;
    det.DetectBatch = @detectBatch;

    function [probs, info] = detectBatch(rgbs)
        n = size(rgbs, 4);
        probs = zeros(size(rgbs, 1), size(rgbs, 2), 1, n, 'single');
        info = repmat(struct('Lanes', {{}}, 'Ids', zeros(1, 0), 'Ms', 0), n, 1);
        for k = 1:n
            t0 = tic;
            probs(:, :, 1, k) = detect(rgbs(:, :, :, k));
            info(k).Ms = 1000 * toc(t0);
        end
    end

    function [prob, info] = detect(rgb)
        info = struct('Lanes', {{}}, 'Ids', zeros(1, 0), 'Ms', 0);
        prob = lka_paint_probability(rgb);
    end
end
