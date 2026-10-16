function m = lka_seg_metrics(prob, gtMask, thr, tolPx)
% LKA_SEG_METRICS Lane-line segmentation quality of one frame.
%
% Syntax:
%   m = lka_seg_metrics(prob, gtMask, thr, tolPx)
%
% Tolerant precision / recall (a prediction within tolPx of a true lane pixel counts,
% and vice versa), F1, strict IoU, and pixel accuracy. Pixel accuracy is reported only
% to expose its bias: lane pixels are ~3-5 % of a frame, so an empty mask scores ~96 %.
% Without any ground-truth lane pixel (scene without a semantic lane class) every
% metric is NaN.
    gtMask = logical(gtMask);
    if ~any(gtMask(:))
        m = struct('Precision', NaN, 'Recall', NaN, 'F1', NaN, 'IoU', NaN, 'PixelAccuracy', NaN);
        return
    end
    pred = prob > thr;
    se = strel('disk', tolPx);
    m.Precision = nnz(pred & imdilate(gtMask, se)) / max(nnz(pred), 1);
    m.Recall = nnz(gtMask & imdilate(pred, se)) / max(nnz(gtMask), 1);
    m.F1 = 2 * m.Precision * m.Recall / max(m.Precision + m.Recall, eps);
    m.IoU = nnz(pred & gtMask) / max(nnz(pred | gtMask), 1);
    m.PixelAccuracy = mean(pred(:) == gtMask(:));
end
