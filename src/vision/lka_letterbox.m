function [out, pad, scale] = lka_letterbox(img, inputSize, padValue)
% LKA_LETTERBOX Aspect-preserving resize into inputSize with centred constant padding.
%
% Syntax:
%   [out, pad, scale] = lka_letterbox(img, inputSize, padValue)
%
% Same geometry as YOLOP letterbox_for_img(new_shape=640, auto=True) for 1280x720:
% scale 0.5 -> 640x360, 12 rows of padding top and bottom -> 384x640. Box filter
% resize (cv2.INTER_AREA equivalent for integer down-scaling).
% Output pad = [top left nh nw]: the image occupies out(top+1:top+nh, left+1:left+nw, :).

    [h, w, ~] = size(img);
    scale = min(inputSize(1) / h, inputSize(2) / w);
    nh = round(h * scale);
    nw = round(w * scale);
    rs = imresize(img, [nh nw], 'box');
    top = floor((inputSize(1) - nh) / 2);
    left = floor((inputSize(2) - nw) / 2);
    out = repmat(cast(padValue, 'like', img), [inputSize, size(img, 3)]);
    out(top + 1:top + nh, left + 1:left + nw, :) = rs;
    pad = [top, left, nh, nw];
end
