function B = lka_bev_warp(cam, I)
% LKA_BEV_WARP Metric bird's-eye view of one or several camera-sized images.
%
% Syntax:
%   B = lka_bev_warp(cam, I)        cam = lka_camera_model, I = H x W x N (gray / probability)
%
% Same bilinear inverse mapping as transformImage(cam.Bev, I) (precomputed in
% cam.Sampler), but N images at once and on the GPU when I is a gpuArray (the result
% then stays on the GPU). Integer images are converted to single.
    S = cam.Sampler;
    sz = size(I);
    if ~isequal(sz(1:2), S.ImageSize)
        error('lka:vision:imageSize', 'Image %dx%d does not match the camera %dx%d.', ...
            sz(1), sz(2), S.ImageSize(1), S.ImageSize(2));
    end
    if isinteger(I)
        I = im2single(I);
    end
    n = prod(sz(3:end));
    I = reshape(I, [], n);
    if isgpuarray(I)
        [idx, w] = gpuSampler(S);
    else
        idx = S.Index; w = S.Weight;
    end
    w = cast(w, 'like', I);
    B = w(:, 1) .* I(idx(:, 1), :) + w(:, 2) .* I(idx(:, 2), :) ...
        + w(:, 3) .* I(idx(:, 3), :) + w(:, 4) .* I(idx(:, 4), :);
    B = reshape(B, [S.Size, sz(3:end)]);
end

function [idx, w] = gpuSampler(S)
% Sampler tables are uploaded once per camera and kept on the GPU.
    persistent key gIdx gW
    if isempty(key) || ~strcmp(key, S.Key)
        gIdx = gpuArray(S.Index);
        gW = gpuArray(S.Weight);
        key = S.Key;
    end
    idx = gIdx; w = gW;
end
