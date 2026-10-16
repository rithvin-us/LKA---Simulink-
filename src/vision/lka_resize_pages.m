function Y = lka_resize_pages(X, outSize, method, rows)
% LKA_RESIZE_PAGES imresize of H x W x K image pages as two matrix products (fast on the GPU).
%
% Syntax:
%   Y = lka_resize_pages(X, outSize, method)          like imresize(X, outSize, method)
%   Y = lka_resize_pages(X, outSize, method, rows)    only output rows "rows" (e.g. a crop)
%
% imresize is linear and separable, so Y(:, :, k) = Ry * X(:, :, k) * Rx with Ry and Rx
% obtained by resizing identity matrices with imresize itself: the same kernel and
% antialiasing, exactly. The matrices are built once per size and kept (on the GPU when X
% is a gpuArray), and pagemtimes resizes all pages in one call: about 1 ms on an RTX 4050
% for a 464 x 832 RGB frame where imresize on a uint8 gpuArray takes about 11 ms. Integer
% inputs give the integer result of imresize (rounded and saturated) in single.
    persistent cache
    if isempty(cache)
        cache = containers.Map();
    end
    sz = size(X);
    if nargin < 4 || isempty(rows)
        rows = 1:outSize(1);
    end
    onGpu = isgpuarray(X);
    key = sprintf('%d_%d_%d_%d_%s_%d_%d_%d', sz(1), sz(2), outSize(1), outSize(2), method, ...
        rows(1), rows(end), onGpu);
    if ~isKey(cache, key)
        Ry = imresize(eye(sz(1)), [outSize(1) sz(1)], method);
        Rx = imresize(eye(sz(2)), [sz(2) outSize(2)], method);
        M = {single(Ry(rows, :)), single(Rx)};
        if onGpu
            M = cellfun(@gpuArray, M, 'UniformOutput', false);
        end
        cache(key) = M;
    end
    M = cache(key);
    isInt = isUnderlyingType(X, "integer");
    if isInt
        cls = underlyingType(X);
        lim = double([intmin(cls) intmax(cls)]);
    end
    Y = pagemtimes(M{1}, pagemtimes(single(X), M{2}));
    if isInt
        Y = min(max(round(Y), lim(1)), lim(2));
    end
end
