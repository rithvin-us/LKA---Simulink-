function prob = lka_rasterize_lanes(lanes, imgSize, widthPx)
% LKA_RASTERIZE_LANES Lane point lists -> lane-line probability map.
%
% Syntax:
%   prob = lka_rasterize_lanes(lanes, imgSize, widthPx)     lanes from lka_ufld_decode
%
% Consecutive points of a lane (anchor gap <= 3) are joined by a widthPx line whose value
% is the smaller existence probability of its two ends. One insertShape call per frame.
    segs = zeros(0, 4);
    vals = zeros(0, 1);
    for k = 1:numel(lanes)
        P = lanes{k};
        if size(P, 1) < 2
            continue
        end
        join = diff(P(:, 4)) <= 3;
        a = P(1:end - 1, :);
        b = P(2:end, :);
        segs = [segs; a(join, 1:2), b(join, 1:2)]; %#ok<AGROW>
        vals = [vals; min(a(join, 3), b(join, 3))]; %#ok<AGROW>
    end
    canvas = zeros([imgSize 3], 'uint8');
    if ~isempty(segs)
        c = repmat(round(255 * vals), 1, 3);
        canvas = insertShape(canvas, 'Line', segs, 'Color', c, 'LineWidth', widthPx, 'SmoothEdges', false);
    end
    prob = single(canvas(:, :, 1)) / 255;
end
