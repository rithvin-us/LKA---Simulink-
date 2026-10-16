function Y = lka_onnx_output_hwc(Y, inputSize)
% LKA_ONNX_OUTPUT_HWC Reorders a segmentation network output to H x W x C.
%
% Imported ONNX networks may return SSCB, BCSS or unformatted layouts; the H and W
% dimensions are found by size (non-square inputs only).
    Y = squeeze(Y);
    sz = size(Y);
    hDim = find(sz == inputSize(1), 1);
    wDim = find(sz == inputSize(2), 1);
    if isempty(hDim) || isempty(wDim) || hDim == wDim
        error('lka:vision:outputShape', 'Network output %s does not match input %dx%d.', ...
            mat2str(sz), inputSize(1), inputSize(2));
    end
    if numel(sz) == 2
        Y = permute(Y, [hDim wDim]);
    else
        Y = permute(Y, [hDim wDim setdiff(1:3, [hDim wDim])]);
    end
end
