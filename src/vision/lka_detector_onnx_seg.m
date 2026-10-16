function det = lka_detector_onnx_seg(sidecarFile, useGpu)
% LKA_DETECTOR_ONNX_SEG Lane detector from an ONNX segmentation model + JSON sidecar.
%
% Syntax:
%   det = lka_detector_onnx_seg(sidecarFile, useGpu)
%   prob = det.Detect(rgb)        rgb uint8 H x W x 3 -> single H x W lane probability
%
% Contract and sidecar fields: models/perception/README.md (type "seg", e.g. YOLOP).

    cfg = jsondecode(fileread(sidecarFile));
    onnxFile = fullfile(fileparts(sidecarFile), cfg.onnx);
    if ~isfile(onnxFile)
        error('lka:vision:modelMissing', ['Lane model "%s" not found. Export it to ONNX as ' ...
            'described in models/perception/README.md.'], onnxFile);
    end
    net = lka_import_onnx_detector(onnxFile);
    inSize = cfg.input_size(:)';
    mean3 = reshape(single(cfg.mean), 1, 1, []);
    std3 = reshape(single(cfg.std), 1, 1, []);
    useGpu = useGpu && canUseGPU();
    if useGpu
        net = dlupdate(@gpuArray, net);      % weights to the GPU once, not on every predict
    end
    det.Name = cfg.name;
    det.Detect = @detect;

    function prob = detect(rgb)
        im = rgb;
        if strcmpi(cfg.channel_order, 'BGR')
            im = im(:, :, [3 2 1]);
        end
        [h, w, ~] = size(im);
        if strcmpi(cfg.resize, 'letterbox')
            [x, pad] = lka_letterbox(im, inSize, cfg.pad_value);
        else
            x = imresize(im, inSize, 'bilinear');
            pad = [0 0 inSize];
        end
        x = (single(x) / single(cfg.scale) - mean3) ./ std3;
        X = dlarray(x, 'SSCB');
        if useGpu
            X = gpuArray(X);
        end
        Y = lka_onnx_output_hwc(gather(extractdata(predict(net, X))), inSize);
        switch lower(cfg.output)
            case 'softmax'
                e = exp(Y - max(Y, [], 3));
                p = e(:, :, cfg.lane_channel) ./ sum(e, 3);
            case 'sigmoid'
                p = 1 ./ (1 + exp(-Y(:, :, cfg.lane_channel)));
            otherwise
                p = Y(:, :, cfg.lane_channel);
        end
        p = p(pad(1) + 1:pad(1) + pad(3), pad(2) + 1:pad(2) + pad(4));
        prob = single(min(max(imresize(p, [h w], 'bilinear'), 0), 1));
    end
end
