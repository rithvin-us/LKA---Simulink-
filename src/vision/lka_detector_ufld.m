function det = lka_detector_ufld(sidecarFile, useGpu)
% LKA_DETECTOR_UFLD Team CNN (UFLDv2) lane detector -> lane-line probability map.
%
% Syntax:
%   det = lka_detector_ufld(sidecarFile, useGpu)
%   prob = det.Detect(rgb)        rgb uint8 H x W x 3 -> single H x W
%
% Preprocessing as the UFLDv2 test transform: resize to (input_size(1)/crop_ratio) x
% input_size(2), keep the bottom input_size(1) rows, scale and ImageNet-normalise
% (RGB). The four heads are decoded (lka_ufld_decode) and rasterised
% (lka_rasterize_lanes), so the team CNN uses the same bird's-eye / fit / tracker chain as
% YOLOP.
    cfg = jsondecode(fileread(sidecarFile));
    onnxFile = fullfile(fileparts(sidecarFile), cfg.onnx);
    if ~isfile(onnxFile)
        error('lka:vision:modelMissing', ['Lane model "%s" not found. Export it to ONNX as ' ...
            'described in models/perception/README.md.'], onnxFile);
    end
    net = lka_import_onnx_detector(onnxFile);
    inSize = cfg.input_size(:)';
    fullH = floor(inSize(1) / cfg.crop_ratio + 1e-9);
    mean3 = reshape(single(cfg.mean), 1, 1, []);
    std3 = reshape(single(cfg.std), 1, 1, []);
    useGpu = useGpu && canUseGPU();
    if useGpu
        net = dlupdate(@gpuArray, net);      % weights to the GPU once, not on every predict
    end
    nOut = numel(net.OutputNames);
    det.Name = cfg.name;
    det.Detect = @detect;

    function prob = detect(rgb)
        im = rgb;
        if strcmpi(cfg.channel_order, 'BGR')
            im = im(:, :, [3 2 1]);
        end
        [h, w, ~] = size(im);
        x = imresize(im, [fullH inSize(2)], 'bilinear');
        x = x(end - inSize(1) + 1:end, :, :);
        x = (single(x) / single(cfg.scale) - mean3) ./ std3;
        X = dlarray(x, 'SSCB');
        if useGpu
            X = gpuArray(X);
        end
        outs = cell(1, nOut);
        [outs{:}] = predict(net, X);
        o = lka_ufld_outputs(cellfun(@(y) gather(extractdata(y)), outs, 'UniformOutput', false), cfg);
        prob = lka_rasterize_lanes(lka_ufld_decode(o, cfg, [h w]), [h w], cfg.line_width_px);
    end
end
