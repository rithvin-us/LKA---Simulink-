function det = lka_detector_create(V)
% LKA_DETECTOR_CREATE Lane detector selected by V.Detector ('yolop' | 'cnn' | 'classic').
%
% Syntax:
%   det = lka_detector_create(V)          V = LKA.Vision
%   prob = det.Detect(rgb)
%
% 'yolop' and 'cnn' read models/perception/<name>.json; its "type" picks the adapter:
% 'seg' (pixel mask, lka_detector_onnx_seg) or 'ufld_v2' (anchors, lka_detector_ufld).
    switch lower(V.Detector)
        case {'yolop', 'cnn'}
            sidecar = fullfile(V.DetectorDir, [lower(V.Detector) '.json']);
            cfg = jsondecode(fileread(sidecar));
            switch cfg.type
                case 'seg'
                    det = lka_detector_onnx_seg(sidecar, V.UseGpu);
                case 'ufld_v2'
                    det = lka_detector_ufld(sidecar, V.UseGpu);
                otherwise
                    error('lka:vision:unknownModelType', 'Unknown model type "%s" in %s.', cfg.type, sidecar);
            end
        case 'classic'
            det = lka_detector_classic(V);
        otherwise
            error('lka:vision:unknownDetector', 'Unknown lane detector "%s" (yolop | cnn | classic).', V.Detector);
    end
end
