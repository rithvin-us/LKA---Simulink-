function be = lka_onnx_backend(onnxFile, useGpu, backend)
% LKA_ONNX_BACKEND Runs an ONNX network on a normalised H x W x C image tensor.
%
% Syntax:
%   be = lka_onnx_backend(onnxFile, useGpu)
%   be = lka_onnx_backend(onnxFile, useGpu, backend)    backend 'auto' | 'matlab' | 'onnxruntime'
%   outs = be.Predict(x)       x single H x W x C -> cell array of network outputs
%
% 'matlab'      Deep Learning Toolbox importer add-on (dlnetwork, GPU when available).
% 'onnxruntime' Python onnxruntime in-process (tools/lka_ort.py, CPU); works without the
%               add-on as long as MATLAB's pyenv Python has onnxruntime installed.
% 'auto'        'matlab' when the add-on is installed, else 'onnxruntime'.
% The output layout differs between back ends (dlarray SSCB vs ONNX NCHW); callers
% reorder by size (lka_onnx_output_hwc, lka_ufld_outputs).
    if nargin < 3 || isempty(backend)
        backend = 'auto';
    end
    if strcmpi(backend, 'auto')
        if lka_onnx_runtime_available('matlab')
            backend = 'matlab';
        else
            backend = 'onnxruntime';
        end
    end
    switch lower(backend)
        case 'matlab'
            net = lka_import_onnx_detector(onnxFile);
            useGpu = useGpu && canUseGPU();
            if useGpu
                net = dlupdate(@gpuArray, net);      % weights to the GPU once
            end
            nOut = numel(net.OutputNames);
            be.Name = 'matlab';
            be.Predict = @predictMatlab;
        case 'onnxruntime'
            if ~lka_onnx_runtime_available('onnxruntime')
                error('lka:vision:noOnnxBackend', ['No ONNX back end: install the add-on ' ...
                    '"Deep Learning Toolbox Converter for ONNX Model Format", or onnxruntime ' ...
                    'for the Python used by pyenv (pip install onnxruntime).']);
            end
            tools = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'tools');
            if count(py.sys.path, tools) == 0
                insert(py.sys.path, int32(0), tools);
            end
            mod = py.importlib.import_module('lka_ort');
            mod.load(onnxFile);
            be.Name = 'onnxruntime';
            be.Predict = @(x) cellfun(@single, cell(mod.run(onnxFile, single(x))), 'UniformOutput', false);
        otherwise
            error('lka:vision:unknownBackend', 'Unknown ONNX back end "%s".', backend);
    end

    function outs = predictMatlab(x)
        X = dlarray(x, 'SSCB');
        if useGpu
            X = gpuArray(X);
        end
        outs = cell(1, nOut);
        [outs{:}] = predict(net, X);
        outs = cellfun(@(y) gather(extractdata(y)), outs, 'UniformOutput', false);
    end
end
