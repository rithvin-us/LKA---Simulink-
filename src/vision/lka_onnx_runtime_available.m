function ok = lka_onnx_runtime_available(backend)
% LKA_ONNX_RUNTIME_AVAILABLE True if an ONNX back end for the lane detectors exists.
%
% Syntax:
%   ok = lka_onnx_runtime_available()               % any back end
%   ok = lka_onnx_runtime_available('matlab')       % Deep Learning Toolbox importer add-on
%   ok = lka_onnx_runtime_available('onnxruntime')  % Python onnxruntime via pyenv
    if nargin < 1
        backend = 'auto';
    end
    hasMatlab = exist('importNetworkFromONNX', 'file') == 2;
    hasOrt = false;
    if ~strcmpi(backend, 'matlab')
        try
            py.importlib.import_module('onnxruntime');
            hasOrt = true;
        catch
        end
    end
    switch lower(backend)
        case 'matlab'
            ok = hasMatlab;
        case 'onnxruntime'
            ok = hasOrt;
        otherwise
            ok = hasMatlab || hasOrt;
    end
end
