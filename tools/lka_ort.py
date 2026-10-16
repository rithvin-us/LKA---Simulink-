"""onnxruntime back end for the MATLAB lane detectors (lka_onnx_backend.m).

Used when the MATLAB "Deep Learning Toolbox Converter for ONNX Model Format" add-on is
not installed: MATLAB calls these functions in-process through its Python interface.

    load(path)    -> number of outputs (creates and caches the session)
    run(path, x)  -> list of outputs, float32, C order, ONNX dimension order
                     x: H x W x C float32 image tensor (already normalised) from MATLAB
"""
import os

import numpy as np
import onnxruntime as ort

_SESSIONS = {}


def load(path):
    if path not in _SESSIONS:
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = max(1, (os.cpu_count() or 2) // 2)
        _SESSIONS[path] = ort.InferenceSession(path, sess_options=opts,
                                               providers=["CPUExecutionProvider"])
    return len(_SESSIONS[path].get_outputs())


def run(path, x):
    sess = _SESSIONS[path]
    a = np.asarray(x, dtype=np.float32)
    a = np.ascontiguousarray(a.transpose(2, 0, 1)[None])
    outs = sess.run(None, {sess.get_inputs()[0].name: a})
    return [np.ascontiguousarray(o, dtype=np.float32) for o in outs]
