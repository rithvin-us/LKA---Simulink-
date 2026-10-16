% ==============================================================================
% test_vision_assets.m
% Perception assets: detector sidecars, YOLOP ONNX model, LKA track scene, git hygiene.
% ==============================================================================

function tests = test_vision_assets
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
end

function testDetectorSidecarsParse(testCase)
    d = fullfile(testCase.TestData.root, 'models', 'perception');
    common = {'type', 'name', 'onnx', 'input_size', 'channel_order', 'scale', 'mean', 'std'};
    perType.seg = {'resize', 'pad_value', 'output', 'lane_channel'};
    perType.ufld_v2 = {'crop_ratio', 'num_grid_row', 'num_cls_row', 'num_grid_col', 'num_cls_col', ...
        'num_lanes', 'row_lanes', 'col_lanes', 'row_anchor_px', 'anchor_image_height', 'local_width', 'line_width_px'};
    for name = ["yolop", "cnn"]
        cfg = jsondecode(fileread(fullfile(d, name + ".json")));
        testCase.verifyTrue(all(isfield(cfg, common)), name);
        testCase.verifyTrue(isfield(perType, cfg.type), name);
        testCase.verifyTrue(all(isfield(cfg, perType.(cfg.type))), name);
        testCase.verifyNumElements(cfg.input_size, 2, name);
        testCase.verifyNumElements(cfg.mean, 3, name);
        testCase.verifyNumElements(cfg.std, 3, name);
    end
    yolop = jsondecode(fileread(fullfile(d, 'yolop.json')));
    testCase.verifyEqual(yolop.type, 'seg');
    % YOLOP was trained on RGB (AutoDriveDataset converts BGR->RGB); its demo.py and the
    % ADAS_2026 port feed BGR, which hid most lane lines (Unreal render: 4 % -> 31 % left).
    testCase.verifyEqual(yolop.channel_order, 'RGB');
    cnn = jsondecode(fileread(fullfile(d, 'cnn.json')));
    testCase.verifyEqual(cnn.type, 'ufld_v2');
end

function testYolopOnnxPresent(testCase)
    f = fullfile(testCase.TestData.root, 'models', 'perception', 'yolop_ll_384x640.onnx');
    testCase.verifyTrue(isfile(f));
    info = dir(f);
    testCase.verifyGreaterThan(info.bytes, 20e6);
    testCase.verifyLessThan(info.bytes, 50e6);          % below GitHub's 50 MB warning
end

function testLkaSceneExported(testCase)
    d = fullfile(testCase.TestData.root, 'data', 'scenes', 'LKA');
    testCase.verifyTrue(isfile(fullfile(d, 'LKA.usd')));
    testCase.verifyTrue(isfile(fullfile(d, 'Roads.mesh.usd')));
end

function testAdas2026IsIgnored(testCase)
    [st, out] = system(sprintf('git -C "%s" check-ignore ADAS_2026/README.md', testCase.TestData.root));
    testCase.verifyEqual(st, 0, out);
end
