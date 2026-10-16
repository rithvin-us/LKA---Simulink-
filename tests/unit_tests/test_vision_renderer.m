% ==============================================================================
% test_vision_renderer.m
% The synthetic renderer used by the vision tests draws lines where the ground
% truth says, with the LKA sign convention.
% ==============================================================================

function tests = test_vision_renderer
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    testCase.TestData.cam = lka_camera_model(LKA.Vision.Camera, LKA.Vision.Bev);
end

function testLinePixelsWhereExpected(testCase)
    cam = testCase.TestData.cam;
    [img, prob] = lka_render_lane_scene(cam, lka_default_scene());
    uv = round(vehicleToImage(cam.Sensor, [10 1.75; 10 -1.75; 10 0]));
    testCase.verifyEqual(prob(uv(1, 2), uv(1, 1)), single(0.95));
    testCase.verifyEqual(prob(uv(2, 2), uv(2, 1)), single(0.95));
    testCase.verifyEqual(prob(uv(3, 2), uv(3, 1)), single(0.02));
    testCase.verifyEqual(size(img), [720 1280 3]);
end

function testGroundTruthOffset(testCase)
    s = lka_default_scene(); s.Offset = 0.5;          % vehicle 0.5 m left of the centre
    [~, ~, gt] = lka_render_lane_scene(testCase.TestData.cam, s);
    testCase.verifyEqual(gt.Left.LateralOffset, 1.25, 'AbsTol', 1e-3);
    testCase.verifyEqual(gt.Right.LateralOffset, -2.25, 'AbsTol', 1e-3);
end

function testGroundTruthYaw(testCase)
    s = lka_default_scene(); s.Yaw = deg2rad(3);      % vehicle yawed left -> lane rotates right
    [~, ~, gt] = lka_render_lane_scene(testCase.TestData.cam, s);
    testCase.verifyEqual(gt.Left.HeadingAngle, -deg2rad(3), 'AbsTol', deg2rad(0.01));
end

function testGroundTruthCurvature(testCase)
    s = lka_default_scene(); s.Curvature = 1 / 127.5;
    [~, ~, gt] = lka_render_lane_scene(testCase.TestData.cam, s);
    testCase.verifyEqual(gt.Left.Curvature, 1 / 127.5, 'AbsTol', 1e-4);
    testCase.verifyEqual(gt.Right.Curvature, 1 / 127.5, 'AbsTol', 1e-4);
end

function testMissingLineNotDrawn(testCase)
    cam = testCase.TestData.cam;
    s = lka_default_scene(); s.Missing = [false true];
    [~, prob] = lka_render_lane_scene(cam, s);
    uv = round(vehicleToImage(cam.Sensor, [10 -1.75]));
    testCase.verifyEqual(prob(uv(2), uv(1)), single(0.02));
end
