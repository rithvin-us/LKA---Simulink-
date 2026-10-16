% ==============================================================================
% test_vision_lane_step.m
% Camera frame (probability map) -> LaneSensorBus: accuracy, signs, robustness.
% Tolerances: offset 5 cm, heading 0.4 deg, curvature 1.5e-3 1/m (baja camera).
% ==============================================================================

function tests = test_vision_lane_step
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    evalin('base', 'create_lka_buses;');
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    testCase.TestData.V = LKA.Vision;
    testCase.TestData.cam = lka_camera_model(LKA.Vision.Camera, LKA.Vision.Bev);
end

function testStraightCentred(testCase), checkScene(testCase, lka_default_scene()); end

function testOffsetLeft(testCase)
    s = lka_default_scene(); s.Offset = 0.5;
    sensor = checkScene(testCase, s);
    testCase.verifyLessThan(sensor.Left.LateralOffset, 1.75);     % boundaries move right
    testCase.verifyLessThan(sensor.Right.LateralOffset, -1.75);
end

function testOffsetRight(testCase), s = lka_default_scene(); s.Offset = -0.5; checkScene(testCase, s); end
function testYawLeft(testCase),     s = lka_default_scene(); s.Yaw = deg2rad(3);  checkScene(testCase, s); end
function testYawRight(testCase),    s = lka_default_scene(); s.Yaw = deg2rad(-3); checkScene(testCase, s); end
function testCurveLeft(testCase),   s = lka_default_scene(); s.Curvature = 1/127.5;  checkScene(testCase, s); end
function testCurveRight(testCase),  s = lka_default_scene(); s.Curvature = -1/127.5; checkScene(testCase, s); end
function testTightCurve(testCase),  s = lka_default_scene(); s.Curvature = 1/50; checkScene(testCase, s); end
function testDashedLeft(testCase),  s = lka_default_scene(); s.Dashed = [true false]; checkScene(testCase, s); end

function testDistractorIgnored(testCase)
    s = lka_default_scene(); s.Distractor = 1.1;
    checkScene(testCase, s);
end

function testMissingRight(testCase)
    s = lka_default_scene(); s.Missing = [false true];
    [sensor, gt] = runFrame(testCase, s);
    testCase.verifyEqual(sensor.Right.Strength, 0);
    testCase.verifyTrue(sensor.FrameValid);
    testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.05);
end

function testBothMissing(testCase)
    s = lka_default_scene(); s.Missing = [true true];
    sensor = runFrame(testCase, s);
    testCase.verifyFalse(sensor.FrameValid);
end

function testSkeletonCameraPitchZero(testCase)
    C = testCase.TestData.V.Camera;
    C.X = 2.6; C.Height = 1.1; C.PitchDeg = 0;
    cam = lka_camera_model(C, testCase.TestData.V.Bev);
    s = lka_default_scene(); s.Curvature = 1/127.5;
    [~, prob, gt] = lka_render_lane_scene(cam, s);
    sensor = lka_vision_lane_step(double(transformImage(cam.Bev, prob)), cam, testCase.TestData.V, ...
        lka_vision_state_init(testCase.TestData.V), 0.05);
    testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.08);
    testCase.verifyEqual(sensor.Left.Curvature, gt.Left.Curvature, 'AbsTol', 2.5e-3);
end

function testSequenceTracksOffsetRamp(testCase)
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    state = lka_vision_state_init(V);
    for off = linspace(0, 0.4, 9)
        s = lka_default_scene(); s.Offset = off;
        [~, prob, gt] = lka_render_lane_scene(cam, s);
        [sensor, state] = lka_vision_lane_step(double(transformImage(cam.Bev, prob)), cam, V, state, 0.05);
        testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.05);
        testCase.verifyEqual(sensor.Right.LateralOffset, gt.Right.LateralOffset, 'AbsTol', 0.05);
    end
    testCase.verifyEqual(state.Width, 3.5, 'AbsTol', 0.05);
end

function testHoldExpires(testCase)
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    [~, prob] = lka_render_lane_scene(cam, lka_default_scene());
    [~, state] = lka_vision_lane_step(double(transformImage(cam.Bev, prob)), cam, V, lka_vision_state_init(V), 0.05);
    empty = zeros(numel(cam.RowX), numel(cam.ColY));
    for k = 1:21
        [~, state] = lka_vision_lane_step(empty, cam, V, state, 0.05);
    end
    testCase.verifyFalse(state.Left.Valid);
    testCase.verifyFalse(state.Right.Valid);
end

function testBusFieldOrder(testCase)
    sensor = runFrame(testCase, lka_default_scene());
    bus = evalin('base', 'LaneBoundaryBus');
    testCase.verifyEqual(fieldnames(sensor.Left)', {bus.Elements.Name});
    sbus = evalin('base', 'LaneSensorBus');
    testCase.verifyEqual(fieldnames(sensor)', {sbus.Elements.Name});
    testCase.verifyClass(sensor.FrameValid, 'logical');
    testCase.verifyClass(sensor.Left.LateralOffset, 'double');
end

function testNarrowLaneWidthAccepted(testCase)
    % A 3.0 m lane (outside WidthInit +- WidthTol) must not lock out one side forever.
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    state = lka_vision_state_init(V);
    s = lka_default_scene(); s.Width = 3.0;
    [~, prob, gt] = lka_render_lane_scene(cam, s);
    bev = double(transformImage(cam.Bev, prob));
    for k = 1:15
        [sensor, state] = lka_vision_lane_step(bev, cam, V, state, 0.05);
    end
    testCase.verifyGreaterThan(sensor.Left.Strength, 0);
    testCase.verifyGreaterThan(sensor.Right.Strength, 0);
    testCase.verifyEqual(sensor.Right.LateralOffset, gt.Right.LateralOffset, 'AbsTol', 0.05);
    testCase.verifyEqual(state.Width, 3.0, 'AbsTol', 0.05);
end

function testColdStartFarOffCentre(testCase)
    s = lka_default_scene(); s.Offset = 1.2;             % left line only 0.55 m from the axis
    checkScene(testCase, s);
end

function testWarmPriorExpiresQuickly(testCase)
    % Without ego-motion compensation the previous fit is only a prior for PriorMaxAge.
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    [~, prob] = lka_render_lane_scene(cam, lka_default_scene());
    [~, state] = lka_vision_lane_step(double(transformImage(cam.Bev, prob)), cam, V, lka_vision_state_init(V), 0.05);
    empty = zeros(numel(cam.RowX), numel(cam.ColY));
    n = ceil(V.Track.HoldTime / 0.05) + 1;
    for k = 1:n
        [~, state] = lka_vision_lane_step(empty, cam, V, state, 0.05);
    end
    testCase.verifyFalse(state.Left.Valid);
    testCase.verifyLessThanOrEqual(V.Track.HoldTime, 0.3);
end

function testWidthChangeReseeds(testCase)
    % 3.5 m lane, then 2.8 m (outside WidthTol of the estimate): after WidthReseedFrames
    % rejected frames the estimate is re-seeded and both sides are reported again.
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    state = lka_vision_state_init(V);
    [~, p35] = lka_render_lane_scene(cam, lka_default_scene());
    s = lka_default_scene(); s.Width = 2.8;
    [~, p28, gt] = lka_render_lane_scene(cam, s);
    b35 = double(transformImage(cam.Bev, p35)); b28 = double(transformImage(cam.Bev, p28));
    for k = 1:5
        [~, state] = lka_vision_lane_step(b35, cam, V, state, 0.05);
    end
    for k = 1:V.Track.WidthReseedFrames + 3
        [sensor, state] = lka_vision_lane_step(b28, cam, V, state, 0.05);
    end
    testCase.verifyGreaterThan(sensor.Left.Strength, 0);
    testCase.verifyGreaterThan(sensor.Right.Strength, 0);
    testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.05);
    testCase.verifyEqual(state.Width, 2.8, 'AbsTol', 0.05);
end

% ------------------------------------------------------------------ helpers
function [sensor, gt] = runFrame(testCase, scene)
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    [~, prob, gt] = lka_render_lane_scene(cam, scene);
    sensor = lka_vision_lane_step(double(transformImage(cam.Bev, prob)), cam, V, lka_vision_state_init(V), 0.05);
end

function sensor = checkScene(testCase, scene)
    [sensor, gt] = runFrame(testCase, scene);
    testCase.verifyTrue(sensor.FrameValid);
    for side = ["Left", "Right"]
        b = sensor.(side); g = gt.(side);
        testCase.verifyGreaterThan(b.Strength, 0.3, side);
        testCase.verifyEqual(b.LateralOffset, g.LateralOffset, 'AbsTol', 0.05, side);
        testCase.verifyEqual(b.HeadingAngle, g.HeadingAngle, 'AbsTol', deg2rad(0.4), side);
        testCase.verifyEqual(b.Curvature, g.Curvature, 'AbsTol', 1.5e-3, side);
    end
end
