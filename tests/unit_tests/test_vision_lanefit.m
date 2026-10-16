% ==============================================================================
% test_vision_lanefit.m
% Sliding-window boundary search and robust polynomial fit in the metric BEV.
% ==============================================================================

function tests = test_vision_lanefit
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    testCase.TestData.V = LKA.Vision;
    testCase.TestData.cam = lka_camera_model(LKA.Vision.Camera, LKA.Vision.Bev);
end

% ------------------------------------------------------------------ fit
function testFitRecoversPolynomial(testCase)
    x = linspace(5, 30, 400)';
    y = 1.6 + 0.02 * x + 0.003 * x.^2;
    [b, ok] = lka_fit_boundary([x y 0.9 * ones(size(x))], testCase.TestData.V.Fit);
    testCase.verifyTrue(ok);
    testCase.verifyEqual(b.LateralOffset, 1.6, 'AbsTol', 1e-3);
    testCase.verifyEqual(b.HeadingAngle, atan(0.02), 'AbsTol', 1e-4);
    testCase.verifyEqual(b.Curvature, 0.006, 'AbsTol', 1e-4);
    testCase.verifyEqual(fieldnames(b)', {'LateralOffset', 'HeadingAngle', 'Curvature', ...
        'CurvatureDerivative', 'Strength', 'ViewRange'});
end

function testFitRejectsOutliers(testCase)
    rng(3);
    x = linspace(5, 30, 400)';
    y = 1.75 + 0 * x;
    bad = rand(size(x)) < 0.1;
    y(bad) = y(bad) + 1.0;
    b = lka_fit_boundary([x y 0.9 * ones(size(x))], testCase.TestData.V.Fit);
    testCase.verifyEqual(b.LateralOffset, 1.75, 'AbsTol', 0.02);
end

function testFitNeedsEnoughPoints(testCase)
    x = linspace(5, 30, 20)';
    [b, ok] = lka_fit_boundary([x x * 0 + 1.75 x * 0 + 0.9], testCase.TestData.V.Fit);
    testCase.verifyFalse(ok);
    testCase.verifyEqual(b.Strength, 0);
end

function testShortSpanIsLinearAndWeak(testCase)
    x = linspace(6, 11, 200)';
    y = 1.75 + 0.01 * x.^2;
    [b, ok] = lka_fit_boundary([x y 0.9 * ones(size(x))], testCase.TestData.V.Fit);
    testCase.verifyTrue(ok);
    testCase.verifyEqual(b.Curvature, 0);
    testCase.verifyLessThan(b.Strength, 0.5);
end

% ------------------------------------------------------------------ search
function testColdSearchFindsBothLines(testCase)
    [bev, cam, S] = bevOf(testCase, lka_default_scene());
    L = lka_search_boundary(bev, cam, coldPrior(+1), S);
    R = lka_search_boundary(bev, cam, coldPrior(-1), S);
    testCase.verifyEqual(mean(L(:, 2)), 1.75, 'AbsTol', 0.03);
    testCase.verifyEqual(mean(R(:, 2)), -1.75, 'AbsTol', 0.03);
end

function testDashedLineStillFound(testCase)
    s = lka_default_scene(); s.Dashed = [true false];
    [bev, cam, S] = bevOf(testCase, s);
    [L, nHit] = lka_search_boundary(bev, cam, coldPrior(+1), S);
    testCase.verifyGreaterThanOrEqual(nHit, 4);
    testCase.verifyEqual(mean(L(:, 2)), 1.75, 'AbsTol', 0.04);
end

function testDistractorInsideWindowIgnored(testCase)
    s = lka_default_scene(); s.Distractor = 1.1;       % extra line 0.65 m inside the left line
    [bev, cam, S] = bevOf(testCase, s);
    L = lka_search_boundary(bev, cam, coldPrior(+1), S);
    testCase.verifyEqual(mean(L(:, 2)), 1.75, 'AbsTol', 0.05);
end

function testCurveFollowed(testCase)
    s = lka_default_scene(); s.Curvature = 1 / 50;
    [bev, cam, S] = bevOf(testCase, s);
    [L, nHit] = lka_search_boundary(bev, cam, coldPrior(+1), S);
    testCase.verifyGreaterThanOrEqual(nHit, 10);
    farX = max(L(:, 1));
    testCase.verifyGreaterThan(farX, 15);
end

function testWindowOutsideBevDoesNotThrow(testCase)
    [bev, cam, S] = bevOf(testCase, lka_default_scene());
    prior = struct('Fun', @(x) 20 + 0 * x, 'HalfWidth', 0.45, 'Side', 1, 'Cold', false);
    [pts, nHit] = lka_search_boundary(bev, cam, prior, S);
    testCase.verifyEmpty(pts);
    testCase.verifyEqual(nHit, 0);
end

% ------------------------------------------------------------------ helpers
function [bev, cam, S] = bevOf(testCase, scene)
    cam = testCase.TestData.cam;
    [~, prob] = lka_render_lane_scene(cam, scene);
    bev = double(transformImage(cam.Bev, prob));
    S = testCase.TestData.V.Search;
end

function p = coldPrior(side)
    p = struct('Fun', @(x) side * 1.75 + 0 * x, 'HalfWidth', 0.9, 'Side', side, 'Cold', true);
end
