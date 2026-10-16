% ==============================================================================
% test_ego_sensors.m
% Wheel-speed and gyro models that feed LKA_Controller (rule I.2.3).
% ==============================================================================

function tests = test_ego_sensors
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    testCase.TestData.P = LKA.EgoSen;
end

function testNoiseFreeOutputs(testCase)
    P = testCase.TestData.P;
    [v, r] = lka_ego_sensor_model(8.0, 0.1, [0 0], P);
    testCase.verifyEqual(v, round(8.0 * (1 + P.SpeedScaleErr) / P.SpeedQuant) * P.SpeedQuant, 'AbsTol', 1e-12);
    testCase.verifyEqual(r, 0.1 + P.YawRateBias, 'AbsTol', 1e-12);
end

function testSpeedNeverNegative(testCase)
    v = lka_ego_sensor_model(0.0, 0, [-5 0], testCase.TestData.P);
    testCase.verifyEqual(v, 0);
end

function testNoiseScales(testCase)
    P = testCase.TestData.P;
    [~, r1] = lka_ego_sensor_model(5, 0, [0 1], P);
    testCase.verifyEqual(r1 - P.YawRateBias, P.YawRateNoiseStd, 'AbsTol', 1e-12);
end
