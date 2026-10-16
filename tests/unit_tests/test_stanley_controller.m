% ==============================================================================
% test_stanley_controller.m
% Unit test for the Stanley Steering Controller function.
% ==============================================================================

function tests = test_stanley_controller
    tests = functiontests(localfunctions);
end

function setupOnce(~)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
end

function testZeroErrorHeadingStraight(testCase)
    % When vehicle is on path and aligned, commanded steer should be 0
    vehState = struct('X', 0, 'Y', 0, 'yawAngle', 0, 'Vx', 8.0, 'yawRate', 0);
    pathPt   = struct('x', 0, 'y', 0, 'yaw', 0, 'wheelbase_a', 0.75);
    params   = struct('k_stanley', 1.25, 'k_soft', 1.0, 'k_yaw', 1.0, 'maxSteerAngle', deg2rad(25));

    [deltaCmd, e_fa, theta_e] = stanley_steer_control(vehState, pathPt, params);

    testCase.verifyEqual(deltaCmd, 0.0, 'AbsTol', 1e-4);
    testCase.verifyEqual(e_fa, 0.0, 'AbsTol', 1e-4);
    testCase.verifyEqual(theta_e, 0.0, 'AbsTol', 1e-4);
end

function testLeftOffsetCorrection(testCase)
    % When vehicle is left of path (positive e_fa), steering command should steer right (negative)
    vehState = struct('X', 0, 'Y', 0.5, 'yawAngle', 0, 'Vx', 8.0, 'yawRate', 0);
    pathPt   = struct('x', 0, 'y', 0, 'yaw', 0, 'wheelbase_a', 0.75);
    params   = struct('k_stanley', 1.25, 'k_soft', 1.0, 'k_yaw', 1.0, 'maxSteerAngle', deg2rad(25));

    [deltaCmd, e_fa, ~] = stanley_steer_control(vehState, pathPt, params);

    testCase.verifyGreaterThan(e_fa, 0);
    testCase.verifyLessThan(deltaCmd, 0);
end

function testSaturationLimits(testCase)
    % Large error should saturate at maxSteerAngle
    vehState = struct('X', 0, 'Y', 50.0, 'yawAngle', 0, 'Vx', 8.0, 'yawRate', 0);
    pathPt   = struct('x', 0, 'y', 0, 'yaw', 0, 'wheelbase_a', 0.75);
    maxAngle = deg2rad(25.0);
    params   = struct('k_stanley', 1.25, 'k_soft', 1.0, 'k_yaw', 1.0, 'maxSteerAngle', maxAngle);

    [deltaCmd, ~, ~] = stanley_steer_control(vehState, pathPt, params);

    testCase.verifyEqual(abs(deltaCmd), maxAngle, 'AbsTol', 1e-4);
end

function testSteadyCurveNeedsNoOffset(testCase)
    % On a R 50 m left bend, on the path, aligned, turning at the path yaw rate: the
    % command is the curvature feed-forward atan(L/R) and the yaw damping is zero.
    % (Damping the absolute yaw rate used to steer about 9 deg out of the bend.)
    R = 50; V = 8.33; L = 1.45;
    vehState = struct('X', 0, 'Y', 0, 'yawAngle', 0, 'Vx', V, 'yawRate', V / R);
    pathPt   = struct('x', 0.75, 'y', 0.75^2 / (2 * R), 'yaw', 0.75 / R, 'wheelbase_a', 0.75, ...
        'curvature', 1 / R, 'wheelbase', L);
    params   = struct('k_stanley', 1.25, 'k_soft', 1.0, 'k_yaw', 1.0, 'maxSteerAngle', deg2rad(25));

    [deltaCmd, e_fa, theta_e] = stanley_steer_control(vehState, pathPt, params);

    testCase.verifyEqual(e_fa, 0, 'AbsTol', 0.01);          % front axle a^2/(2R) inside the circle
    testCase.verifyEqual(deltaCmd, atan(L / R) + theta_e, 'AbsTol', 0.01);
    testCase.verifyGreaterThan(deltaCmd, 0);
end
