% ==============================================================================
% test_skeleton_integration.m
% aBaja2026.slx with the LKA: both lane-source variants compile; the camera variant
% feeds the controller no ground truth (rule I.2.3).
% ==============================================================================

function tests = test_skeleton_integration
    tests = functiontests(localfunctions);
end

function testCameraVariantCompiles(testCase)
    [ok, msg] = lka_skeleton_compile(1);
    testCase.verifyTrue(ok, msg);
end

function testEmulatedVariantCompiles(testCase)
    [ok, msg] = lka_skeleton_compile(2);
    testCase.verifyTrue(ok, msg);
end

function testStructure(testCase)
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    warning('off', 'all'); restoreW = onCleanup(@() warning('on', 'all'));
    load_system(fullfile(root, 'SoftwareVirtualWorldSimulation-master', 'Models', 'aBaja2026.slx'));
    testCase.addTeardown(@() close_system('aBaja2026', 0));
    lka = 'aBaja2026/LKA (Lane Keeping Assist)';
    testCase.verifyEqual(get_param([lka '/Lane Source'], 'Variant'), 'on');
    testCase.verifyEqual(get_param([lka '/Lane Source/Camera Lane Detector'], 'VariantControl'), 'LKA_LANE_SOURCE == 1');
    testCase.verifyGreaterThan(getSimulinkBlockHandle([lka '/Ego Sensors (wheel speed + IMU)']), 0);
    % the camera image reaches the LKA subsystem
    ph = get_param(lka, 'PortHandles');
    src = get_param(get_param(ph.Inport(5), 'Line'), 'SrcBlockHandle');
    testCase.verifyEqual(getfullname(src), 'aBaja2026/Simulation 3D Camera');
end
