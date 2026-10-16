% ==============================================================================
% test_lka_mil_scenarios.m
% Closed-loop acceptance tests of the main LKA controller (pure pursuit) on the
% LKA_MIL_TestBench. Criteria: aBaja rulebook K.2 (all tyres inside the lane,
% no take-over, <= 33 km/h) and UNECE R79 B1 comfort limits (a_y <= 3 m/s^2,
% 0.5 s lateral jerk <= 5 m/s^3), plus the team target max |offset| < 0.3 m.
% ==============================================================================

function tests = test_lka_mil_scenarios
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(projectRoot, 'scripts'));
    testCase.TestData.results = run_lka_mil(Controllers = 1, Plot = false, ...
        OutDir = fullfile(tempdir, 'lka_mil_test'));
end

function testNominalTrack(testCase)
    k = kpiOf(testCase, 'S1_Nominal');
    verifyRulebookAndComfort(testCase, k);
    testCase.verifyLessThan(k.MaxLatOffset_m, 0.3);
    testCase.verifyEqual(k.RulebookScore, k.TimeInLane_s / k.TotalTime_s^2, 'RelTol', 1e-12);
end

function testSpeedLimitTrack(testCase)
    k = kpiOf(testCase, 'S2_SpeedLimit');
    verifyRulebookAndComfort(testCase, k);
    testCase.verifyLessThan(k.MaxLatOffset_m, 0.3);
end

function testTightTrack(testCase)
    k = kpiOf(testCase, 'S3_TightTrack');
    verifyRulebookAndComfort(testCase, k);
end

function testDegradedDetector(testCase)
    k = kpiOf(testCase, 'S4_Degraded');
    % The 1.5 s total outage must raise exactly one take-over request ...
    testCase.verifyEqual(k.TakeOverRequests, 1);
    % ... and the short outages must be bridged by the tracker without leaving the lane
    testCase.verifyEqual(k.InLaneRatio, 1, 'AbsTol', 1e-12);
    testCase.verifyFalse(k.AllWheelsOut);
end

function testInitialOffsetRecovery(testCase)
    r = runOf(testCase, 'S5_InitialOffset');
    gt = r.res.gt;
    late = gt.t > 10;
    testCase.verifyLessThan(max(abs(gt.latOffset(late))), 0.3);
    verifyRulebookAndComfort(testCase, r.kpi);
end

function verifyRulebookAndComfort(testCase, k)
    testCase.verifyEqual(k.InLaneRatio, 1, 'AbsTol', 1e-12, 'A tyre touched a lane line');
    testCase.verifyFalse(k.AllWheelsOut);
    testCase.verifyFalse(k.SpeedDQ);
    testCase.verifyEqual(k.TakeOverRequests, 0);
    testCase.verifyLessThanOrEqual(k.MaxLatAccel_mps2, 3.0);
    testCase.verifyLessThanOrEqual(k.MaxLatJerk05_mps3, 5.0);
end

function k = kpiOf(testCase, name)
    k = runOf(testCase, name).kpi;
end

function r = runOf(testCase, name)
    all = testCase.TestData.results;
    r = all(arrayfun(@(x) strcmp(x.res.Scenario, name), all));
    testCase.assertNumElements(r, 1);
end
