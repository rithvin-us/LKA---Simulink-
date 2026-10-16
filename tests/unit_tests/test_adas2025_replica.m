% ==============================================================================
% test_adas2025_replica.m
% Replica of the ADAS_2026 (2025) Python steering chain: sign of each term and
% closed-loop effect of the heading-sign bug (docs/adas2026_audit.md, C1).
% Frame: ISO 8855, + = left. psiC > 0: lane heads to the left of the vehicle.
% ==============================================================================

function tests = test_adas2025_replica
    tests = functiontests(localfunctions);
end

function setupOnce(~)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
end

function testHeadingTermSignAsCodedIsWrong(testCase)
    [P, st] = lka_adas2025_params();
    P.MeanWindow = 0; P.SlewRateDps = inf;
    P.HeadingBug = true;
    dAs = lka_adas2025_steer(0, deg2rad(5), 8, 0, st, P, 0.05);
    P.HeadingBug = false;
    dFix = lka_adas2025_steer(0, deg2rad(5), 8, 0, st, P, 0.05);
    testCase.verifyLessThan(dAs, 0);        % steers right although the lane turns left
    testCase.verifyGreaterThan(dFix, 0);
end

function testCrossTrackTermSign(testCase)
    % lane centre 0.5 m to the left -> both variants steer left
    [P, st] = lka_adas2025_params();
    P.MeanWindow = 0; P.SlewRateDps = inf;
    for bug = [true false]
        P.HeadingBug = bug;
        testCase.verifyGreaterThan(lka_adas2025_steer(0.5, 0, 8, 0, st, P, 0.05), 0);
    end
end

function testOneSecondMeanDelaysCommand(testCase)
    [P, st] = lka_adas2025_params();
    P.HeadingBug = false; P.MeanWindow = 1.0; P.SlewRateDps = inf;
    out = zeros(1, 25);
    for k = 1:25
        [out(k), st] = lka_adas2025_steer(0, deg2rad(4), 8, 0, st, P, 0.05);
    end
    testCase.verifyEqual(out(1:19), zeros(1, 19));           % held at the last window mean (0)
    testCase.verifyEqual(out(20), deg2rad(4), 'AbsTol', 1e-9); % first window closes after 1.0 s
end

function testClosedLoopHeadingBugIsUnstable(testCase)
    T = verify_adas2025_control(Plot=false);
    asCoded = T(T.Variant == "as_coded", :);
    fixed = T(T.Variant == "fixed", :);
    % If 'fixed' is not 100 % in lane, record it in docs/adas2026_audit.md (the corrected
    % 2025 law is then not competitive either). Do not relax this assertion silently.
    testCase.verifyEqual(fixed.InLaneRatio, 1, 'AbsTol', 1e-9);
    testCase.verifyGreaterThan(asCoded.MaxLatOffset_m, 2 * fixed.MaxLatOffset_m);
end
