% ==============================================================================
% test_lka_core.m
% Unit tests for the LKA algorithm functions used inside LKA_Controller.slx and
% the emulated lane sensor. Sign convention under test: ISO 8855, + = left.
% ==============================================================================

function tests = test_lka_core
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    testCase.TestData.vehicle = evalin('base', 'vehicle');
    testCase.TestData.sensor  = evalin('base', 'sensor');
    testCase.TestData.LKA = lka_build_params('baja', testCase.TestData.vehicle, testCase.TestData.sensor);
end

% ------------------------------------------------------------------ geometry
function testGeometryCentredStraight(testCase)
    [L, R, C] = lka_lane_geometry([0 0 0], [0 0 0], 0, 0, 3.5);
    testCase.verifyEqual(L(1),  1.75, 'AbsTol', 1e-12);
    testCase.verifyEqual(R(1), -1.75, 'AbsTol', 1e-12);
    testCase.verifyEqual(C, zeros(4,1), 'AbsTol', 1e-12);
end

function testGeometryVehicleRightOfCentre(testCase)
    % Vehicle 0.5 m right of the centre -> lane centre is 0.5 m to its left
    [~, ~, C] = lka_lane_geometry([0 -0.5 0], [0 0 0], 0, 0, 3.5);
    testCase.verifyEqual(C(1), 0.5, 'AbsTol', 1e-12);
end

function testGeometryHeadingAndShift(testCase)
    % Vehicle yawed 5 deg right of a straight lane: lane heading relative = +5 deg
    [~, ~, C] = lka_lane_geometry([0 0 -deg2rad(5)], [0 0 0], 0, 0, 3.5);
    testCase.verifyEqual(C(2), deg2rad(5), 'AbsTol', 1e-12);
    % Centre point 2 m ahead on a curve: offset at x=0 recovers the curve shape
    k = 0.01;
    [~, ~, C2] = lka_lane_geometry([0 0 0], [2 0.5*k*4 2*k], k, 0, 3.5);
    testCase.verifyEqual(C2(1), 0, 'AbsTol', 2e-4);
    testCase.verifyEqual(C2(2), 0, 'AbsTol', 1e-4);
end

% ---------------------------------------------------------------- validation
function testValidationAcceptsGoodPair(testCase)
    P = testCase.TestData.LKA;
    [l, r] = boundaries(1.75, -1.75, 0.9, 0.9);
    [vL, vR, w, wv] = lka_validate_boundaries(l, r, 1.75, -1.75, true, true, P.Val, P.Lane);
    testCase.verifyTrue(vL && vR && wv);
    testCase.verifyEqual(w, 3.5, 'AbsTol', 1e-12);
end

function testValidationRejectsWeakJumpAndWrongSide(testCase)
    P = testCase.TestData.LKA;
    [l, r] = boundaries(1.75, -1.75, 0.1, 0.9);              % weak left
    [vL, vR] = lka_validate_boundaries(l, r, 0, 0, false, false, P.Val, P.Lane);
    testCase.verifyFalse(vL); testCase.verifyTrue(vR);
    [l, r] = boundaries(1.75, -1.75, 0.9, 0.9);              % right jumped 1 m
    [~, vR] = lka_validate_boundaries(l, r, 1.75, -0.75, true, true, P.Val, P.Lane);
    testCase.verifyFalse(vR);
    [l, r] = boundaries(-1.0, -1.75, 0.9, 0.9);              % left line on the right side
    vL = lka_validate_boundaries(l, r, 0, 0, false, false, P.Val, P.Lane);
    testCase.verifyFalse(vL);
end

function testValidationImplausibleWidthDropsWeaker(testCase)
    P = testCase.TestData.LKA;
    [l, r] = boundaries(4.0, -3.0, 0.5, 0.9);                % 7 m wide
    [vL, vR, ~, wv] = lka_validate_boundaries(l, r, 0, 0, false, false, P.Val, P.Lane);
    testCase.verifyFalse(vL); testCase.verifyTrue(vR); testCase.verifyFalse(wv);
end

% ------------------------------------------------------------ centre fusion
function testCentreFusionBothAndSingle(testCase)
    [l, r] = boundaries(2.05, -1.45, 0.9, 0.9);              % centre 0.3 m left
    [z, ok, ns, n] = lka_estimate_lane_center(l, r, true, true, 3.5);
    testCase.verifyTrue(ok); testCase.verifyEqual(n, 2);
    testCase.verifyEqual(z(1), 0.3, 'AbsTol', 1e-12);
    testCase.verifyLessThan(ns, 1.2);
    [z1, ok1, ns1, n1] = lka_estimate_lane_center(l, r, false, true, 3.5);
    testCase.verifyTrue(ok1); testCase.verifyEqual(n1, 1);
    testCase.verifyEqual(z1(1), 0.3, 'AbsTol', 1e-12);
    testCase.verifyGreaterThan(ns1, ns);
    [~, ok0] = lka_estimate_lane_center(l, r, false, false, 3.5);
    testCase.verifyFalse(ok0);
end

function testCentreCurvatureFromEachSide(testCase)
    k = 1/127.48; w = 3.5;
    [Lb, Rb] = lka_lane_geometry([0 0 0], [0 0 0], k, 0, w);
    l = toBoundary(Lb, 0.9); r = toBoundary(Rb, 0.9);
    zL = lka_estimate_lane_center(l, r, true, false, w);
    zR = lka_estimate_lane_center(l, r, false, true, w);
    testCase.verifyEqual(zL(3), k, 'RelTol', 1e-9);
    testCase.verifyEqual(zR(3), k, 'RelTol', 1e-9);
end

% ------------------------------------------------------------------ tracker
function testTrackerConvergesAndCoasts(testCase)
    P = testCase.TestData.LKA; Ts = P.Ts;
    x = zeros(4,1); Pm = eye(4); st = zeros(3,1);
    z = [0.4; 0.02; 0.005; 0];
    for k = 1:40
        [x, Pm, st, q] = lka_lane_tracker_step(x, Pm, st, z, true, 1, 0, 0, Ts, P.KF);
    end
    testCase.verifyEqual(x, z, 'AbsTol', 1e-3);
    testCase.verifyGreaterThan(q, 0.85);
    % Coast 0.5 s at 8 m/s on a constant curve while yawing with it: lane stays put
    v = 8; kap = 1/127.48;
    x = [0; 0; kap; 0]; Pm = diag(P.KF.SigmaMeas.^2); st = [1; 0; 0];
    for k = 1:10
        [x, Pm, st, q2] = lka_lane_tracker_step(x, Pm, st, zeros(4,1), false, 1, v, v*kap, Ts, P.KF);
    end
    testCase.verifyEqual(x(1), 0, 'AbsTol', 1e-3);
    testCase.verifyEqual(x(2), 0, 'AbsTol', 1e-6);
    testCase.verifyEqual(st(2), 0.5, 'AbsTol', 1e-9);
    testCase.verifyLessThan(q2, q);
end

function testTrackerGateRejectsThenReinitialises(testCase)
    P = testCase.TestData.LKA; Ts = P.Ts;
    x = zeros(4,1); Pm = diag(P.KF.SigmaMeas.^2); st = [1; 0; 0];
    zJump = [1.5; 0; 0; 0];
    [x1, ~, st1] = lka_lane_tracker_step(x, Pm, st, zJump, true, 1, 5, 0, Ts, P.KF);
    testCase.verifyLessThan(abs(x1(1)), 0.1);                 % outlier rejected
    testCase.verifyEqual(st1(3), 1);
    for k = 1:P.KF.MaxRejects
        [x, Pm, st] = lka_lane_tracker_step(x, Pm, st, zJump, true, 1, 0, 0, Ts, P.KF);
    end
    testCase.verifyEqual(x(1), 1.5, 'AbsTol', 1e-9);           % persistent -> re-initialised
end

% ------------------------------------------------------------- pure pursuit
function testPurePursuitStraightAndCurve(testCase)
    PP = testCase.TestData.LKA.PP;
    est = estimate(0, 0, 0);
    testCase.verifyEqual(lka_pure_pursuit(est, 6, PP), 0, 'AbsTol', 1e-15);
    k = 1/127.48;
    [kpp, Ld, ~, kff] = lka_pure_pursuit(estimate(0, 0, k), 6, PP);
    testCase.verifyEqual(kpp, k, 'RelTol', 0.01);             % feed-forward reproduces the road
    testCase.verifyEqual(kff, k);
    testCase.verifyLessThan(Ld, 6);                           % shortened in the curve
    testCase.verifyGreaterThan(lka_pure_pursuit(estimate(0.3, 0, 0), 6, PP), 0);   % centre left -> turn left
    testCase.verifyLessThan(lka_pure_pursuit(estimate(0, -0.05, 0), 6, PP), 0);    % lane heads right -> turn right
end

function testPurePursuitSmallAngleDecomposition(testCase)
    PP = testCase.TestData.LKA.PP; PP.LdCurvGain = 0;
    y = 0.05; psi = 0.004; k = 0.002; Ld = 6;
    kpp = lka_pure_pursuit(estimate(y, psi, k), Ld, PP);
    testCase.verifyEqual(kpp, k + 2*psi/Ld + 2*y/Ld^2, 'RelTol', 1e-3);
end

% ----------------------------------------------------- steering geometry
function testSteerCurvatureRoundTrip(testCase)
    Veh = testCase.TestData.LKA.Veh;
    for k = [-0.1 -0.01 0 0.0078 0.05]
        d = lka_curvature_to_steer(k, 8.33, Veh);
        testCase.verifyEqual(lka_steer_to_curvature(d, 8.33, Veh), k, 'AbsTol', 1e-12);
    end
    testCase.verifyEqual(lka_curvature_to_steer(0.1, 0, Veh), atan(Veh.L*0.1), 'AbsTol', 1e-12);
end

function testCurvatureLimits(testCase)
    P = testCase.TestData.LKA;
    [kMax, kRate] = lka_curvature_limits(30/3.6, P.Veh, P.Lim);
    testCase.verifyEqual(kMax, 3.0 / (30/3.6)^2, 'RelTol', 1e-12);
    testCase.verifyEqual(kRate, 5.0 / (30/3.6)^2, 'RelTol', 1e-12);
    kMax0 = lka_curvature_limits(0, P.Veh, P.Lim);
    testCase.verifyLessThanOrEqual(kMax0, tan(P.Veh.SteerMax) / P.Veh.L);
end

% ---------------------------------------------------- Stanley 2025 baseline
function testStanley2025Signs(testCase)
    P = testCase.TestData.LKA; st0 = zeros(6,1);
    d0 = lka_stanley_2025(estimate(0, 0, 0), 8, true, st0, P.Ts, P.St);
    testCase.verifyEqual(d0, 0, 'AbsTol', 1e-15);
    st = st0; d = 0;
    for k = 1:60
        [d, st] = lka_stanley_2025(estimate(0.5, 0, 0), 8, true, st, P.Ts, P.St);
    end
    testCase.verifyGreaterThan(d, 0);                          % centre left -> steer left
    testCase.verifyLessThanOrEqual(abs(d), P.St.SteerLimit + 1e-12);
    [d1, st1] = lka_stanley_2025(estimate(5, 0, 0), 8, true, st0, P.Ts, P.St);
    testCase.verifyLessThanOrEqual(abs(st1(4)), P.St.RateLimit * P.Ts + 1e-12);   % rate limited
    testCase.verifyGreaterThan(d1, 0);
end

% ---------------------------------------------------------------- MPC
function testMpcSteadyStateAndSigns(testCase)
    P = testCase.TestData.LKA; v = 30/3.6; iA0 = false(4*P.Mpc.Horizon, 1);
    mpcStep = @(est, kPrev) lka_mpc_curvature(est, v, 0, kPrev, kPrev, iA0, P.Mpc, P.Veh, P.Lim, P.Ts);
    testCase.verifyEqual(mpcStep(estimate(0, 0, 0), 0), 0, 'AbsTol', 1e-9);
    k = 1/127.48;                                             % centred on a curve, already turning
    testCase.verifyEqual(mpcStep(estimate(0, 0, k), k), k, 'AbsTol', 2e-5);
    testCase.verifyGreaterThan(mpcStep(estimate(0.3, 0, 0), 0), 0);   % centre left  -> turn left
    testCase.verifyLessThan(mpcStep(estimate(-0.3, 0, 0), 0), 0);     % centre right -> turn right
    testCase.verifyLessThan(mpcStep(estimate(0, -0.05, 0), 0), 0);     % lane heads right -> turn right
end

function testMpcRespectsEnvelope(testCase)
    P = testCase.TestData.LKA; v = 30/3.6; iA0 = false(4*P.Mpc.Horizon, 1);
    [kMax, kRate] = lka_curvature_limits(v, P.Veh, P.Lim);
    kPrev = 0.01;
    [kc, iA, info] = lka_mpc_curvature(estimate(1.5, 0.2, 0), v, 0, kPrev, kPrev, iA0, P.Mpc, P.Veh, P.Lim, P.Ts);
    testCase.verifyGreaterThan(info(1), 0);                   % QP solved
    testCase.verifyLessThanOrEqual(abs(kc), kMax + 1e-9);
    testCase.verifyLessThanOrEqual(abs(kc - kPrev), kRate * P.Ts + 1e-9);
    testCase.verifyTrue(any(iA));                             % rate constraint active
end

% --------------------------------------------------- lane departure warning
function testLaneDepartureMarginsAndWarning(testCase)
    P = testCase.TestData.LKA;
    est = estimate(0, 0, 0); est.LaneWidth = 3.5;
    [wl, wr, dl, dr, tl, tr] = lka_lane_departure(est, 8, 0, 1, P.Veh, P.Lane, P.Ldw);
    expected = 1.75 - 0.0625 - (0.65 + 0.10);
    testCase.verifyEqual([dl dr], [expected expected], 'AbsTol', 1e-12);
    testCase.verifyFalse(wl || wr);
    testCase.verifyEqual([tl tr], [P.Ldw.TlcMax P.Ldw.TlcMax]);
    % Lane heading 3 deg right relative to vehicle = vehicle drifting left
    est2 = estimate(-0.6, -deg2rad(3), 0); est2.LaneWidth = 3.5;
    [wl2, wr2, ~, ~, tl2] = lka_lane_departure(est2, 8, 0, 1, P.Veh, P.Lane, P.Ldw);
    testCase.verifyTrue(wl2); testCase.verifyFalse(wr2);
    testCase.verifyLessThan(tl2, 1.0);
end

% ---------------------------------------------------------------- track
function testOfficialTrackReader(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    road = read_xodr_lane(fullfile(projectRoot, 'data', 'scenarios', 'tracks', 'LKA_IPG.xodr'), 1, 0.25);
    testCase.verifyEqual(road.length, 1002.3, 'AbsTol', 0.5);
    testCase.verifyEqual([road.x(1) road.y(1)], [-500 1.75], 'AbsTol', 1e-9);
    testCase.verifyEqual(road.width(1), 3.5);
    % s ~ 100 m lies in the left arc (reference curvature +0.0078446, lane 1.75 m left of it)
    kExpected = 7.8446454e-3 / (1 - 7.8446454e-3 * 1.75);
    testCase.verifyEqual(interp1(road.s, road.kappa, 100), kExpected, 'RelTol', 1e-3);
    % Right arc around s ~ 50 m
    testCase.verifyLessThan(interp1(road.s, road.kappa, 50), 0);
end

% ---------------------------------------------------------------- helpers
function [l, r] = boundaries(offL, offR, sL, sR)
    l = toBoundary([offL; 0; 0; 0], sL);
    r = toBoundary([offR; 0; 0; 0], sR);
end

function b = toBoundary(v, strength)
    b = struct('LateralOffset', v(1), 'HeadingAngle', v(2), 'Curvature', v(3), ...
        'CurvatureDerivative', v(4), 'Strength', strength, 'ViewRange', 25);
end

function est = estimate(y, psi, k)
    est = struct('CenterOffset', y, 'HeadingAngle', psi, 'Curvature', k, ...
        'CurvatureDerivative', 0, 'LaneWidth', 3.5, 'Quality', 1, 'CoastTime', 0, ...
        'NumBoundaries', 2, 'MeasValid', true);
end
