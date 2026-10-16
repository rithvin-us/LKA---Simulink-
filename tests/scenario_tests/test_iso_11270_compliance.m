% ==============================================================================
% test_iso_11270_compliance.m
% Scenario test validating LKA performance against ISO 11270 standards.
% ==============================================================================

function tests = test_iso_11270_compliance
    tests = functiontests(localfunctions);
end

function testConstantRadiusCurve(testCase)
    % Initialize environment
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    vehicle = evalin('base', 'vehicle');
    controller = evalin('base', 'controller');

    scenarios = generate_test_scenarios();
    track = scenarios.constant_curve;

    dt = 0.01;
    t_vec = 0:dt:15.0;
    N = length(t_vec);

    X = zeros(1, N); Y = zeros(1, N); psi = zeros(1, N);
    Vy = zeros(1, N); r = zeros(1, N); lat_accel = zeros(1, N);
    e_fa_arr = zeros(1, N); delta_arr = zeros(1, N);

    X(1) = track.X(1); Y(1) = track.Y(1); psi(1) = track.yaw(1);
    Vx = vehicle.nominalSpeed;
    filterState = 0.0;

    for k = 1:(N-1)
        vehState.X = X(k); vehState.Y = Y(k); vehState.yawAngle = psi(k);
        vehState.Vx = Vx; vehState.Vy = Vy(k); vehState.yawRate = r(k);

        vehPose = [X(k), Y(k), psi(k)];
        [~, ~, nearIdx, ~] = compute_lateral_error(vehPose, track);

        pathPt.x = track.X(nearIdx); pathPt.y = track.Y(nearIdx);
        pathPt.yaw = track.yaw(nearIdx); pathPt.wheelbase_a = vehicle.a;
        pathPt.curvature = track.curvature(nearIdx); pathPt.wheelbase = vehicle.a + vehicle.b;

        [deltaRaw, e_fa, ~] = stanley_steer_control(vehState, pathPt, controller);
        [deltaCmd, filterState] = lowpass_steer_filter(...
            deltaRaw, filterState, dt, controller.filterCutoffHz, controller.maxSteerRate);

        e_fa_arr(k) = e_fa;
        delta_arr(k) = deltaCmd;

        alpha_f = deltaCmd - (Vy(k) + vehicle.a * r(k)) / Vx;
        alpha_r = - (Vy(k) - vehicle.b * r(k)) / Vx;
        Fy_f = vehicle.Cf * alpha_f;
        Fy_r = vehicle.Cr * alpha_r;

        dVy = (Fy_f + Fy_r) / vehicle.mass - Vx * r(k);
        dr  = (vehicle.a * Fy_f - vehicle.b * Fy_r) / vehicle.Iz;

        lat_accel(k) = dVy + Vx * r(k);

        X(k+1)   = X(k)   + (Vx * cos(psi(k)) - Vy(k) * sin(psi(k))) * dt;
        Y(k+1)   = Y(k)   + (Vx * sin(psi(k)) + Vy(k) * cos(psi(k))) * dt;
        psi(k+1) = psi(k) + r(k) * dt;
        Vy(k+1)  = Vy(k)  + dVy * dt;
        r(k+1)   = r(k)   + dr * dt;
    end

    % Evaluate ISO limits after initial settling transient (t > 3 sec)
    idxSteady = round(3.0 / dt):N;
    maxSteadyError = max(abs(e_fa_arr(idxSteady)));
    maxAcceleration = max(abs(lat_accel));

    testCase.verifyLessThanOrEqual(maxSteadyError, 0.35, 'Steady-state error exceeds ISO 11270 limit');
    testCase.verifyLessThanOrEqual(maxAcceleration, 3.0, 'Lateral acceleration exceeds 3.0 m/s^2 comfort/safety limit');
end
