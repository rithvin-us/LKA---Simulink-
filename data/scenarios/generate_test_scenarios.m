% ==============================================================================
% generate_test_scenarios.m
% Generates reference road centerline waypoints and scenarios for LKA testing.
% Scenarios comply with ISO 11270 test tracks.
% ==============================================================================

function scenarios = generate_test_scenarios()

    scenarios = struct();
    ds = 0.5; % Waypoint discretization distance [m]

    % --- 1. Straight Track with Lateral Offset ---
    % 200 m straight line along global X
    s_straight = 0:ds:200;
    scenarios.straight.X = s_straight;
    scenarios.straight.Y = zeros(size(s_straight));
    scenarios.straight.yaw = zeros(size(s_straight));
    scenarios.straight.curvature = zeros(size(s_straight));
    scenarios.straight.initialOffset = 0.8; % Initial lateral error [m]
    scenarios.straight.name = 'Straight Track (0.8m Initial Departure)';

    % --- 2. Constant Radius Curve (R = 50 m) ---
    % 50 m straight approach followed by a 90-degree curve of radius 50 m
    s_approach = 0:ds:50;
    X_app = s_approach;
    Y_app = zeros(size(s_approach));
    yaw_app = zeros(size(s_approach));
    curv_app = zeros(size(s_approach));

    R = 50.0;
    dtheta = ds / R;
    theta = 0:dtheta:(pi/2);
    X_curve = 50 + R * sin(theta);
    Y_curve = R - R * cos(theta);
    yaw_curve = theta;
    curv_curve = (1/R) * ones(size(theta));

    scenarios.constant_curve.X = [X_app, X_curve(2:end)];
    scenarios.constant_curve.Y = [Y_app, Y_curve(2:end)];
    scenarios.constant_curve.yaw = [yaw_app, yaw_curve(2:end)];
    scenarios.constant_curve.curvature = [curv_app, curv_curve(2:end)];
    scenarios.constant_curve.initialOffset = 0.0;
    scenarios.constant_curve.name = 'Constant Radius Curve (R = 50m)';

    % --- 3. S-Curve (Chicane) ---
    s_sCurve = 0:ds:150;
    scenarios.s_curve.X = s_sCurve;
    scenarios.s_curve.Y = 2.0 * sin((2*pi / 60) * s_sCurve); % 2m amplitude sine wave
    scenarios.s_curve.yaw = atan(2.0 * (2*pi/60) * cos((2*pi / 60) * s_sCurve));
    scenarios.s_curve.curvature = gradient(scenarios.s_curve.yaw) ./ ds;
    scenarios.s_curve.initialOffset = 0.0;
    scenarios.s_curve.name = 'Continuous S-Curve (Chicane)';

    % Save scenario bundle to data/scenarios directory
    saveDir = fileparts(mfilename('fullpath'));
    save(fullfile(saveDir, 'lka_test_scenarios.mat'), 'scenarios');
    assignin('base', 'scenarios', scenarios);
    fprintf('[LKA Scenarios] Generated standard ISO 11270 test scenarios in %s\n', saveDir);

end
