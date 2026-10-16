% ==============================================================================
% run_baseline_simulation.m
% Executes numerical baseline simulation of LKA on an ATV with Stanley Controller
% Follows 2-DOF Bicycle Dynamics (Rajamani 2012).
% ==============================================================================

clear; clc; close all;

fprintf('====================================================\n');
fprintf('  LKA Simulation Runner - SAE / aBAJA Benchmark     \n');
fprintf('====================================================\n');

% 1. Initialize environment
projectRoot = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(projectRoot, 'config', 'startup.m'));

% 2. Load or generate scenario
scenarios = generate_test_scenarios();
track = scenarios.constant_curve; % Select track: straight, constant_curve, or s_curve
fprintf('Selected Scenario: %s\n', track.name);

% 3. Simulation Time Settings
dt = 0.01;            % Integration time step [s]
T_final = 18.0;       % Total duration [s]
t_vec = 0:dt:T_final;
N = length(t_vec);

% 4. Initialize State Vectors
% State vector: [X, Y, psi, Vy, r]
X_sim   = zeros(1, N);
Y_sim   = zeros(1, N);
psi_sim = zeros(1, N);
Vy_sim  = zeros(1, N);
r_sim   = zeros(1, N);
delta_sim = zeros(1, N);
e_fa_sim  = zeros(1, N);
lat_accel = zeros(1, N);

% Initial conditions (offset from start of path)
X_sim(1)   = track.X(1);
Y_sim(1)   = track.Y(1) + track.initialOffset;
psi_sim(1) = track.yaw(1);
Vy_sim(1)  = 0.0;
r_sim(1)   = 0.0;

Vx = vehicle.nominalSpeed; % Constant forward speed [m/s]
steerFilterState = 0.0;

% 5. Main Simulation Loop
fprintf('Running forward simulation...\n');
for k = 1:(N - 1)
    % Current vehicle state
    vehState.X = X_sim(k);
    vehState.Y = Y_sim(k);
    vehState.yawAngle = psi_sim(k);
    vehState.Vx = Vx;
    vehState.Vy = Vy_sim(k);
    vehState.yawRate = r_sim(k);

    % Find closest path point
    vehPose = [vehState.X, vehState.Y, vehState.yawAngle];
    [latErr, headErr, nearIdx, curKappa] = compute_lateral_error(vehPose, track);

    pathPt.x = track.X(nearIdx);
    pathPt.y = track.Y(nearIdx);
    pathPt.yaw = track.yaw(nearIdx);
    pathPt.wheelbase_a = vehicle.a;
    pathPt.curvature = curKappa;
    pathPt.wheelbase = vehicle.a + vehicle.b;

    % Compute Stanley Steering Command
    [deltaRaw, e_fa, theta_e] = stanley_steer_control(vehState, pathPt, controller);

    % Lowpass filter & rate limit actuator
    [deltaCmd, steerFilterState] = lowpass_steer_filter(...
        deltaRaw, steerFilterState, dt, controller.filterCutoffHz, controller.maxSteerRate);

    % Record diagnostics
    delta_sim(k) = deltaCmd;
    e_fa_sim(k)  = e_fa;

    % 2-DOF Lateral Tire Slip Angles (Rajamani 2012)
    alpha_f = deltaCmd - (Vy_sim(k) + vehicle.a * r_sim(k)) / Vx;
    alpha_r = - (Vy_sim(k) - vehicle.b * r_sim(k)) / Vx;

    % Lateral Tire Forces (Linear Model)
    Fy_f = vehicle.Cf * alpha_f;
    Fy_r = vehicle.Cr * alpha_r;

    % Equations of Motion (Bicycle Model)
    % dVy/dt = (Fy_f + Fy_r)/m - Vx * r
    % dr/dt  = (a * Fy_f - b * Fy_r) / Iz
    dVy_dt = (Fy_f + Fy_r) / vehicle.mass - Vx * r_sim(k);
    dr_dt  = (vehicle.a * Fy_f - vehicle.b * Fy_r) / vehicle.Iz;

    % Lateral Acceleration
    lat_accel(k) = dVy_dt + Vx * r_sim(k);

    % Kinematic state integration (Euler forward)
    dX_dt   = Vx * cos(psi_sim(k)) - Vy_sim(k) * sin(psi_sim(k));
    dY_dt   = Vx * sin(psi_sim(k)) + Vy_sim(k) * cos(psi_sim(k));
    dpsi_dt = r_sim(k);

    X_sim(k+1)   = X_sim(k)   + dX_dt * dt;
    Y_sim(k+1)   = Y_sim(k)   + dY_dt * dt;
    psi_sim(k+1) = psi_sim(k) + dpsi_dt * dt;
    Vy_sim(k+1)  = Vy_sim(k)  + dVy_dt * dt;
    r_sim(k+1)   = r_sim(k)   + dr_dt * dt;
end

delta_sim(N) = delta_sim(N-1);
e_fa_sim(N)  = e_fa_sim(N-1);
lat_accel(N) = lat_accel(N-1);

% 6. Plot Results
figure('Name', 'LKA Simulation Results', 'Color', 'w', 'Position', [100, 100, 1000, 750]);

subplot(2, 2, [1, 2]);
plot(track.X, track.Y, 'k--', 'LineWidth', 2, 'DisplayName', 'Lane Centerline'); hold on;
plot(X_sim, Y_sim, 'b-', 'LineWidth', 2, 'DisplayName', 'ATV Trajectory');
grid on; axis equal;
xlabel('Global X [m]'); ylabel('Global Y [m]');
title(['Trajectory: ', track.name]);
legend('Location', 'best');

subplot(2, 2, 3);
plot(t_vec, e_fa_sim, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]'); ylabel('Lateral Error e_{fa} [m]');
title('Front Axle Cross-Track Error');
yline(0.3, 'k--', 'Max Allowable (ISO 11270)');
yline(-0.3, 'k--');

subplot(2, 2, 4);
plot(t_vec, rad2deg(delta_sim), 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]'); ylabel('Steering Angle \delta [deg]');
title('Commanded Steering Angle');

% 7. Evaluate Performance KPIs
kpis = evaluate_kpis(t_vec, e_fa_sim, lat_accel, delta_sim, controller);
