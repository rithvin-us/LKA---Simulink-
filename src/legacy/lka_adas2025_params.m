function [P, state] = lka_adas2025_params()
% LKA_ADAS2025_PARAMS Calibration of the 2025 ADAS_2026 Python steering chain (replica).
%
% Syntax:
%   [P, state] = lka_adas2025_params()
%
% Values copied from ADAS_2026 (read-only reference, see docs/adas2026_audit.md):
%   steering_v2.py:52-60        K_GAIN, K_SOFT, K_YAW_DAMP, MAX_STEERING_DEG, MAX_STEER_RATE_DPS
%   arduino_steering_link.py:199 1 s mean output window
%   lane_detection_node_birdeye.py:108  lane_width_m = 3.0 (track is 3.5 m) -> WidthScale
%   ego_state_estimator.py:29   default_speed_mps = 5.0 (no speed sensor) -> AssumedSpeed
%   arduinocode.ino:160-161     firmware full lock 24 deg

    P.K             = 0.8;          % K_GAIN
    P.KSoft         = 1.0;          % K_SOFT [m/s]
    P.KYawDamp      = 0.15;         % K_YAW_DAMP [deg per deg/s]
    P.MaxSteerDeg   = 30.0;         % MAX_STEERING_DEG
    P.SlewRateDps   = 60.0;         % MAX_STEER_RATE_DPS
    P.FirmwareMaxDeg = 24.0;        % firmware recalcCPD /24
    P.MeanWindow    = 1.0;          % SecondMeanSteering output_period_s (0 = per-frame)
    P.HeadingBug    = true;         % true = sign as coded (steering_v2.py:81)
    P.UseImu        = false;        % NullImuSource: no yaw damping on the 2025 car
    P.WidthScale    = 3.0 / 3.5;    % CTE scaling from the assumed 3.0 m lane
    P.AssumedSpeed  = 5.0;          % speed fed to Stanley [m/s]; NaN = use measured
    P.HeadingScale  = 1.0;          % measured heading / true heading (1 = metric IPM)

    state = struct('Last', NaN, 'Sum', 0, 'N', 0, 'Elapsed', 0, 'Out', 0);
end
