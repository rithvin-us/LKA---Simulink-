% ==============================================================================
% sensor_params.m
% Specifications for Vision/Perception Lane Detection Sensors
%
% References:
%   1. ISO 17361:2017 - Lane Departure Warning Systems (LDWS).
% ==============================================================================

sensor = struct();

% --- Sensor Geometry & Mount ---
sensor.cameraHeight    = 1.10;   % Camera mounting height above ground [m]
sensor.pitchAngleDeg   = 4.5;    % Camera downward pitch angle [deg]
sensor.maxRange        = 25.0;   % Maximum reliable lane marker detection range [m]
sensor.laneWidthStd    = 3.0;    % Standard lane width [m]

% --- Sensor Dynamics & Latency ---
sensor.sampleTime      = 0.04;   % Vision pipeline frame period [s] (25 fps)
sensor.latencyMs       = 40.0;   % End-to-end processing delay [ms]

% --- Synthetic Noise (Zero-mean Gaussian) ---
sensor.lateralNoiseStd = 0.02;   % Standard deviation of lateral position error [m]
sensor.headingNoiseStd = deg2rad(0.5); % Standard deviation of heading angle error [rad]

% --- Driver Warning / TLC Thresholds ---
sensor.tlcWarningThreshold = 0.8; % Time-To-Line-Crossing threshold for LDW warning [s]

% Assign to base workspace
assignin('base', 'sensor', sensor);
