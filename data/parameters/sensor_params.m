% ==============================================================================
% sensor_params.m
% Specifications for Vision/Perception Lane Detection Sensors
%
% The emulated lane detector (models/VirtualLaneSensor.slx) reports the left and
% right lane boundaries as clothoids in the ISO 8855 vehicle frame (x forward,
% y left) referenced to the rear-axle centre, at a fixed frame rate with a fixed
% processing latency, Gaussian measurement noise and random frame dropouts.
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
sensor.sampleTime      = 0.05;   % Vision pipeline frame period [s] (20 Hz)
sensor.latencyMs       = 50.0;   % End-to-end processing delay [ms]

% --- Synthetic Noise (Zero-mean Gaussian) ---
sensor.lateralNoiseStd = 0.03;   % Standard deviation of lateral position error [m]
sensor.headingNoiseStd = deg2rad(0.3); % Standard deviation of heading angle error [rad]
sensor.curvatureNoiseStd = 4.0e-4;     % Standard deviation of curvature error [1/m]
sensor.curvDerivNoiseStd = 2.0e-5;     % Standard deviation of curvature-rate error [1/m^2]

% --- Detection Quality ---
sensor.strengthNominal = 0.90;   % Reported detection strength when a boundary is seen [0..1]
sensor.dropoutProb     = 0.01;   % Probability that a boundary is missed in one frame [-]
sensor.noiseSeed       = 2026;   % Base seed for the noise generators

% --- Driver Warning / TLC Thresholds ---
sensor.tlcWarningThreshold = 0.8; % Time-To-Line-Crossing threshold for LDW warning [s]

% Assign to base workspace
assignin('base', 'sensor', sensor);
