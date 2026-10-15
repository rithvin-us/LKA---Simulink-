% ==============================================================================
% lka_controller_params.m
% Calibration and gain parameters for the LKA Steering Controller.
% Supports both Stanley Formulation and Pure Pursuit Path Tracking.
%
% References:
%   1. Snider, J. M. (2009). Automatic Steering Methods for Autonomous
%      Automobile Path Tracking. CMU-RI-TR-09-08.
%   2. ISO 11270:2014 - LKAS Performance requirements & limits.
% ==============================================================================

controller = struct();

% --- Sample Time ---
controller.Ts = 0.02;           % Controller sample time [s] (50 Hz control loop)

% --- Stanley Steering Control Gains ---
controller.k_stanley = 1.25;    % Cross-track error gain (higher = more aggressive recovery)
controller.k_soft    = 1.00;    % Velocity softening factor [m/s] to avoid div by zero
controller.k_yaw     = 1.00;    % Heading error damping gain

% --- Pure Pursuit Controller Parameters ---
controller.lookaheadDist0 = 2.5; % Minimum lookahead distance at low speed [m]
controller.k_lookahead   = 0.35; % Velocity lookahead gain: Ld = Ld0 + k_v * Vx [s]

% --- Safety & Saturation Limits (per ISO 11270) ---
controller.maxLateralAccel = 3.0; % Maximum allowable lateral acceleration [m/s^2]
controller.maxSteerAngle   = deg2rad(25.0); % Steering command saturation [rad]
controller.maxSteerRate    = deg2rad(50.0); % Steering rate limit [rad/s]

% --- Lowpass Filter for Actuator Smoothing ---
controller.filterCutoffHz  = 5.0; % Lowpass cutoff frequency for steering output [Hz]

% Assign to base workspace
assignin('base', 'controller', controller);
