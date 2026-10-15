function [steerFilt, state] = lowpass_steer_filter(steerRaw, state, Ts, cutoffHz, maxRate)
% LOWPASS_STEER_FILTER Applies first-order lowpass filtering and rate limiting to steering commands.
%
% Inputs:
%   steerRaw - Raw target steering angle [rad]
%   state    - Previous filtered angle [rad]
%   Ts       - Sample period [s]
%   cutoffHz - Filter cutoff frequency [Hz]
%   maxRate  - Slew rate limit [rad/s]

    % 1. Rate limiting
    deltaSteer = steerRaw - state;
    maxChange = maxRate * Ts;
    steerRateLimited = state + max(min(deltaSteer, maxChange), -maxChange);

    % 2. Discrete first-order lowpass filter: alpha = dt / (RC + dt)
    RC = 1.0 / (2 * pi * cutoffHz);
    alpha = Ts / (RC + Ts);
    steerFilt = alpha * steerRateLimited + (1 - alpha) * state;

    % Update state
    state = steerFilt;

end
