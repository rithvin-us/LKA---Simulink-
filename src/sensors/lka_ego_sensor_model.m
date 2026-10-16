function [vMeas, rMeas] = lka_ego_sensor_model(v, r, n, P)
% LKA_EGO_SENSOR_MODEL Wheel-speed sensor and gyro yaw rate as seen by the LKA.
%
% Syntax:
%   [vMeas, rMeas] = lka_ego_sensor_model(v, r, n, P)       P = LKA.EgoSen
%
% vMeas = quantised((1 + SpeedScaleErr)*v + SpeedNoiseStd*n(1)), >= 0      [m/s]
% rMeas = r + YawRateBias + YawRateNoiseStd*n(2)                          [rad/s, + CCW]
% n = [nSpeed nGyro] standard normal samples (Random Number blocks, seeded).
    vMeas = (1 + P.SpeedScaleErr) * v + P.SpeedNoiseStd * n(1);
    vMeas = max(0, round(vMeas / P.SpeedQuant) * P.SpeedQuant);
    rMeas = r + P.YawRateBias + P.YawRateNoiseStd * n(2);
end
