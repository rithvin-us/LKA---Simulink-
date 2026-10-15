function [x, F] = lka_lane_predict(x, v, yawRate, dt)
% LKA_LANE_PREDICT Moves the lane-centre state forward by dt of ego motion.
%
% Same motion model as the Kalman filter (lka_lane_tracker_step): the rear axle
% travels d = v*dt along its heading while yawing by yawRate*dt, so the lane
% clothoid seen from the vehicle shifts by d and rotates by -yawRate*dt.
% Used for the filter prediction and for latency compensation: detector frames
% describe the lane as it was LatencySteps frames ago.
%
% Inputs:  x = [y; psi; kappa; dkappa], v [m/s], yawRate [rad/s], dt [s]
% Outputs: x predicted state, F state-transition matrix

    d = max(v, 0) * dt;
    F = [1 d d^2/2 d^3/6;
         0 1 d     d^2/2;
         0 0 1     d;
         0 0 0     1];
    x = F * x + [-yawRate * dt * d / 2; -yawRate * dt; 0; 0];
end
