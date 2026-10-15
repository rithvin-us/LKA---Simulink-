function [left, right, center] = lka_lane_geometry(egoPose, centerPose, kappa, dkappa, laneWidth)
% LKA_LANE_GEOMETRY Ego-lane boundaries in the vehicle frame from world geometry.
%
% Core of the emulated lane detector: given where the vehicle is and a point on
% the true lane centreline, returns what an ideal camera-based detector would
% report - both lane boundaries as clothoids in the ISO 8855 vehicle frame with
% the origin at the rear-axle centre.
%
% Syntax:
%   [left, right, center] = lka_lane_geometry(egoPose, centerPose, kappa, dkappa, laneWidth)
%
% Inputs:
%   egoPose    - [x y psi] rear-axle position [m] and heading [rad], world frame
%   centerPose - [xc yc psic] a point on the lane centreline and its tangent heading
%                (normally the point closest to the rear axle)
%   kappa      - centreline curvature at centerPose [1/m] (+ left)
%   dkappa     - centreline curvature derivative dkappa/ds [1/m^2]
%   laneWidth  - lane width [m]
%
% Outputs (vectors [offset; heading; curvature; curvatureDerivative]):
%   left, right - lane boundaries at x = 0 (+ offset = left of the vehicle)
%   center      - lane centreline at x = 0
%
% The centreline is expanded as a clothoid around the given point and moved to
% x = 0 of the vehicle frame (second order in the longitudinal distance).

    psi = egoPose(3);
    dx = centerPose(1) - egoPose(1);
    dy = centerPose(2) - egoPose(2);

    % Centre point expressed in the vehicle frame
    xv =  cos(psi) * dx + sin(psi) * dy;
    yv = -sin(psi) * dx + cos(psi) * dy;
    psiRel = lka_wrap_angle(centerPose(3) - psi);

    % Shift the clothoid from x = xv back to x = 0
    c = cos(psiRel);
    c = max(c, 0.2);                                   % beyond 78 deg the lane is not trackable anyway
    y0     = yv - tan(psiRel) * xv + 0.5 * kappa * xv^2 / c^3;
    psi0   = psiRel - kappa * xv / c;
    kappa0 = kappa - dkappa * xv / c;

    center = [y0; psi0; kappa0; dkappa];

    % Boundaries are parallel curves at +-w/2 measured normal to the centreline
    halfW = 0.5 * laneWidth / max(cos(psi0), 0.2);
    kL = kappa0 / max(1 - kappa0 * 0.5 * laneWidth, 0.1);
    kR = kappa0 / max(1 + kappa0 * 0.5 * laneWidth, 0.1);

    left  = [y0 + halfW; psi0; kL; dkappa];
    right = [y0 - halfW; psi0; kR; dkappa];
end
