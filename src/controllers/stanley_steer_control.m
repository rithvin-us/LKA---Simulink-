function [deltaCmd, e_fa, theta_e] = stanley_steer_control(vehicleState, pathPoint, controllerParams)
% STANLEY_STEER_CONTROL Calculates steering angle using the Stanley control law.
%
% Syntax:
%   [deltaCmd, e_fa, theta_e] = stanley_steer_control(vehicleState, pathPoint, controllerParams)
%
% Inputs:
%   vehicleState     - Struct with fields:
%                      .X        : Global X of C.G. [m]
%                      .Y        : Global Y of C.G. [m]
%                      .yawAngle : Heading angle psi [rad]
%                      .Vx       : Longitudinal speed [m/s]
%                      .yawRate  : Yaw rate r [rad/s]
%   pathPoint        - Struct with target waypoint info:
%                      .x        : Reference path X [m]
%                      .y        : Reference path Y [m]
%                      .yaw      : Reference path tangent angle [rad]
%                      .wheelbase_a : Distance from C.G. to front axle [m]
%                      .curvature   : (optional) path curvature at the point [1/m], + left
%                      .wheelbase   : (optional) wheelbase L [m] for the curvature feed-forward
%                      With .curvature the yaw damping acts on r - Vx*kappa (the yaw rate the
%                      path itself demands) and atan(L*kappa) is fed forward, as in Hoffmann
%                      et al. (2007). Without it (straight path) the law is unchanged. Damping
%                      the absolute yaw rate steered ~10 deg out of a R 50 m bend at 30 km/h,
%                      which the cross-track term could only cancel with ~1.5 m of offset.
%   controllerParams - Struct with control gains:
%                      .k_stanley: Cross-track gain
%                      .k_soft   : Speed softening constant [m/s]
%                      .k_yaw    : Yaw damping gain
%                      .maxSteerAngle : Max allowable steer [rad]
%
% Outputs:
%   deltaCmd - Commanded front wheel steering angle [rad]
%   e_fa     - Signed front-axle cross-track error [m]
%   theta_e  - Heading error [rad]
%
% Reference:
%   Snider, J. M. (2009). Automatic Steering Methods for Autonomous
%   Automobile Path Tracking. CMU-RI-TR-09-08.

    % 1. Compute front axle location
    a = pathPoint.wheelbase_a;
    fx = vehicleState.X + a * cos(vehicleState.yawAngle);
    fy = vehicleState.Y + a * sin(vehicleState.yawAngle);

    % 2. Vector from reference waypoint to front axle
    dx = fx - pathPoint.x;
    dy = fy - pathPoint.y;

    % 3. Cross-track error (signed distance orthogonal to path heading)
    % Positive when vehicle is to the left of the desired path
    e_fa = -sin(pathPoint.yaw) * dx + cos(pathPoint.yaw) * dy;

    % 4. Heading error normalized to [-pi, pi]
    theta_e = pathPoint.yaw - vehicleState.yawAngle;
    theta_e = atan2(sin(theta_e), cos(theta_e));

    % 5. Stanley cross-track correction term
    Vx = max(vehicleState.Vx, 0.1); % Prevent non-positive speeds
    crossTrackTerm = atan2(controllerParams.k_stanley * (-e_fa), Vx + controllerParams.k_soft);

    % 6. Raw commanded steering angle with optional yaw rate damping
    kappa = 0;
    if isfield(pathPoint, 'curvature')
        kappa = pathPoint.curvature;
    end
    feedForward = 0;
    if isfield(pathPoint, 'wheelbase')
        feedForward = atan(pathPoint.wheelbase * kappa);
    end
    yawDamping = -controllerParams.k_yaw * (vehicleState.yawRate - Vx * kappa);
    deltaRaw = theta_e + crossTrackTerm + yawDamping + feedForward;

    % 7. Saturation to mechanical limits
    deltaCmd = max(min(deltaRaw, controllerParams.maxSteerAngle), -controllerParams.maxSteerAngle);

end
