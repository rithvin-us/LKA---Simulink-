function deltaCmd = pure_pursuit_steer(vehicleState, lookaheadPoint, wheelbase, controllerParams)
% PURE_PURSUIT_STEER Calculates steering command using the Pure Pursuit geometric method.
%
% Syntax:
%   deltaCmd = pure_pursuit_steer(vehicleState, lookaheadPoint, wheelbase, controllerParams)
%
% Inputs:
%   vehicleState   - Struct with .X, .Y, .yawAngle, .Vx
%   lookaheadPoint - Target point [x, y] ahead on path
%   wheelbase      - Distance between axles L [m]
%   controllerParams - Struct with .maxSteerAngle
%
% Reference:
%   Snider, J. M. (2009). CMU-RI-TR-09-08.

    % Vector from rear axle to lookahead target
    dx = lookaheadPoint(1) - vehicleState.X;
    dy = lookaheadPoint(2) - vehicleState.Y;

    % Angle to target relative to vehicle heading
    targetAngle = atan2(dy, dx);
    alpha = targetAngle - vehicleState.yawAngle;
    alpha = atan2(sin(alpha), cos(alpha)); % Wrap to [-pi, pi]

    % Lookahead distance
    Ld = max(hypot(dx, dy), 0.5);

    % Pure pursuit curvature command: kappa = 2*sin(alpha)/Ld
    deltaRaw = atan2(2 * wheelbase * sin(alpha), Ld);

    % Saturation
    deltaCmd = max(min(deltaRaw, controllerParams.maxSteerAngle), -controllerParams.maxSteerAngle);

end
