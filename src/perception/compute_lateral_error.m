function [latError, headingError, nearestIdx, curvature] = compute_lateral_error(vehPose, path)
% COMPUTE_LATERAL_ERROR Calculates orthogonal distance and heading discrepancy to lane centerline.
%
% Inputs:
%   vehPose - [X, Y, Yaw] 1x3 vector of vehicle position and heading [m, m, rad]
%   path    - Struct with .X, .Y, .yaw, .curvature arrays
%
% Outputs:
%   latError     - Signed lateral error [m] (positive = vehicle is to left of centerline)
%   headingError - Angular error relative to path [rad]
%   nearestIdx   - Index of nearest centerline waypoint
%   curvature    - Local road curvature [1/m]

    X = vehPose(1);
    Y = vehPose(2);
    yaw = vehPose(3);

    % Find closest waypoint
    distancesSq = (path.X - X).^2 + (path.Y - Y).^2;
    [~, nearestIdx] = min(distancesSq);

    pathX = path.X(nearestIdx);
    pathY = path.Y(nearestIdx);
    pathYaw = path.yaw(nearestIdx);

    % Lateral cross-track error
    dx = X - pathX;
    dy = Y - pathY;
    latError = -sin(pathYaw) * dx + cos(pathYaw) * dy;

    % Heading error
    headingError = pathYaw - yaw;
    headingError = atan2(sin(headingError), cos(headingError));

    % Road curvature
    curvature = path.curvature(nearestIdx);

end
