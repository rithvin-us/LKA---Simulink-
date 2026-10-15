function [z, zValid, noiseScale, numBoundaries] = lka_estimate_lane_center(left, right, validL, validR, laneWidth)
% LKA_ESTIMATE_LANE_CENTER Fuses the accepted boundaries into one lane-centre measurement.
%
% Each accepted boundary is moved half a lane width towards the centre (normal
% to the lane, so the vertical shift is w/(2 cos psi)) and its curvature is
% converted to the centreline radius. With both boundaries the two centre
% candidates are averaged, weighted by detection strength (as in the MathWorks
% "Estimate Lane Center" reference design). With one boundary the measurement
% is still produced but its noise is doubled, and the tracker relies more on
% its prediction.
%
% Outputs:
%   z             - [offset; heading; curvature; curvatureDerivative] of the
%                   lane centre at x = 0 (vehicle frame, rear axle)
%   zValid        - at least one boundary was usable
%   noiseScale    - measurement-variance multiplier for the Kalman filter
%   numBoundaries - 0, 1 or 2

    halfW = 0.5 * laneWidth;
    z = zeros(4, 1);
    zValid = false;
    noiseScale = 1;
    numBoundaries = 0;

    zL = zeros(4, 1);
    zR = zeros(4, 1);
    if validL
        zL = [left.LateralOffset - halfW / max(cos(left.HeadingAngle), 0.2); ...
              left.HeadingAngle; ...
              left.Curvature / max(1 + left.Curvature * halfW, 0.1); ...
              left.CurvatureDerivative];
        numBoundaries = numBoundaries + 1;
    end
    if validR
        zR = [right.LateralOffset + halfW / max(cos(right.HeadingAngle), 0.2); ...
              right.HeadingAngle; ...
              right.Curvature / max(1 - right.Curvature * halfW, 0.1); ...
              right.CurvatureDerivative];
        numBoundaries = numBoundaries + 1;
    end

    if validL && validR
        wL = max(left.Strength, 1e-3);
        wR = max(right.Strength, 1e-3);
        z = (wL * zL + wR * zR) / (wL + wR);
        zValid = true;
        noiseScale = 1 / max(min(0.5 * (wL + wR), 1), 0.3);
    elseif validL
        z = zL;
        zValid = true;
        noiseScale = 2 / max(min(left.Strength, 1), 0.3);
    elseif validR
        z = zR;
        zValid = true;
        noiseScale = 2 / max(min(right.Strength, 1), 0.3);
    end
end
