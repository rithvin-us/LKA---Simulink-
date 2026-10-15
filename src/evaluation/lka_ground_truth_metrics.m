function [latOffset, headingErr, margins, inLane] = lka_ground_truth_metrics(cgPose, centerPose, curv, width, lineWidth, vehicle)
% LKA_GROUND_TRUTH_METRICS True lane-keeping performance of the vehicle.
%
% Uses the exact lane geometry (not the detector) at the point closest to the CG.
%
% Outputs:
%   latOffset  - CG lateral position relative to the lane centre [m] (+ left)
%   headingErr - vehicle heading relative to the lane [rad] (+ CCW)
%   margins    - [FL FR RL RR] distance from each tyre's outer edge to the inner
%                edge of the painted line on its side [m]; < 0 means the tyre
%                touches/crosses the line (rulebook K.2.4: vehicle outside lane)
%   inLane     - all four tyres inside the lane

    [~, ~, c] = lka_lane_geometry(cgPose, centerPose, curv(1), curv(2), width);
    yc0 = c(1); psiC = c(2); kC = c(3);

    latOffset = -yc0;
    headingErr = -psiC;

    edge = 0.5 * vehicle.trackWidth + 0.5 * vehicle.tireWidth;
    xAxle = [vehicle.a, -vehicle.b];
    margins = zeros(1, 4);
    for k = 1:2
        xk = xAxle(k);
        ycx = yc0 + tan(psiC) * xk + 0.5 * kC * xk^2;
        % line inner edges measured along the vehicle y axis (lane at relative heading psi)
        inner = (0.5 * width - 0.5 * lineWidth) / max(cos(psiC + kC * xk), 0.2);
        margins(2*k - 1) = (ycx + inner) - edge;   % left tyre
        margins(2*k)     = -edge - (ycx - inner);  % right tyre
    end
    inLane = all(margins >= 0);
end
