function [validL, validR, widthMeas, widthValid] = lka_validate_boundaries(left, right, prevOffL, prevOffR, prevValidL, prevValidR, Val, Lane)
% LKA_VALIDATE_BOUNDARIES Plausibility gating of the detected lane boundaries.
%
% A boundary is used only if
%   1. the detector strength is high enough,
%   2. it lies on its own side of the vehicle (left boundary left, right right),
%      with SideMargin tolerance so a vehicle touching a line keeps the line,
%   3. heading and curvature are physically plausible,
%   4. its offset did not jump by more than JumpMax since the last accepted frame
%      (rejects mis-detections such as the opposite-lane line or a tar seam).
% When both pass, the implied lane width must lie inside [WidthMin, WidthMax];
% otherwise the weaker boundary is dropped.
%
% Inputs:
%   left, right - LaneBoundaryBus structs from the lane detector
%   prevOffL/R  - lateral offsets accepted in the previous frame [m]
%   prevValidL/R- whether the previous frame accepted that boundary
%   Val, Lane   - LKA.Val and LKA.Lane calibration structs
%
% Outputs:
%   validL, validR - boundary accepted
%   widthMeas      - lane width implied by both boundaries [m] (0 if not both)
%   widthValid     - widthMeas is usable for the lane-width estimator

    validL = boundaryOk(left,  +1, prevOffL, prevValidL, Val);
    validR = boundaryOk(right, -1, prevOffR, prevValidR, Val);

    widthMeas = 0;
    widthValid = false;
    if validL && validR
        psiMean = 0.5 * (left.HeadingAngle + right.HeadingAngle);
        w = (left.LateralOffset - right.LateralOffset) * cos(psiMean);
        if w >= Lane.WidthMin && w <= Lane.WidthMax
            widthMeas = w;
            widthValid = true;
        elseif left.Strength >= right.Strength
            validR = false;
        else
            validL = false;
        end
    end
end

function ok = boundaryOk(b, side, prevOff, prevValid, Val)
    ok = b.Strength >= Val.StrengthMin ...
        && side * b.LateralOffset > -Val.SideMargin ...
        && abs(b.LateralOffset) <= Val.OffsetMax ...
        && abs(b.HeadingAngle) <= Val.HeadingMax ...
        && abs(b.Curvature) <= Val.CurvatureMax;
    if ok && prevValid
        ok = abs(b.LateralOffset - prevOff) <= Val.JumpMax;
    end
end
