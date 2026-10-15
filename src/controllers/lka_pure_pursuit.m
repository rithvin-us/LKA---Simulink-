function [kappaPP, Ld, yTarget, kappaFF] = lka_pure_pursuit(est, LdBase, PP)
% LKA_PURE_PURSUIT Pure pursuit on the tracked lane centre (curvature output).
%
% The look-ahead point is placed on the estimated lane centre (a clothoid in
% the vehicle frame) at distance Ld ahead of the rear axle:
%
%   yT = y + psi*Ld + kappa*Ld^2/2 + dkappa*Ld^3/6
%
% Pure pursuit (Coulter 1992) fits the circular arc through the rear axle,
% tangent to the vehicle heading, that hits the target point (Ld, yT):
%
%   kappaPP = 2*yT / (Ld^2 + yT^2)          turning radius R = 1/kappaPP
%
% For small yT this is kappaPP ~ kappa + 2*psi/Ld + 2*y/Ld^2 + dkappa*Ld/3:
% road-curvature feed-forward plus a PD law on lane offset and heading whose
% gains are set by the single tuning knob Ld. Ld comes from a speed schedule
% (LdBase) and is shortened in tight curves to limit corner cutting.
%
% Inputs:
%   est    - LaneEstimateBus struct (lane centre, rear-axle origin)
%   LdBase - speed-scheduled look-ahead from the lookup table [m]
%   PP     - LKA.PP calibration
% Outputs:
%   kappaPP - commanded path curvature [1/m] (+ left)
%   Ld      - look-ahead distance used [m]
%   yTarget - lane-centre offset at the look-ahead point [m]
%   kappaFF - road-curvature feed-forward part (lane curvature) [1/m]

    Ld = LdBase / (1 + PP.LdCurvGain * abs(est.Curvature) * LdBase);
    Ld = min(max(Ld, PP.LdMin), PP.LdMax);

    yTarget = est.CenterOffset + est.HeadingAngle * Ld ...
            + 0.5 * est.Curvature * Ld^2 + est.CurvatureDerivative * Ld^3 / 6;

    kappaPP = 2 * yTarget / (Ld^2 + yTarget^2);
    kappaFF = est.Curvature;
end
