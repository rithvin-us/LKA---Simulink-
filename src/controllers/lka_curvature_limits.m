function [kappaMax, kappaRateMax] = lka_curvature_limits(v, Veh, Lim)
% LKA_CURVATURE_LIMITS Speed-dependent limits on commanded path curvature.
%
%   |kappa|     <= min( AyMax / v^2 ,  tan(SteerMax) / (L + K_us v^2) )
%   |dkappa/dt| <= min( JerkMax / v^2, SteerRateMax / (L + K_us v^2) )
%
% The a_y and jerk caps (3 m/s^2, 5 m/s^3) are the UNECE R79 ACSF category B1
% values for lane-keeping systems; the geometric caps follow from the steering
% angle and rate limits through d(kappa)/d(delta) ~ 1/(L + K_us v^2).
% Below Lim.SpeedFloor the a_y/jerk terms use SpeedFloor so the limits stay
% finite at standstill (the steering limits dominate there anyway).

    vEff = max(abs(v), Lim.SpeedFloor);
    Leff = Veh.L + Veh.Kus * v^2;
    kappaMax     = min(Lim.AyMax / vEff^2,   tan(Veh.SteerMax) / Leff);
    kappaRateMax = min(Lim.JerkMax / vEff^2, Veh.SteerRateMax / Leff);
end
