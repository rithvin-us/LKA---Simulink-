function [ldwL, ldwR, dlcL, dlcR, tlcL, tlcR] = lka_lane_departure(est, v, yawRate, quality, Veh, Lane, Ldw)
% LKA_LANE_DEPARTURE Distance / time to line crossing and lane departure warning.
%
% The tracked lane centre y(x) = y0 + psi*x + kappa*x^2/2 is evaluated at the
% rear axle (x = 0) and the front axle (x = L). At each axle the outer edge of
% each tyre is compared with the inner edge of the painted line, which is the
% aBaja rulebook criterion (K.2.4: a wheel touching the line is outside the lane).
%
%   DLC_left  = (y_c + w/2 - line/2) - (track/2 + tyre/2)
%   DLC_right = -(track/2 + tyre/2) - (y_c - w/2 + line/2)
%
% The smaller of the two axles is reported. The lateral rate of the lane centre
% at the axle, d(y_c)/dt = v*psi + x*(v*kappa - yawRate), gives the time to line
% crossing TLC = DLC / closing speed (ISO 17361 style). A warning is raised when
% DLC < Ldw.DlcWarn or TLC < Ldw.TlcWarn, above Ldw.MinSpeed and with a usable
% lane estimate.
%
% Inputs:
%   est     - LaneEstimateBus struct
%   v       - speed [m/s]; yawRate [rad/s]; quality - estimate quality 0..1

    w = est.LaneWidth;
    edge = 0.5 * Veh.Track + 0.5 * Veh.TireWidth;
    halfLine = 0.5 * Lane.LineWidth;

    xs = [0, Veh.L];
    dlcL = inf;  dlcR = inf;
    rateAtMinL = 0; rateAtMinR = 0;
    for k = 1:2
        xk = xs(k);
        yc = est.CenterOffset + est.HeadingAngle * xk + 0.5 * est.Curvature * xk^2;
        ycDot = v * est.HeadingAngle + xk * (v * est.Curvature - yawRate);
        c = max(cos(est.HeadingAngle + est.Curvature * xk), 0.2);   % lane width along vehicle y
        dL = (yc + (0.5 * w - halfLine) / c) - edge;
        dR = -edge - (yc - (0.5 * w - halfLine) / c);
        if dL < dlcL
            dlcL = dL;
            rateAtMinL = ycDot;
        end
        if dR < dlcR
            dlcR = dR;
            rateAtMinR = ycDot;
        end
    end

    % Left line approaches when the lane centre moves right (ycDot < 0)
    tlcL = Ldw.TlcMax;
    if rateAtMinL < -1e-3
        tlcL = min(max(dlcL, 0) / (-rateAtMinL), Ldw.TlcMax);
    end
    tlcR = Ldw.TlcMax;
    if rateAtMinR > 1e-3
        tlcR = min(max(dlcR, 0) / rateAtMinR, Ldw.TlcMax);
    end

    armed = v >= Ldw.MinSpeed && quality > 0;
    ldwL = armed && (dlcL < Ldw.DlcWarn || tlcL < Ldw.TlcWarn);
    ldwR = armed && (dlcR < Ldw.DlcWarn || tlcR < Ldw.TlcWarn);
end
