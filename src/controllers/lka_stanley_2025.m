function [delta, st, cte, headingErr] = lka_stanley_2025(est, v, active, st, Ts, St)
% LKA_STANLEY_2025 aBaja 2025 lateral controller (Stanley + PID), frame-fixed port.
%
% Faithful port of ros2_ws/src/lka/scripts/lka.py (2025 CarMaker entry):
%   Ld        = clip((6 + 0.4 v) * max(0.5, 1 - 0.2|cte_prev|), 3, 15)
%   cte       = lane-centre offset (deadzone 0.1 m)        [+ = steer left]
%   headErr   = direction of the chord from the closest lane point to the
%               look-ahead lane point, relative to vehicle heading
%   delta     = headErr + atan2(k*min(2, v/5)*cte, v + 1) + PID(cte)
%   PID       = 0.015/0.001/0.005, integral clip 0.15, output clip 3 deg,
%               derivative low-pass alpha 0.7
%   then clip +-12 deg, rate-limit 25 deg/s, output smoothing alpha 0.7,
%   decay x0.9 per step when inactive.
%
% Fix vs 2025: the 2025 node took the vehicle position from /InertialData
% (world frame) and the lane path from the camera (vehicle frame), producing
% cross-track errors of -249..-300 m. Here every error comes from the
% vehicle-frame lane estimate, so cte and heading are consistent.
%
% st = [integral; prevErr; lastDeriv; lastCmd; smoothed; prevCte]

    integ     = st(1);
    prevErr   = st(2);
    lastDeriv = st(3);
    lastCmd   = st(4);
    smoothed  = st(5);
    prevCte   = st(6);

    vEff = max(abs(v), 0.1);

    % Adaptive look-ahead (shorter when far off the centre)
    Ld = (St.LdBase + v * St.LdSpeedGain) * max(0.5, 1 - 0.2 * abs(prevCte));
    Ld = min(max(Ld, St.LdMin), St.LdMax);

    y0 = est.CenterOffset;
    yLd = y0 + est.HeadingAngle * Ld + 0.5 * est.Curvature * Ld^2;
    headingErr = atan2(yLd - y0, Ld);

    cte = y0;
    if abs(cte) < St.Deadzone
        cte = 0;
    end

    if active
        speedFactor = min(2.0, vEff / St.SpeedNorm);
        stanleyTerm = atan2(St.K * speedFactor * cte, vEff + St.KSoft);

        % PID on cte
        integ = min(max(integ + cte * Ts, -St.IntLimit), St.IntLimit);
        deriv = (cte - prevErr) / Ts;
        deriv = St.DerivAlpha * lastDeriv + (1 - St.DerivAlpha) * deriv;
        pid = St.Kp * cte + St.Ki * integ + St.Kd * deriv;
        pid = min(max(pid, -St.PidLimit), St.PidLimit);
        prevErr = cte;
        lastDeriv = deriv;

        raw = headingErr + stanleyTerm + pid;
        limited = min(max(raw, -St.SteerLimit), St.SteerLimit);
        dmax = St.RateLimit * Ts;
        rateLimited = lastCmd + min(max(limited - lastCmd, -dmax), dmax);
        smoothed = St.SmoothAlpha * rateLimited + (1 - St.SmoothAlpha) * smoothed;
        lastCmd = rateLimited;
    else
        smoothed = St.DecayInactive * smoothed;
        if abs(smoothed) < 0.01
            smoothed = 0;
            lastCmd = 0;
            integ = 0;
            prevErr = 0;
            lastDeriv = 0;
        end
    end

    delta = smoothed;
    st = [integ; prevErr; lastDeriv; lastCmd; smoothed; cte];
end
