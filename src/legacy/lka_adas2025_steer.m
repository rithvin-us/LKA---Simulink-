function [deltaLeft, state] = lka_adas2025_steer(yc, psiC, v, yawRate, state, P, dt)
% LKA_ADAS2025_STEER Replica of the ADAS_2026 Python steering chain in the LKA frame.
%
% Syntax:
%   [deltaLeft, state] = lka_adas2025_steer(yc, psiC, v, yawRate, state, P, dt)
%
% Chain replicated (ADAS_2026, read-only): steering_v2.stanley_steering_deg ->
% SlewRateLimiter -> SecondMeanSteering (1 s mean) -> firmware 24 deg lock.
%
% Their conventions: cte > 0 = lane centre RIGHT of the vehicle; heading_deg has the
% SAME sign as the lane heading psiC of this repo (lane turning left -> positive,
% see docs/adas2026_audit.md C1); steering + = RIGHT. As coded:
%   steer_right = heading + atan(K*cte/(Ksoft + v))  [deg]
% The heading term must oppose the error, so the corrected law is
%   steer_right = -heading + atan(K*cte/(Ksoft + v)).
%
% Inputs:
%   yc      - lane-centre offset at the measurement point (+ left) [m]
%   psiC    - lane heading relative to the vehicle (+ CCW) [rad]
%   v       - measured speed [m/s] (replaced by P.AssumedSpeed unless NaN)
%   yawRate - gyro yaw rate (+ CCW) [rad/s], used only with P.UseImu
%   state   - from lka_adas2025_params
%   P       - from lka_adas2025_params
%   dt      - call period [s]
% Outputs:
%   deltaLeft - road-wheel angle command (+ left) [rad]

    cte = -yc * P.WidthScale;
    headingDeg = P.HeadingScale * rad2deg(psiC);
    vEff = v;
    if ~isnan(P.AssumedSpeed)
        vEff = P.AssumedSpeed;
    end
    vEff = max(vEff, 0);
    if P.HeadingBug
        steerRight = headingDeg + rad2deg(atan(P.K * cte / (P.KSoft + vEff)));
    else
        steerRight = -headingDeg + rad2deg(atan(P.K * cte / (P.KSoft + vEff)));
    end
    if P.UseImu
        rDps = rad2deg(yawRate);                         % gyro, + left
        if P.HeadingBug
            steerRight = steerRight - P.KYawDamp * rDps; % as coded: steering_v2.py:93
        else
            steerRight = steerRight + P.KYawDamp * rDps; % damping opposes the rotation
        end
    end
    steerRight = min(max(steerRight, -P.MaxSteerDeg), P.MaxSteerDeg);

    % SlewRateLimiter (first call passes through)
    if isnan(state.Last)
        state.Last = steerRight;
    else
        step = P.SlewRateDps * dt;
        state.Last = state.Last + min(max(steerRight - state.Last, -step), step);
    end

    % SecondMeanSteering: emit the mean of each window, hold it in between
    if P.MeanWindow > 0
        state.Sum = state.Sum + state.Last;
        state.N = state.N + 1;
        state.Elapsed = state.Elapsed + dt;
        if state.Elapsed >= P.MeanWindow - 1e-9
            state.Out = state.Sum / state.N;
            state.Sum = 0; state.N = 0; state.Elapsed = 0;
        end
        out = state.Out;
    else
        out = state.Last;
    end
    out = min(max(out, -P.FirmwareMaxDeg), P.FirmwareMaxDeg);
    deltaLeft = -deg2rad(out);
end
