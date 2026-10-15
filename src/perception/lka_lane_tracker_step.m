function [x, P, st, quality] = lka_lane_tracker_step(x, P, st, z, zValid, noiseScale, v, yawRate, Ts, KF)
% LKA_LANE_TRACKER_STEP One step of the lane-centre Kalman filter.
%
% State (lane centre seen from the rear axle, vehicle frame):
%   x = [y; psi; kappa; dkappa]
%       y      lateral offset of the lane centre at x = 0      [m]   (+ left)
%       psi    heading of the lane centre relative to vehicle  [rad] (+ CCW)
%       kappa  lane curvature                                   [1/m]
%       dkappa curvature rate along the lane                   [1/m^2]
%
% Motion model (rear axle has no side-slip, so it moves along its own heading):
%   dy/dt      = v*psi
%   dpsi/dt    = v*kappa - yawRate
%   dkappa/dt  = v*dkappa
%   ddkappa/dt = white noise
% Discretised exactly for constant v over Ts (clothoid shift by v*Ts). The
% measured yaw rate is the control input, so the filter keeps predicting the
% lane through detector dropouts and latency (dead reckoning).
%
% Measurement: z = x + noise (lane-centre measurement from
% lka_estimate_lane_center), noise variance KF.SigmaMeas.^2 * noiseScale.
% Offset/heading innovations outside a chi-square gate are rejected; after KF.MaxRejects
% consecutive rejections the filter re-initialises on the measurement (genuine
% lane geometry change, e.g. lane change or re-acquisition).
%
% st = [initialised; coastTime; rejectCount]
% quality = 1 - sqrt(P(1,1))/KF.SigmaYRef, clipped to [0,1] (0 before init).

    initialised = st(1) > 0.5;
    coastTime   = st(2);
    rejects     = st(3);

    Rm = diag(KF.SigmaMeas.^2) * noiseScale;

    if ~initialised
        if zValid
            x = z;
            P = Rm;
            initialised = true;
            coastTime = 0;
            rejects = 0;
        else
            coastTime = coastTime + Ts;
        end
    else
        % ---------------- Predict
        [x, F] = lka_lane_predict(x, v, yawRate, Ts);
        Q = diag(KF.Qdiag) * Ts;
        P = F * P * F' + Q;

        % ---------------- Update
        if zValid
            nu = z - x;
            nu(2) = lka_wrap_angle(nu(2));
            S = P + Rm;
            % Gate on offset and heading only: mis-detections show up there, while
            % curvature may legitimately step where arcs join without spirals.
            nis = nu(1:2)' * (S(1:2, 1:2) \ nu(1:2));
            if nis <= KF.Gate
                K = P / S;
                x = x + K * nu;
                IKH = eye(4) - K;
                P = IKH * P * IKH' + K * Rm * K';   % Joseph form, keeps P symmetric PD
                coastTime = 0;
                rejects = 0;
            else
                rejects = rejects + 1;
                coastTime = coastTime + Ts;
                if rejects >= KF.MaxRejects
                    x = z;
                    P = Rm;
                    rejects = 0;
                    coastTime = 0;
                end
            end
        else
            coastTime = coastTime + Ts;
        end
        P = 0.5 * (P + P');
    end

    st = [double(initialised); coastTime; rejects];
    if initialised
        quality = min(max(1 - sqrt(max(P(1,1), 0)) / KF.SigmaYRef, 0), 1);
    else
        quality = 0;
    end
end
