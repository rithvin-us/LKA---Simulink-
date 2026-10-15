function [kappaCmd, iA, info] = lka_mpc_curvature(est, v, accel, kappaPrev, kappaVeh, iA0, Mpc, Veh, Lim, Ts)
% LKA_MPC_CURVATURE Linear time-varying MPC on lane errors with curvature command.
%
% Prediction model (vehicle relative to the lane centre):
%   e_y'     = v_k * e_psi                      lateral deviation        (+ left)
%   e_psi'   = v_k * (kappa_v - kappa_road)     heading error            (+ CCW)
%   kappa_v' = (kappa_cmd - kappa_v) / tau      path curvature with steering lag
% discretised exactly (zero-order hold) at Ts for each horizon step. The speed
% along the horizon follows the measured acceleration, v_k = v + accel*k*Ts
% (clipped to [InitialSpeed, VMax]); holding the speed constant made the MPC
% over-steer while the vehicle accelerates from standstill. Road curvature is
% previewed from the tracked lane clothoid: kappa_road(k) = kappa + dkappa*s_k.
%
% Cost over N steps:
%   sum Qy*e_y^2 + Qpsi*e_psi^2 + Ru*(kappa_cmd - kappa_road)^2 + Rdu*dkappa_cmd^2
% (terminal state weighted TerminalFactor times), i.e. track the lane centre, use
% the road curvature as feed-forward, and move the command smoothly.
%
% Constraints, per step with that step's speed (the Curvature Limiter envelope):
%   |kappa_cmd(k)|            <= kappa_max(v_k)          lateral acceleration / steering angle
%   |kappa_cmd(k)-kappa(k-1)| <= kappa_rate_max(v_k)*Ts  lateral jerk / steering rate
%
% Solved as a condensed QP with mpcActiveSetSolver (warm-started with the active
% set iA0 of the previous step). On solver failure the first move of the
% unconstrained optimum, clipped to the constraints, is applied.
%
% Inputs:
%   est       - LaneEstimateBus struct (lane centre seen from the rear axle)
%   v, accel  - speed [m/s] and longitudinal acceleration [m/s^2]
%   kappaPrev - curvature command applied in the previous step [1/m]
%   kappaVeh  - current vehicle path curvature [1/m]
%   iA0       - previous active set (logical, 4*N x 1)
% Outputs:
%   kappaCmd  - first optimal move [1/m]
%   iA        - active set for warm start
%   info      - [exitflag; predicted e_y at end of horizon]

    N = Mpc.Horizon;
    tau = max(Veh.SteerLagTau, 1e-3);
    vk = min(max(v + accel * Ts * (0:N-1)', Mpc.InitialSpeed), Mpc.VMax);
    sk = [0; cumsum(vk(1:end-1) * Ts)];

    x0 = [-est.CenterOffset; -est.HeadingAngle; kappaVeh];
    kRoad = est.Curvature + est.CurvatureDerivative * sk;

    % ---------------- LTV prediction X = Phi*x0 + Gam*U + Psi*D  (stacked x1..xN)
    Phi = zeros(3*N, 3);
    Gam = zeros(3*N, N);
    Psi = zeros(3*N, N);
    kMax = zeros(N, 1);
    dMax = zeros(N, 1);
    prevRows = (1:3)';
    for k = 1:N
        Ac = [0 vk(k) 0; 0 0 vk(k); 0 0 -1/tau];
        M = expm([Ac, [0; 0; 1/tau], [0; -vk(k); 0]; zeros(2, 5)] * Ts);
        A = M(1:3, 1:3); B = M(1:3, 4); E = M(1:3, 5);
        rows = 3*(k-1) + (1:3)';
        if k == 1
            Phi(rows, :) = A;
        else
            % columns >= k of the previous block row are still zero
            Phi(rows, :) = A * Phi(prevRows, :);
            Gam(rows, :) = A * Gam(prevRows, :);
            Psi(rows, :) = A * Psi(prevRows, :);
        end
        Gam(rows, k) = B;
        Psi(rows, k) = E;
        prevRows = rows;
        [kMax(k), kRate] = lka_curvature_limits(vk(k), Veh, Lim);
        dMax(k) = kRate * Ts;
    end

    % ---------------- cost
    qd = repmat([Mpc.Qy; Mpc.Qpsi; 0], N, 1);
    qd(end-2:end) = qd(end-2:end) * Mpc.TerminalFactor;
    Q = diag(qd);
    Dm = eye(N) - diag(ones(N-1, 1), -1);          % U(k) - U(k-1)
    u1 = zeros(N, 1); u1(1) = kappaPrev;
    free = Phi * x0 + Psi * kRoad;
    H = Gam' * Q * Gam + Mpc.Ru * eye(N) + Mpc.Rdu * (Dm' * Dm);
    f = Gam' * Q * free - Mpc.Ru * kRoad - Mpc.Rdu * Dm' * u1;
    H = 0.5 * (H + H');

    % ---------------- constraints  Ain*U <= bin
    Ain = [eye(N); -eye(N); Dm; -Dm];
    bin = [kMax; kMax; dMax + u1; dMax - u1];

    opt = mpcActiveSetOptions;
    opt.MaxIterations = 200;
    Aeq = zeros(0, N);
    beq = zeros(0, 1);
    [U, exitflag, iA] = mpcActiveSetSolver(H, f, Ain, bin, Aeq, beq, iA0, opt);

    if exitflag > 0
        kappaCmd = U(1);
    else
        Uu = -(H \ f);
        kappaCmd = min(max(Uu(1), kappaPrev - dMax(1)), kappaPrev + dMax(1));
        kappaCmd = min(max(kappaCmd, -kMax(1)), kMax(1));
        iA = false(4*N, 1);
        U = Uu;
    end
    Xend = free(end-2:end) + Gam(end-2:end, :) * U;
    info = [double(exitflag); Xend(1)];
end
