function kappa = lka_steer_to_curvature(delta, v, Veh)
% LKA_STEER_TO_CURVATURE Path curvature produced by a road-wheel angle.
%
% Inverse of lka_curvature_to_steer:  kappa = tan(delta) / (L + K_us*v^2).
% The turning radius of the vehicle (rear-axle path) is R = 1/kappa.
%
% Inputs:  delta [rad] (+ left), v [m/s], Veh (LKA.Veh: L, Kus)
% Output:  kappa [1/m] (+ left)

    kappa = tan(delta) / (Veh.L + Veh.Kus * v^2);
end
