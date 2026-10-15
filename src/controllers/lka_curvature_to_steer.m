function delta = lka_curvature_to_steer(kappa, v, Veh)
% LKA_CURVATURE_TO_STEER Road-wheel angle that drives a path of curvature kappa.
%
% Steady-state single-track (bicycle) relation with understeer gradient:
%   delta = L*kappa + K_us*a_y,   a_y = v^2*kappa      (Gillespie 1992)
% written as delta = atan(kappa*(L + K_us*v^2)) so that it reduces to the exact
% kinematic Ackermann angle atan(L*kappa) at low speed.
%
% Inputs:  kappa [1/m] (+ left), v [m/s], Veh (LKA.Veh: L, Kus)
% Output:  delta [rad] (+ left)

    delta = atan(kappa * (Veh.L + Veh.Kus * v^2));
end
