function [dx, ay, rearPose] = lka_single_track_dynamics(x, delta, vx, vehicle)
% LKA_SINGLE_TRACK_DYNAMICS Nonlinear single-track (bicycle) model of the Baja.
%
% State x = [X; Y; psi; vy; r]  (CG position and heading in the world frame,
% lateral velocity and yaw rate in the body frame). Longitudinal speed vx is an
% input (the rulebook gives throttle to the safety driver; LKA is lateral only).
%
% Tyres: saturating lateral force F = mu*Fz*tanh(C*alpha/(mu*Fz)), linear with
% axle cornering stiffness C at small slip, limited by friction at large slip.
%
%   m*(vy_dot + vx*r) = Fyf*cos(delta) + Fyr
%   Iz*r_dot          = a*Fyf*cos(delta) - b*Fyr
%
% Below ~3 m/s the slip-angle model is ill-conditioned, so the derivatives are
% blended towards the kinematic bicycle model (r = vx*tan(delta)/L) with a
% 50 ms first-order convergence.
%
% Outputs: dx state derivative, ay lateral acceleration [m/s^2], rearPose
% [X Y psi] of the rear-axle centre (the LKA reference point).

    m = vehicle.mass; Iz = vehicle.Iz; a = vehicle.a; b = vehicle.b;
    L = a + b; g = 9.81; mu = vehicle.mu;

    X = x(1); Y = x(2); psi = x(3); vy = x(4); r = x(5);
    vxs = max(vx, 1.0);

    alphaF = delta - atan((vy + a*r) / vxs);
    alphaR = -atan((vy - b*r) / vxs);
    FzF = m*g*b/L;  FzR = m*g*a/L;
    FyF = mu*FzF * tanh(vehicle.Cf*alphaF / (mu*FzF));
    FyR = mu*FzR * tanh(vehicle.Cr*alphaR / (mu*FzR));

    vyDotDyn = (FyF*cos(delta) + FyR)/m - vx*r;
    rDotDyn  = (a*FyF*cos(delta) - b*FyR)/Iz;

    beta  = atan(b*tan(delta)/L);
    rKin  = vx*cos(beta)*tan(delta)/L;
    vyKin = vx*tan(beta);
    tauKin = 0.05;
    lam = min(max((vx - 1.5)/1.5, 0), 1);
    vyDot = lam*vyDotDyn + (1 - lam)*(vyKin - vy)/tauKin;
    rDot  = lam*rDotDyn  + (1 - lam)*(rKin - r)/tauKin;

    dx = [vx*cos(psi) - vy*sin(psi);
          vx*sin(psi) + vy*cos(psi);
          r;
          vyDot;
          rDot];
    ay = vyDot + vx*r;
    rearPose = [X - b*cos(psi), Y - b*sin(psi), psi];
end
