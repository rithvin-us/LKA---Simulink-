function [b, ok] = lka_fit_boundary(pts, F)
% LKA_FIT_BOUNDARY Robust weighted polynomial y(x) of one lane boundary.
%
% Syntax:
%   [b, ok] = lka_fit_boundary(pts, F)        F = LKA.Vision.Fit
%
% pts = [x y w]: support pixels in the vehicle frame [m] with detector probability w.
% Degree from the longitudinal span (linear < QuadSpan <= quadratic < CubicSpan <= cubic),
% ridge penalty on the higher coefficients (scaled abscissa x/10) against wild
% extrapolation, Huber IRLS (HuberDelta) against outliers.
% Output b uses the LaneBoundaryBus fields and order:
%   y(x) = LateralOffset + tan(HeadingAngle)*x + Curvature*x^2/2 + CurvatureDerivative*x^3/6
% Strength = min(1, span/FullStrengthSpan) * min(1, mean(w)/ProbFull); 0 when ok is false.

    b = struct('LateralOffset', 0, 'HeadingAngle', 0, 'Curvature', 0, ...
        'CurvatureDerivative', 0, 'Strength', 0, 'ViewRange', 0);
    ok = false;
    if size(pts, 1) < F.MinPoints
        return
    end
    x = pts(:, 1); y = pts(:, 2); w0 = pts(:, 3);
    span = max(x) - min(x);
    if span < F.MinSpan
        return
    end
    if span >= F.CubicSpan
        deg = 3;
    elseif span >= F.QuadSpan
        deg = 2;
    else
        deg = 1;
    end

    A = (x / 10) .^ (0:deg);
    R = diag(F.Ridge(1:deg + 1));
    w = w0;
    for it = 1:F.Iterations
        Aw = A .* w;
        cs = (A' * Aw + R) \ (Aw' * y);
        r = y - A * cs;
        w = w0 .* min(1, F.HuberDelta ./ max(abs(r), eps));
    end
    c = [cs; zeros(3 - deg, 1)] ./ (10 .^ (0:3))';

    b.LateralOffset = c(1);
    b.HeadingAngle = atan(c(2));
    b.Curvature = 2 * c(3);
    b.CurvatureDerivative = 6 * c(4);
    b.Strength = min(1, span / F.FullStrengthSpan) * min(1, mean(w0) / F.ProbFull);
    b.ViewRange = max(x);
    ok = true;
end
