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
%
% Adaptive range (F.AdaptiveRange, lka_set_track_profile 'tight'; off by default): one
% cubic cannot follow two or three bends of a snake lane, so an S over 15 m comes out
% nearly straight. While the 90th percentile of |residual| exceeds F.MaxResidual [m],
% the far end of the fitted range is cut back 1 m at a time (not below F.MinRange [m]
% of span). The near part, which the controller steers on, is then fitted faithfully,
% and ViewRange reports where the fit ends.
%
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
    deg = fitDegree(span, F);
    [cs, r] = robustFit(x, y, w0, deg, F);

    if isfield(F, 'AdaptiveRange') && F.AdaptiveRange
        xEnd = max(x);
        while upperResidual(r) > F.MaxResidual && xEnd - min(x) > F.MinRange
            xEnd = max(min(x) + F.MinRange, xEnd - 1);
            keep = x <= xEnd;
            if nnz(keep) < F.MinPoints
                break
            end
            x = x(keep); y = y(keep); w0 = w0(keep);
            span = max(x) - min(x);
            deg = fitDegree(span, F);
            [cs, r] = robustFit(x, y, w0, deg, F);
        end
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

function deg = fitDegree(span, F)
    if span >= F.CubicSpan
        deg = 3;
    elseif span >= F.QuadSpan
        deg = 2;
    else
        deg = 1;
    end
end

function [cs, r] = robustFit(x, y, w0, deg, F)
% Huber IRLS on the scaled abscissa x/10; r = unweighted residuals [m].
    A = (x / 10) .^ (0:deg);
    R = diag(F.Ridge(1:deg + 1));
    w = w0;
    for it = 1:F.Iterations
        Aw = A .* w;
        cs = (A' * Aw + R) \ (Aw' * y);
        r = y - A * cs;
        w = w0 .* min(1, F.HuberDelta ./ max(abs(r), eps));
    end
end

function v = upperResidual(r)
% 90th percentile of |r| (no Statistics Toolbox needed).
    a = sort(abs(r));
    v = a(max(1, ceil(0.9 * numel(a))));
end
