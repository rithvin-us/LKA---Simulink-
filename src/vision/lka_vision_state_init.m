function state = lka_vision_state_init(V)
% LKA_VISION_STATE_INIT Frame-to-frame state of the camera lane detector.
%   Width   lane-width estimate [m]
%   Left / Right: Valid (a direct fit is available as prior), Age [s] since that fit,
%                 Coeffs [c0 c1 c2 c3] of y(x) = c0 + c1 x + c2 x^2 + c3 x^3
    side = struct('Valid', false, 'Age', inf, 'Coeffs', zeros(1, 4));
    state = struct('Width', V.Track.WidthInit, 'Left', side, 'Right', side);
end
