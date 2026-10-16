function out = lka_wheels_out(margins, track)
% LKA_WHEELS_OUT Tyres touching or beyond EITHER lane line (rulebook K.2.4 / K.2.5).
%
% Syntax:
%   out = lka_wheels_out(margins, track)
%
% margins (N x 4, [FL FR RL RR]) from lka_ground_truth_metrics measure each tyre's outer
% edge against the line on its own side only. Once the car has crossed a line completely,
% the far-side tyres sit beyond that line while their own-side margin stays positive, so
% all(margins < 0) never flags "all four wheels out". The margin of a left tyre's inner
% edge to the right line equals the right tyre's own margin plus the track width (and vice
% versa), so both lines are checked from the margins alone.
%
% Output: N x 4 logical, true = tyre touching or beyond a lane line.

    mL = margins(:, [1 3]);
    mR = margins(:, [2 4]);
    outL = mL < 0 | mR + track < 0;
    outR = mR < 0 | mL + track < 0;
    out = [outL(:, 1), outR(:, 1), outL(:, 2), outR(:, 2)];
end
