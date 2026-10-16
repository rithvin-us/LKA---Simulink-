function LKA = lka_set_lane_width(LKA, width, options)
% LKA_SET_LANE_WIDTH Lane width of the track for the controller and the camera chain.
%
% Syntax:
%   LKA = lka_set_lane_width(LKA, 2.0)                       % e.g. the college track
%   LKA = lka_set_lane_width(LKA, 2.0, MinFactor=0.5, MaxFactor=2)
%
% The official aBaja LKA track has 3.5 m lanes, which is the default of lka_build_params:
% nominal width 3.5 m, plausible band 2.5..5 m in the lane fusion (LKA.Lane) and in the
% camera lane step (LKA.Vision.Track). On another track (a 2 m painted test lane) the
% band would reject every correct pair, so the nominal width and the band are set here
% together. The band scales with the width (factors 2.5/3.5 and 5/3.5 by default; use a
% wider band where the lane widens in bends).
% Base-workspace shortcut: set lkaLaneWidth before config/startup.m (or before
% lka_controller_params) and the built LKA uses it.

    arguments
        LKA (1,1) struct
        width (1,1) double {mustBePositive}
        options.MinFactor (1,1) double {mustBePositive} = 2.5 / 3.5
        options.MaxFactor (1,1) double {mustBePositive} = 5.0 / 3.5
    end
    LKA.Lane.WidthNominal = width;
    LKA.Lane.WidthMin = options.MinFactor * width;
    LKA.Lane.WidthMax = options.MaxFactor * width;
    LKA.Vision.Track.WidthInit = width;
    LKA.Vision.Track.WidthMin = options.MinFactor * width;
    LKA.Vision.Track.WidthMax = options.MaxFactor * width;
end
