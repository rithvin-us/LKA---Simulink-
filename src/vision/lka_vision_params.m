function V = lka_vision_params(profile)
% LKA_VISION_PARAMS Calibration of the camera lane detector (LKA.Vision).
%
% Syntax:
%   V = lka_vision_params(profile)        profile 'baja' | 'skeleton'
%
% Camera: Simulation 3D Camera of aBaja2026.slx (1280x720, f = 1109 px, no distortion).
% Extrinsics in the LKA vehicle frame (ISO 8855, origin = rear-axle centre on the ground):
%   'baja'     ZED-like mount used by the synthetic tests and LKA_Vision_TestBench.
%   'skeleton' "Hood center" mount of the Muscle car in aBaja2026.slx; initial estimate,
%              replaced by the calibrate_vision_bench measurement and the RoadRunner
%              co-simulation check (Phase 5).
% Pipeline: detector -> probability map -> metric bird's-eye (birdsEyeView) ->
% sliding-window search (Search) -> robust fit (Fit) -> frame-to-frame logic (Track).

    arguments
        profile (1,:) char {mustBeMember(profile, {'baja', 'skeleton'})}
    end

    % ---------------------------------------------------------- camera
    V.Camera.FocalLength    = [1109 1109];   % [px]
    V.Camera.PrincipalPoint = [640 360];     % [px]
    V.Camera.ImageSize      = [720 1280];    % [rows cols]
    switch profile
        case 'baja'
            V.Camera.X        = 1.60;        % ahead of the rear axle [m]
            V.Camera.Y        = 0;
            V.Camera.Height   = 1.20;        % [m]
            V.Camera.PitchDeg = 8;           % nose down [deg]
        case 'skeleton'
            V.Camera.X        = 2.60;
            V.Camera.Y        = 0;
            V.Camera.Height   = 1.10;
            V.Camera.PitchDeg = 0;
    end

    % ---------------------------------------------------------- bird's-eye view
    V.Bev.Resolution = 0.05;     % [m/px]
    V.Bev.Range      = 30;       % look-ahead from the camera [m]
    V.Bev.HalfWidth  = 6;        % lateral half-width [m]
    V.Bev.NearMargin = 0.5;      % start this far beyond the first visible ground point [m]

    % ---------------------------------------------------------- detector
    V.Detector    = 'yolop';     % 'yolop' | 'cnn' | 'classic'
    V.DetectorDir = '';          % set by lka_build_params (models/perception)
    V.UseGpu      = true;

    % ---------------------------------------------------------- sliding-window search
    V.Search.ProbThreshold = 0.5;
    V.Search.SliceLength   = 1.0;    % [m]
    V.Search.ColdHalfWidth = 0.9;    % window half-width around the nominal lane [m]
    V.Search.WarmHalfWidth = 0.45;   % around the previous boundary (lateral jump gate) [m]
    V.Search.MinSliceMass  = 4;      % summed probability needed in a slice window [px]
    V.Search.MaxGap        = 8;      % stop after this length without a hit [m]
    V.Search.MinCenterDist = 0.4;    % cold search keeps this far from the vehicle axis [m]
    V.Search.LineHalfWidth = 0.25;   % pixels this close to the slice peak feed the fit [m]

    % ---------------------------------------------------------- robust fit
    V.Fit.MinPoints        = 60;
    V.Fit.MinSpan          = 4;               % [m]
    V.Fit.QuadSpan         = 8;               % quadratic above this span [m]
    V.Fit.CubicSpan        = 15;              % cubic above this span [m]
    V.Fit.Ridge            = [0 0 1e-3 1e-2]; % penalty on the scaled [c0 c1 c2 c3] (numerical
                                              % safety only: larger values trade c2/c3 against
                                              % c0/c1 and bias the extrapolated offset at x = 0)
    V.Fit.HuberDelta       = 0.08;            % [m]
    V.Fit.Iterations       = 3;
    V.Fit.FullStrengthSpan = 12;              % span that gives full strength [m]
    V.Fit.ProbFull         = 0.7;             % mean probability that gives full strength

    % ---------------------------------------------------------- frame-to-frame logic
    V.Track.HoldTime     = 1.0;               % previous direct fit is a prior this long [s]
    V.Track.WidthInit    = 3.5;               % [m]
    V.Track.WidthTau     = 2.0;               % [s]
    V.Track.WidthTol     = 0.6;               % pair width vs estimate [m]
    V.Track.WidthMin     = 2.5;               % [m]
    V.Track.WidthMax     = 5.0;               % [m]
    V.Track.HeadingMax   = deg2rad(35);       % [rad]
    V.Track.CurvatureMax = 0.1;               % [1/m]

    % ---------------------------------------------------------- outputs
    V.PreviewSize  = [135 60];                % bird's-eye preview for the recorder [px]
    V.LatencySteps = 1;                       % modelled processing delay [frames]

    % ---------------------------------------------------------- Unreal test bench
    V.Bench.ScenePath    = '';                % set by lka_build_params (data/scenes/LKA/LKA.usd)
    V.Bench.FrameMap     = 3;                 % aBaja2026 InitFcn: X = x, Y = -y, yaw = -psi
    V.Bench.MountZOffset = 0;                 % camera height correction [m] (calibrate_vision_bench)
    V.Bench.PitchSign    = 1;                 % sign of the Simulation 3D Camera pitch mount
    V.Bench.LaneLabelId  = 0;                 % semantic id of lane markings (calibrate_vision_bench)
    V.Bench.VideoFile    = '';
    V.Bench.DatasetDir   = '';
    V.Bench.Decimation   = 5;                 % dataset: keep every Nth frame
end
