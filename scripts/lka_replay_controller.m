function out = lka_replay_controller(t, sensors, speed, yawRate, LKA)
% LKA_REPLAY_CONTROLLER Runs LKA_Controller open loop on recorded lane measurements.
%
% Syntax:
%   out = lka_replay_controller(t, sensors, speed, yawRate, LKA)
%
% For lane videos: the car in the video does not react to the commands, so this gives
% what the LKA *would* command frame by frame. The controller model is the unchanged
% LKA_Controller (lane tracker, supervisor, pure pursuit / Stanley / MPC, limiter); it is
% fed through models/LKA_Replay_Harness.slx (built on first use).
%
% Inputs:
%   t        - n x 1 frame times [s], step LKA.Ts
%   sensors  - n x 1 LaneSensorBus structs (lka_vision_lane_step)
%   speed    - n x 1 [m/s];  yawRate - n x 1 [rad/s, + CCW]
%   LKA      - calibration (lka_build_params)
% Output struct (n x 1 each): SteerCmd [rad, + left], TurnRadius [m, + left], State
% (0 OFF .. 4 TAKEOVER), CurvatureCmd [1/m], LaneOffset [m] / LaneHeading [rad] /
% LaneCurvature [1/m] of the tracked lane centre at the rear axle, Quality.

    root = fileparts(fileparts(mfilename('fullpath')));
    mdl = 'LKA_Replay_Harness';
    if ~isfile(fullfile(root, 'models', [mdl '.slx']))
        buildHarness(fullfile(root, 'models'), mdl);
    end
    t = t(:); n = numel(t);
    L = [sensors.Left]; R = [sensors.Right];
    side = @(B) struct('LateralOffset', ts([B.LateralOffset]), 'HeadingAngle', ts([B.HeadingAngle]), ...
        'Curvature', ts([B.Curvature]), 'CurvatureDerivative', ts([B.CurvatureDerivative]), ...
        'Strength', ts([B.Strength]), 'ViewRange', ts([B.ViewRange]));
    REPLAY.Sensor = struct('Left', side(L), 'Right', side(R), 'FrameValid', ts(logical([sensors.FrameValid])));
    REPLAY.Speed = [t, speed(:)];
    REPLAY.YawRate = [t, yawRate(:)];
    REPLAY.StopTime = t(end) - t(1);

    prevLKA = [];
    if evalin('base', 'exist(''LKA'', ''var'')')
        prevLKA = evalin('base', 'LKA');
    end
    restore = onCleanup(@() restoreBase(prevLKA));
    assignin('base', 'LKA', LKA);
    assignin('base', 'REPLAY', REPLAY);
    evalin('base', 'create_lka_buses;');
    load_system(fullfile(root, 'models', [mdl '.slx']));
    simOut = sim(mdl, 'StopTime', num2str(REPLAY.StopTime));
    ls = simOut.logsout;
    pick = @(v) resampleTo(v, t - t(1));
    info = ls.getElement('lka_info').Values;
    est = ls.getElement('lane_estimate').Values;
    out.SteerCmd = pick(ls.getElement('steer_cmd').Values);
    out.TurnRadius = pick(ls.getElement('turn_radius').Values);
    out.State = pick(info.State);
    out.CurvatureCmd = pick(info.CurvatureCmd);
    out.LaneOffset = pick(est.CenterOffset);
    out.LaneHeading = pick(est.HeadingAngle);
    out.LaneCurvature = pick(est.Curvature);
    out.Quality = pick(est.Quality);

    function s = ts(v)
        s = timeseries(v(:), t - t(1));
    end
end

function v = resampleTo(tsv, tq)
    d = double(squeeze(tsv.Data));
    tt = tsv.Time;
    v = interp1(tt, d(:), tq, 'previous', 'extrap');
end

function restoreBase(prevLKA)
    if ~isempty(prevLKA)
        assignin('base', 'LKA', prevLKA);
    end
end

function buildHarness(modelDir, mdl)
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', 'LKA.Ts', ...
        'SaveFormat', 'Dataset', 'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
        'ReturnWorkspaceOutputs', 'on');
    set_param(mdl, 'Description', ['Open-loop replay of LKA_Controller on recorded lane measurements ' ...
        '(lka_replay_controller.m): REPLAY.Sensor (LaneSensorBus timeseries), REPLAY.Speed, REPLAY.YawRate.']);
    fw = @(name, var, pos) add_block('simulink/Sources/From Workspace', [mdl '/' name], 'VariableName', var, ...
        'SampleTime', 'LKA.Ts', 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value', 'Position', pos);
    fw('Lane Sensor', 'REPLAY.Sensor', [40 40 160 70]);
    set_param([mdl '/Lane Sensor'], 'OutDataTypeStr', 'Bus: LaneSensorBus');
    fw('Speed', 'REPLAY.Speed', [40 110 160 140]);
    fw('Yaw Rate', 'REPLAY.YawRate', [40 180 160 210]);
    add_block('simulink/Sources/Constant', [mdl '/Enable'], 'Value', 'true', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', 'LKA.Ts', 'Position', [40 250 160 280]);
    add_block('simulink/Ports & Subsystems/Model', [mdl '/LKA Controller'], 'ModelName', 'LKA_Controller', ...
        'SimulationMode', 'Normal', 'Position', [260 40 460 300]);
    inNames = {'Lane Sensor', 'Speed', 'Yaw Rate', 'Enable'};
    for k = 1:4
        add_line(mdl, [inNames{k} '/1'], sprintf('LKA Controller/%d', k), 'autorouting', 'smart');
    end
    names = {'steer_cmd', 'turn_radius', 'lka_info', 'lane_estimate'};
    ph = get_param([mdl '/LKA Controller'], 'PortHandles');
    for k = 1:4
        tb = add_block('simulink/Sinks/Terminator', sprintf('%s/Out %d', mdl, k), ...
            'Position', [560 40 + 60 * k 580 60 + 60 * k]);
        tp = get_param(tb, 'PortHandles');
        lh = add_line(mdl, ph.Outport(k), tp.Inport(1), 'autorouting', 'smart');
        set_param(lh, 'Name', names{k});
        set_param(ph.Outport(k), 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName');
    end
    save_system(mdl, fullfile(modelDir, [mdl '.slx']));
    close_system(mdl, 0);
end
