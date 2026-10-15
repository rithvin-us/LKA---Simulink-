function build_lka_models()
% BUILD_LKA_MODELS Generates the LKA Simulink models from source.
%
%   models/LKA_Controller.slx     referenced model - complete LKA algorithm
%   models/VirtualLaneSensor.slx  subsystem reference - emulated lane detector
%   models/LKA_MIL_TestBench.slx  closed-loop MIL test bench (Baja plant)
%
% The models are generated, not hand-drawn, so that every block, parameter and
% connection is reviewable in this file and the models can be rebuilt after a
% change. All algorithm code lives in src/ and is called from MATLAB Function
% blocks, so the unit tests exercise exactly the code the models run.
%
% Requires the base workspace from config/startup.m (LKA, vehicle, sensor, buses).

    root = fileparts(fileparts(mfilename('fullpath')));
    modelDir = fullfile(root, 'models');
    if ~isfolder(modelDir), mkdir(modelDir); end
    addpath(modelDir);

    evalin('base', 'create_lka_buses;');
    names = {'LKA_MIL_TestBench', 'LKA_Controller', 'VirtualLaneSensor'};
    for k = 1:numel(names)
        if bdIsLoaded(names{k}), close_system(names{k}, 0); end
        f = fullfile(modelDir, [names{k} '.slx']);
        if isfile(f), delete(f); end
    end

    buildController(modelDir);
    buildSensor(modelDir);
    buildBench(modelDir);
    fprintf('[LKA build] Models written to %s\n', modelDir);
end

% =============================================================================
%                              LKA_Controller.slx
% =============================================================================
function buildController(modelDir)
    mdl = 'LKA_Controller';
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
        'FixedStep', 'LKA.Ts', 'StopTime', 'inf', 'SaveFormat', 'Dataset', ...
        'SignalLogging', 'on', 'AlgebraicLoopMsg', 'error', ...
        'ModelReferenceNumInstancesAllowed', 'Single');
    set_param(mdl, 'Description', ['Lane Keeping Assist: lane perception (validation, centre fusion, ' ...
        'Kalman tracker), supervisor FSM, lane departure warning, pure pursuit / Stanley 2025 / MPC ' ...
        'lateral controllers, a_y and jerk limited curvature command, steering actuator manager, turning radius.']);

    inp(mdl, 'LaneSensor', 1, [30 100 60 114], 'Bus: LaneSensorBus');
    inp(mdl, 'Speed',      2, [30 220 60 234], 'double');
    inp(mdl, 'YawRate',    3, [30 300 60 314], 'double');
    inp(mdl, 'Enable',     4, [30 380 60 394], 'boolean');

    lp  = sub(mdl, 'Lane Perception',   {'LaneSensor', 'Speed', 'YawRate'}, {'LaneEstimate'}, [150 80 330 330]);
    sup = sub(mdl, 'LKA Supervisor',    {'Enable', 'Speed', 'LaneEstimate'}, {'LkaState', 'Active'}, [450 360 620 470]);
    ldw = sub(mdl, 'Lane Departure Warning', {'LaneEstimate', 'Speed', 'YawRate'}, ...
        {'LdwLeft', 'LdwRight', 'DlcLeft', 'DlcRight', 'TlcLeft', 'TlcRight'}, [450 520 620 680]);
    lc  = sub(mdl, 'Lateral Controllers', {'LaneEstimate', 'Speed', 'Active', 'SteerApplied', 'Saturated', 'YawRate'}, ...
        {'CurvatureReq', 'CurvaturePP', 'CurvatureStanley', 'CurvatureMPC', 'CurvatureFF', 'Lookahead', 'TargetOffset'}, ...
        [700 60 900 300]);
    cl  = sub(mdl, 'Curvature Limiter', {'CurvatureReq', 'Speed'}, {'CurvatureCmd', 'CurvatureLimit'}, [980 60 1130 140]);
    sc  = sub(mdl, 'Steering Command', {'CurvatureCmd', 'Speed'}, {'SteerReq'}, [1200 60 1340 120]);
    sam = sub(mdl, 'Steering Actuator Manager', {'SteerReq', 'Active'}, {'SteerCmd', 'Authority', 'Saturated'}, [1400 60 1560 160]);
    tr  = sub(mdl, 'Turning Radius', {'SteerCmd', 'Speed'}, {'TurnRadius', 'CurvatureOut', 'LatAccel'}, [1640 60 1780 160]);

    buildLanePerception(lp);
    buildSupervisor(sup);
    buildLdw(ldw);
    buildLateralControllers(lc);
    buildLimiter(cl);
    buildSteeringCommand(sc);
    buildActuatorManager(sam);
    buildTurningRadius(tr);

    add_block('simulink/Discrete/Unit Delay', [mdl '/Applied Steer Memory'], ...
        'InitialCondition', '0', 'SampleTime', '-1', 'Position', [1450 250 1490 280]);
    add_block('simulink/Discrete/Unit Delay', [mdl '/Saturation Memory'], ...
        'InitialCondition', '0', 'SampleTime', '-1', 'Position', [1450 310 1490 340]);

    pack = mlfb(mdl, 'Pack LKA Info', packInfoCode(), [1840 300 2000 820], ...
        'params', {'LKA'}, 'busOut', {'info', 'LkaInfoBus'});

    outp(mdl, 'SteerCmd',     1, [2100 80 2130 94], 'double');
    outp(mdl, 'TurnRadius',   2, [2100 180 2130 194], 'double');
    outp(mdl, 'LkaInfo',      3, [2100 550 2130 564], 'Bus: LkaInfoBus');
    outp(mdl, 'LaneEstimate', 4, [2100 650 2130 664], 'Bus: LaneEstimateBus');

    % ---- wiring
    ln(mdl, 'LaneSensor/1', 'Lane Perception/1');
    ln(mdl, 'Speed/1', 'Lane Perception/2');
    ln(mdl, 'YawRate/1', 'Lane Perception/3');
    ln(mdl, 'Enable/1', 'LKA Supervisor/1');
    ln(mdl, 'Speed/1', 'LKA Supervisor/2');
    ln(mdl, 'Lane Perception/1', 'LKA Supervisor/3');
    ln(mdl, 'Lane Perception/1', 'Lane Departure Warning/1');
    ln(mdl, 'Speed/1', 'Lane Departure Warning/2');
    ln(mdl, 'YawRate/1', 'Lane Departure Warning/3');
    ln(mdl, 'Lane Perception/1', 'Lateral Controllers/1');
    ln(mdl, 'Speed/1', 'Lateral Controllers/2');
    ln(mdl, 'LKA Supervisor/2', 'Lateral Controllers/3');
    ln(mdl, 'Applied Steer Memory/1', 'Lateral Controllers/4');
    ln(mdl, 'Saturation Memory/1', 'Lateral Controllers/5');
    ln(mdl, 'YawRate/1', 'Lateral Controllers/6');
    ln(mdl, 'Lateral Controllers/1', 'Curvature Limiter/1');
    ln(mdl, 'Speed/1', 'Curvature Limiter/2');
    ln(mdl, 'Curvature Limiter/1', 'Steering Command/1');
    ln(mdl, 'Speed/1', 'Steering Command/2');
    ln(mdl, 'Steering Command/1', 'Steering Actuator Manager/1');
    ln(mdl, 'LKA Supervisor/2', 'Steering Actuator Manager/2');
    ln(mdl, 'Steering Actuator Manager/1', 'Turning Radius/1');
    ln(mdl, 'Speed/1', 'Turning Radius/2');
    ln(mdl, 'Steering Actuator Manager/1', 'Applied Steer Memory/1');
    ln(mdl, 'Steering Actuator Manager/3', 'Saturation Memory/1');
    ln(mdl, 'Steering Actuator Manager/1', 'SteerCmd/1');
    ln(mdl, 'Turning Radius/1', 'TurnRadius/1');
    ln(mdl, 'Lane Perception/1', 'LaneEstimate/1');

    packSrc = {'LKA Supervisor/1', 'LKA Supervisor/2', 'Steering Actuator Manager/2', ...
        'Lateral Controllers/1', 'Lateral Controllers/2', 'Lateral Controllers/3', 'Lateral Controllers/4', ...
        'Lateral Controllers/5', 'Curvature Limiter/1', 'Curvature Limiter/2', 'Lateral Controllers/6', ...
        'Lateral Controllers/7', 'Steering Command/1', 'Steering Actuator Manager/1', 'Turning Radius/2', ...
        'Turning Radius/1', 'Turning Radius/3', 'Steering Actuator Manager/3', ...
        'Lane Departure Warning/1', 'Lane Departure Warning/2', 'Lane Departure Warning/3', ...
        'Lane Departure Warning/4', 'Lane Departure Warning/5', 'Lane Departure Warning/6'};
    for k = 1:numel(packSrc)
        ln(mdl, packSrc{k}, sprintf('Pack LKA Info/%d', k));
    end
    ln(mdl, 'Pack LKA Info/1', 'LkaInfo/1');
    nameOut(pack, 1, 'lka_info');

    nameOut([mdl '/Lane Perception'], 1, 'lane_estimate');
    nameOut([mdl '/Steering Actuator Manager'], 1, 'steer_cmd');
    nameOut([mdl '/Turning Radius'], 1, 'turn_radius');
    nameOut([mdl '/Lateral Controllers'], 1, 'curvature_req');
    nameOut([mdl '/Curvature Limiter'], 1, 'curvature_cmd');

    tidy(mdl);
    save_system(mdl, fullfile(modelDir, [mdl '.slx']));
end

function buildLanePerception(s)
    bv = mlfb(s, 'Boundary Validation', strjoin({
        'function [validL, validR, widthMeas, widthValid, offL, offR] = fcn(sensor, prevOffL, prevOffR, prevValidL, prevValidR, LKA)'
        '% Plausibility gating of the detected boundaries (see lka_validate_boundaries.m)'
        '[validL, validR, widthMeas, widthValid] = lka_validate_boundaries(sensor.Left, sensor.Right, ...'
        '    prevOffL, prevOffR, prevValidL, prevValidR, LKA.Val, LKA.Lane);'
        'validL = validL && sensor.FrameValid;'
        'validR = validR && sensor.FrameValid;'
        'offL = prevOffL; offR = prevOffR;'
        'if validL, offL = sensor.Left.LateralOffset; end'
        'if validR, offR = sensor.Right.LateralOffset; end'}, newline), [200 60 380 220], ...
        'params', {'LKA'}, 'busIn', {'sensor', 'LaneSensorBus'}, ...
        'outSpec', {'validL', 'boolean', '1', 'validR', 'boolean', '1', 'widthMeas', 'double', '1', ...
                    'widthValid', 'boolean', '1', 'offL', 'double', '1', 'offR', 'double', '1'});
    for nm = {'Offset Memory L', 'Offset Memory R', 'Valid Memory L', 'Valid Memory R'}
        add_block('simulink/Discrete/Unit Delay', [s '/' nm{1}], 'InitialCondition', '0', 'SampleTime', '-1');
    end
    ln(s, 'LaneSensor/1', 'Boundary Validation/1');
    ln(s, 'Offset Memory L/1', 'Boundary Validation/2');
    ln(s, 'Offset Memory R/1', 'Boundary Validation/3');
    ln(s, 'Valid Memory L/1', 'Boundary Validation/4');
    ln(s, 'Valid Memory R/1', 'Boundary Validation/5');
    ln(s, 'Boundary Validation/5', 'Offset Memory L/1');
    ln(s, 'Boundary Validation/6', 'Offset Memory R/1');
    ln(s, 'Boundary Validation/1', 'Valid Memory L/1');
    ln(s, 'Boundary Validation/2', 'Valid Memory R/1');
    nameOut(bv, 1, 'valid_left'); nameOut(bv, 2, 'valid_right');

    % Lane width estimator: w(k) = w(k-1) + Ts/tau*(meas - w(k-1)) when both lines are valid
    add_block('simulink/Math Operations/Sum', [s '/Width Error'], 'Inputs', '+-');
    add_block('simulink/Math Operations/Gain', [s '/Width Filter Gain'], 'Gain', 'LKA.Ts/LKA.Lane.WidthTau');
    add_block('simulink/Math Operations/Sum', [s '/Width Update'], 'Inputs', '++');
    add_block('simulink/Signal Routing/Switch', [s '/Width Valid Switch'], 'Criteria', 'u2 ~= 0');
    add_block('simulink/Discrete/Unit Delay', [s '/Lane Width Memory'], ...
        'InitialCondition', 'LKA.Lane.WidthNominal', 'SampleTime', '-1');
    ln(s, 'Boundary Validation/3', 'Width Error/1');
    ln(s, 'Lane Width Memory/1', 'Width Error/2');
    ln(s, 'Width Error/1', 'Width Filter Gain/1');
    ln(s, 'Width Filter Gain/1', 'Width Update/1');
    ln(s, 'Lane Width Memory/1', 'Width Update/2');
    ln(s, 'Width Update/1', 'Width Valid Switch/1');
    ln(s, 'Boundary Validation/4', 'Width Valid Switch/2');
    ln(s, 'Lane Width Memory/1', 'Width Valid Switch/3');
    ln(s, 'Width Valid Switch/1', 'Lane Width Memory/1');
    nameOut([s '/Width Valid Switch'], 1, 'lane_width');

    mlfb(s, 'Lane Centre Estimation', strjoin({
        'function [z, zValid, noiseScale, numBoundaries] = fcn(sensor, validL, validR, laneWidth)'
        '% Strength-weighted fusion of both boundaries into one lane-centre measurement'
        '[z, zValid, noiseScale, numBoundaries] = lka_estimate_lane_center(sensor.Left, sensor.Right, validL, validR, laneWidth);'}, newline), ...
        [480 60 660 200], 'busIn', {'sensor', 'LaneSensorBus'});
    ln(s, 'LaneSensor/1', 'Lane Centre Estimation/1');
    ln(s, 'Boundary Validation/1', 'Lane Centre Estimation/2');
    ln(s, 'Boundary Validation/2', 'Lane Centre Estimation/3');
    ln(s, 'Width Valid Switch/1', 'Lane Centre Estimation/4');

    kf = mlfb(s, 'Lane State Tracker (Kalman)', strjoin({
        'function [x, P, st, quality] = fcn(xPrev, PPrev, stPrev, z, zValid, noiseScale, v, yawRate, LKA)'
        '% Kalman filter on [offset; heading; curvature; curvature rate] of the lane centre,'
        '% predicted with vehicle speed and yaw rate (see lka_lane_tracker_step.m)'
        '[x, P, st, quality] = lka_lane_tracker_step(xPrev, PPrev, stPrev, z, zValid, noiseScale, v, yawRate, LKA.Ts, LKA.KF);'}, newline), ...
        [760 60 960 260], 'params', {'LKA'}, ...
        'outSpec', {'x', 'double', '[4 1]', 'P', 'double', '[4 4]', 'st', 'double', '[3 1]', 'quality', 'double', '1'});
    add_block('simulink/Discrete/Unit Delay', [s '/State Memory'], 'InitialCondition', 'zeros(4,1)', 'SampleTime', '-1');
    add_block('simulink/Discrete/Unit Delay', [s '/Covariance Memory'], 'InitialCondition', 'eye(4)', 'SampleTime', '-1');
    add_block('simulink/Discrete/Unit Delay', [s '/Tracker Status Memory'], 'InitialCondition', 'zeros(3,1)', 'SampleTime', '-1');
    ln(s, 'State Memory/1', 'Lane State Tracker (Kalman)/1');
    ln(s, 'Covariance Memory/1', 'Lane State Tracker (Kalman)/2');
    ln(s, 'Tracker Status Memory/1', 'Lane State Tracker (Kalman)/3');
    ln(s, 'Lane Centre Estimation/1', 'Lane State Tracker (Kalman)/4');
    ln(s, 'Lane Centre Estimation/2', 'Lane State Tracker (Kalman)/5');
    ln(s, 'Lane Centre Estimation/3', 'Lane State Tracker (Kalman)/6');
    ln(s, 'Speed/1', 'Lane State Tracker (Kalman)/7');
    ln(s, 'YawRate/1', 'Lane State Tracker (Kalman)/8');
    ln(s, 'Lane State Tracker (Kalman)/1', 'State Memory/1');
    ln(s, 'Lane State Tracker (Kalman)/2', 'Covariance Memory/1');
    ln(s, 'Lane State Tracker (Kalman)/3', 'Tracker Status Memory/1');
    nameOut(kf, 1, 'lane_state'); nameOut(kf, 4, 'lane_quality');

    mlfb(s, 'Latency Compensation and Packing', strjoin({
        'function est = fcn(x, laneWidth, quality, st, zValid, numBoundaries, v, yawRate, LKA)'
        '% Detector frames are LKA.KF.LatencyComp old: move the estimate to the current time'
        'xc = lka_lane_predict(x, v, yawRate, LKA.KF.LatencyComp);'
        'est = struct(''CenterOffset'', xc(1), ''HeadingAngle'', xc(2), ''Curvature'', xc(3), ...'
        '    ''CurvatureDerivative'', xc(4), ''LaneWidth'', laneWidth, ''Quality'', quality, ...'
        '    ''CoastTime'', st(2), ''NumBoundaries'', numBoundaries, ''MeasValid'', zValid);'}, newline), ...
        [1060 60 1220 220], 'params', {'LKA'}, 'busOut', {'est', 'LaneEstimateBus'});
    blk = 'Latency Compensation and Packing';
    ln(s, 'Lane State Tracker (Kalman)/1', [blk '/1']);
    ln(s, 'Width Valid Switch/1', [blk '/2']);
    ln(s, 'Lane State Tracker (Kalman)/4', [blk '/3']);
    ln(s, 'Lane State Tracker (Kalman)/3', [blk '/4']);
    ln(s, 'Lane Centre Estimation/2', [blk '/5']);
    ln(s, 'Lane Centre Estimation/4', [blk '/6']);
    ln(s, 'Speed/1', [blk '/7']);
    ln(s, 'YawRate/1', [blk '/8']);
    ln(s, [blk '/1'], 'LaneEstimate/1');
    tidy(s);
end

function buildSupervisor(s)
    % Activation conditions (with hysteresis) feeding the mode state machine
    add_block('simulink/Discontinuities/Relay', [s '/Speed Hysteresis'], ...
        'OnSwitchValue', 'LKA.Sup.SpeedOn', 'OffSwitchValue', 'LKA.Sup.SpeedOff', ...
        'OnOutputValue', '1', 'OffOutputValue', '0');
    add_block('simulink/Signal Attributes/Data Type Conversion', [s '/Speed On'], 'OutDataTypeStr', 'boolean');
    add_block('simulink/Logic and Bit Operations/Compare To Constant', [s '/Below Max Speed'], ...
        'relop', '<=', 'const', 'LKA.Sup.SpeedMax');
    add_block('simulink/Logic and Bit Operations/Logical Operator', [s '/Speed Ok'], 'Operator', 'AND', 'Inputs', '2');
    ln(s, 'Speed/1', 'Speed Hysteresis/1');
    ln(s, 'Speed Hysteresis/1', 'Speed On/1');
    ln(s, 'Speed/1', 'Below Max Speed/1');
    ln(s, 'Speed On/1', 'Speed Ok/1');
    ln(s, 'Below Max Speed/1', 'Speed Ok/2');

    add_block('simulink/Signal Routing/Bus Selector', [s '/Select Lane Status']);
    ln(s, 'LaneEstimate/1', 'Select Lane Status/1');
    set_param([s '/Select Lane Status'], 'OutputSignals', 'Quality,Curvature,CoastTime');
    add_block('simulink/Discontinuities/Relay', [s '/Quality Hysteresis'], ...
        'OnSwitchValue', 'LKA.Sup.QualityOn', 'OffSwitchValue', 'LKA.Sup.QualityOff', ...
        'OnOutputValue', '1', 'OffOutputValue', '0');
    add_block('simulink/Signal Attributes/Data Type Conversion', [s '/Quality Ok'], 'OutDataTypeStr', 'boolean');
    add_block('simulink/Math Operations/Abs', [s '/Abs Curvature']);
    add_block('simulink/Logic and Bit Operations/Compare To Constant', [s '/Within Steering Capability'], ...
        'relop', '<=', 'const', 'LKA.Sup.CurvatureCap');
    add_block('simulink/Logic and Bit Operations/Compare To Constant', [s '/Measurements Recent'], ...
        'relop', '<=', 'const', 'LKA.Sup.CoastDegrade');
    add_block('simulink/Logic and Bit Operations/Logical Operator', [s '/Lane Ok'], 'Operator', 'AND', 'Inputs', '3');
    add_block('simulink/Logic and Bit Operations/Compare To Constant', [s '/Coast Limit Exceeded'], ...
        'relop', '>', 'const', 'LKA.Sup.CoastMax');
    ln(s, 'Select Lane Status/1', 'Quality Hysteresis/1');
    ln(s, 'Quality Hysteresis/1', 'Quality Ok/1');
    ln(s, 'Select Lane Status/2', 'Abs Curvature/1');
    ln(s, 'Abs Curvature/1', 'Within Steering Capability/1');
    ln(s, 'Quality Ok/1', 'Lane Ok/1');
    ln(s, 'Within Steering Capability/1', 'Lane Ok/2');
    ln(s, 'Select Lane Status/3', 'Measurements Recent/1');
    ln(s, 'Measurements Recent/1', 'Lane Ok/3');
    ln(s, 'Select Lane Status/3', 'Coast Limit Exceeded/1');

    makeModeChart([s '/LKA Mode Logic']);
    ln(s, 'Enable/1', 'LKA Mode Logic/1');
    ln(s, 'Speed Ok/1', 'LKA Mode Logic/2');
    ln(s, 'Lane Ok/1', 'LKA Mode Logic/3');
    ln(s, 'Coast Limit Exceeded/1', 'LKA Mode Logic/4');
    ln(s, 'LKA Mode Logic/1', 'LkaState/1');
    ln(s, 'LKA Mode Logic/2', 'Active/1');
    nameOut([s '/Speed Ok'], 1, 'speed_ok');
    nameOut([s '/Lane Ok'], 1, 'lane_ok');
    nameOut([s '/Coast Limit Exceeded'], 1, 'coast_exceeded');
    nameOut([s '/LKA Mode Logic'], 1, 'lka_state');
    nameOut([s '/LKA Mode Logic'], 2, 'lka_active');
    tidy(s);
end

function makeModeChart(path)
    add_block('sflib/Chart', path);
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.Chart', 'Path', path);
    ch.ActionLanguage = 'MATLAB';
    ch.ChartUpdate = 'DISCRETE';
    ch.SampleTime = 'LKA.Ts';
    ch.Description = ['LKA mode logic. OFF <-> ON by enable. Inside ON: STANDBY until speed and lane ok; ', ...
        'ACTIVE steers; DEGRADED keeps steering on the tracker prediction while the lane is lost; ', ...
        'TAKEOVER (no steering) when the lane is lost longer than LKA.Sup.CoastMax.'];

    inNames = {'enable', 'speedOk', 'laneOk', 'coastExceeded'};
    for k = 1:numel(inNames)
        d = Stateflow.Data(ch); d.Name = inNames{k}; d.Scope = 'Input'; d.DataType = 'boolean';
    end
    d = Stateflow.Data(ch); d.Name = 'lkaState'; d.Scope = 'Output'; d.DataType = 'uint8';
    d = Stateflow.Data(ch); d.Name = 'active'; d.Scope = 'Output'; d.DataType = 'boolean';

    torHold = evalin('base', 'LKA.Sup.TorHold');

    sOff = Stateflow.State(ch);
    sOff.LabelString = sprintf('OFF\nentry:\nlkaState = uint8(0);\nactive = false;');
    sOff.Position = [40 140 150 90];

    sOn = Stateflow.State(ch);
    sOn.LabelString = 'ON';
    sOn.Position = [260 30 560 360];

    sStb = Stateflow.State(sOn);
    sStb.LabelString = sprintf('STANDBY\nentry:\nlkaState = uint8(1);\nactive = false;');
    sStb.Position = [290 80 160 90];
    sAct = Stateflow.State(sOn);
    sAct.LabelString = sprintf('ACTIVE\nentry:\nlkaState = uint8(2);\nactive = true;');
    sAct.Position = [620 80 160 90];
    sDeg = Stateflow.State(sOn);
    sDeg.LabelString = sprintf('DEGRADED\nentry:\nlkaState = uint8(3);\nactive = true;');
    sDeg.Position = [620 260 160 90];
    sTor = Stateflow.State(sOn);
    sTor.LabelString = sprintf('TAKEOVER\nentry:\nlkaState = uint8(4);\nactive = false;');
    sTor.Position = [290 260 160 90];

    t = Stateflow.Transition(ch); t.Destination = sOff; t.DestinationOClock = 0;
    t.SourceEndpoint = [115 110]; t.MidPoint = [115 125];
    t = Stateflow.Transition(sOn); t.Destination = sStb; t.DestinationOClock = 0;
    t.SourceEndpoint = [370 55]; t.MidPoint = [370 68];

    % Attachment points as clock positions on the state boxes: antiparallel pairs run
    % as separate straight arrows and every transition stays inside ON.
    tr(ch, sOff, sOn, '[enable]', 1.5, 10.0);
    tr(ch, sOn, sOff, '[~enable]', 8.0, 4.5);
    tr(sOn, sStb, sAct, '[speedOk && laneOk]', 2.5, 9.5);
    tr(sOn, sAct, sStb, '[~speedOk]', 8.5, 3.5);
    tr(sOn, sAct, sDeg, '[~laneOk]', 5.0, 1.0);
    tr(sOn, sDeg, sAct, '[laneOk]', 11.0, 7.0);
    tr(sOn, sDeg, sStb, '[~speedOk]', 10.0, 4.5);
    tr(sOn, sDeg, sTor, '[coastExceeded]', 9.0, 3.0);
    tr(sOn, sTor, sStb, sprintf('after(%g,sec)[laneOk]', torHold), 12.0, 6.0);
end

function t = tr(parent, src, dst, label, srcClock, dstClock)
    t = Stateflow.Transition(parent);
    t.Source = src;
    t.Destination = dst;
    t.SourceOClock = srcClock;
    t.DestinationOClock = dstClock;
    t.LabelString = label;
end

function buildLdw(s)
    m = mlfb(s, 'Line Crossing Assessment', strjoin({
        'function [ldwL, ldwR, dlcL, dlcR, tlcL, tlcR] = fcn(est, v, yawRate, LKA)'
        '% Distance / time to line crossing of the tyre edges and LDW (see lka_lane_departure.m)'
        '[ldwL, ldwR, dlcL, dlcR, tlcL, tlcR] = lka_lane_departure(est, v, yawRate, est.Quality, LKA.Veh, LKA.Lane, LKA.Ldw);'}, newline), ...
        [200 60 400 260], 'params', {'LKA'}, 'busIn', {'est', 'LaneEstimateBus'});
    ln(s, 'LaneEstimate/1', 'Line Crossing Assessment/1');
    ln(s, 'Speed/1', 'Line Crossing Assessment/2');
    ln(s, 'YawRate/1', 'Line Crossing Assessment/3');
    outs = {'LdwLeft', 'LdwRight', 'DlcLeft', 'DlcRight', 'TlcLeft', 'TlcRight'};
    for k = 1:numel(outs)
        ln(s, sprintf('Line Crossing Assessment/%d', k), [outs{k} '/1']);
    end
    nameOut(m, 3, 'dlc_left'); nameOut(m, 4, 'dlc_right');
    tidy(s);
end

function buildLateralControllers(s)
    pp = sub(s, 'Pure Pursuit', {'LaneEstimate', 'Speed', 'Active', 'Saturated'}, ...
        {'CurvaturePP', 'CurvatureFF', 'Lookahead', 'TargetOffset'}, [250 40 420 200]);
    st = sub(s, 'Stanley 2025 Baseline', {'LaneEstimate', 'Speed', 'Active'}, {'CurvatureStanley'}, [250 260 420 360]);
    mp = sub(s, 'MPC Lane Keeping', {'LaneEstimate', 'Speed', 'SteerApplied', 'YawRate'}, {'CurvatureMPC'}, [250 420 420 520]);
    buildPurePursuit(pp);
    buildStanley(st);
    buildMpc(mp);

    add_block('simulink/Sources/Constant', [s '/Controller Select'], 'Value', 'LKA.ControllerSelect', ...
        'SampleTime', 'inf', 'Position', [500 20 560 50]);
    add_block('simulink/Signal Routing/Multiport Switch', [s '/Controller Arbitration'], ...
        'Inputs', '3', 'DataPortOrder', 'One-based contiguous', 'Position', [620 40 680 260]);

    ln(s, 'LaneEstimate/1', 'Pure Pursuit/1');
    ln(s, 'Speed/1', 'Pure Pursuit/2');
    ln(s, 'Active/1', 'Pure Pursuit/3');
    ln(s, 'Saturated/1', 'Pure Pursuit/4');
    ln(s, 'LaneEstimate/1', 'Stanley 2025 Baseline/1');
    ln(s, 'Speed/1', 'Stanley 2025 Baseline/2');
    ln(s, 'Active/1', 'Stanley 2025 Baseline/3');
    ln(s, 'LaneEstimate/1', 'MPC Lane Keeping/1');
    ln(s, 'Speed/1', 'MPC Lane Keeping/2');
    ln(s, 'SteerApplied/1', 'MPC Lane Keeping/3');
    ln(s, 'YawRate/1', 'MPC Lane Keeping/4');

    ln(s, 'Controller Select/1', 'Controller Arbitration/1');
    ln(s, 'Pure Pursuit/1', 'Controller Arbitration/2');
    ln(s, 'Stanley 2025 Baseline/1', 'Controller Arbitration/3');
    ln(s, 'MPC Lane Keeping/1', 'Controller Arbitration/4');

    ln(s, 'Controller Arbitration/1', 'CurvatureReq/1');
    ln(s, 'Pure Pursuit/1', 'CurvaturePP/1');
    ln(s, 'Stanley 2025 Baseline/1', 'CurvatureStanley/1');
    ln(s, 'MPC Lane Keeping/1', 'CurvatureMPC/1');
    ln(s, 'Pure Pursuit/2', 'CurvatureFF/1');
    ln(s, 'Pure Pursuit/3', 'Lookahead/1');
    ln(s, 'Pure Pursuit/4', 'TargetOffset/1');
    nameOut([s '/Pure Pursuit'], 1, 'curvature_pp');
    nameOut([s '/Stanley 2025 Baseline'], 1, 'curvature_stanley');
    nameOut([s '/MPC Lane Keeping'], 1, 'curvature_mpc');
    tidy(s);
end

function buildPurePursuit(s)
    add_block('simulink/Lookup Tables/1-D Lookup Table', [s '/Look-ahead Schedule'], ...
        'Table', 'LKA.PP.LdBase', 'BreakpointsForDimension1', 'LKA.PP.LdSpeed', 'ExtrapMethod', 'Clip');
    mlfb(s, 'Pure Pursuit Geometry', strjoin({
        'function [kappaPP, Ld, yTarget, kappaFF] = fcn(est, LdBase, LKA)'
        '% Arc from the rear axle to the lane-centre point Ld ahead: kappa = 2*yT/(Ld^2 + yT^2)'
        '[kappaPP, Ld, yTarget, kappaFF] = lka_pure_pursuit(est, LdBase, LKA.PP);'}, newline), ...
        [300 40 480 180], 'params', {'LKA'}, 'busIn', {'est', 'LaneEstimateBus'});
    ln(s, 'Speed/1', 'Look-ahead Schedule/1');
    ln(s, 'LaneEstimate/1', 'Pure Pursuit Geometry/1');
    ln(s, 'Look-ahead Schedule/1', 'Pure Pursuit Geometry/2');

    % Integral action on the lane-centre offset (removes steady offset in long curves);
    % integrates only while active, not saturated and above IntMinSpeed, reset when inactive.
    add_block('simulink/Signal Routing/Bus Selector', [s '/Select Centre Offset']);
    ln(s, 'LaneEstimate/1', 'Select Centre Offset/1');
    set_param([s '/Select Centre Offset'], 'OutputSignals', 'CenterOffset');
    add_block('simulink/Math Operations/Gain', [s '/Integral Gain'], 'Gain', 'LKA.PP.Ki');
    add_block('simulink/Logic and Bit Operations/Compare To Constant', [s '/Above Integration Speed'], ...
        'relop', '>=', 'const', 'LKA.PP.IntMinSpeed');
    add_block('simulink/Logic and Bit Operations/Logical Operator', [s '/Not Saturated'], 'Operator', 'NOT', 'Inputs', '1');
    add_block('simulink/Logic and Bit Operations/Logical Operator', [s '/Integration Enable'], 'Operator', 'AND', 'Inputs', '3');
    add_block('simulink/Signal Attributes/Data Type Conversion', [s '/Enable To Double'], 'OutDataTypeStr', 'double');
    add_block('simulink/Math Operations/Product', [s '/Gate Integrator Input'], 'Inputs', '2');
    add_block('simulink/Logic and Bit Operations/Logical Operator', [s '/Reset When Inactive'], 'Operator', 'NOT', 'Inputs', '1');
    add_block('simulink/Discrete/Discrete-Time Integrator', [s '/Offset Integrator'], ...
        'IntegratorMethod', 'Integration: Forward Euler', 'ExternalReset', 'level', 'LimitOutput', 'on', ...
        'UpperSaturationLimit', 'LKA.PP.IntLimit', 'LowerSaturationLimit', '-LKA.PP.IntLimit', ...
        'InitialCondition', '0', 'SampleTime', '-1');
    add_block('simulink/Math Operations/Sum', [s '/Add Integral Curvature'], 'Inputs', '++');

    ln(s, 'Select Centre Offset/1', 'Integral Gain/1');
    ln(s, 'Speed/1', 'Above Integration Speed/1');
    ln(s, 'Saturated/1', 'Not Saturated/1');
    ln(s, 'Active/1', 'Integration Enable/1');
    ln(s, 'Not Saturated/1', 'Integration Enable/2');
    ln(s, 'Above Integration Speed/1', 'Integration Enable/3');
    ln(s, 'Integration Enable/1', 'Enable To Double/1');
    ln(s, 'Integral Gain/1', 'Gate Integrator Input/1');
    ln(s, 'Enable To Double/1', 'Gate Integrator Input/2');
    ln(s, 'Gate Integrator Input/1', 'Offset Integrator/1');
    ln(s, 'Active/1', 'Reset When Inactive/1');
    ln(s, 'Reset When Inactive/1', 'Offset Integrator/2');
    ln(s, 'Pure Pursuit Geometry/1', 'Add Integral Curvature/1');
    ln(s, 'Offset Integrator/1', 'Add Integral Curvature/2');

    ln(s, 'Add Integral Curvature/1', 'CurvaturePP/1');
    ln(s, 'Pure Pursuit Geometry/4', 'CurvatureFF/1');
    ln(s, 'Pure Pursuit Geometry/2', 'Lookahead/1');
    ln(s, 'Pure Pursuit Geometry/3', 'TargetOffset/1');
    nameOut([s '/Offset Integrator'], 1, 'curvature_integral');
    tidy(s);
end

function buildStanley(s)
    mlfb(s, 'Stanley + PID (aBaja 2025)', strjoin({
        'function [delta, st] = fcn(est, v, active, stPrev, LKA)'
        '% 2025 lka.py control law, errors taken from the vehicle-frame lane estimate'
        '[delta, st] = lka_stanley_2025(est, v, active, stPrev, LKA.Ts, LKA.St);'}, newline), ...
        [200 40 400 180], 'params', {'LKA'}, 'busIn', {'est', 'LaneEstimateBus'}, ...
        'outSpec', {'delta', 'double', '1', 'st', 'double', '[6 1]'});
    add_block('simulink/Discrete/Unit Delay', [s '/Stanley State Memory'], 'InitialCondition', 'zeros(6,1)', 'SampleTime', '-1');
    steerToCurv(s, 'Steering To Curvature');
    ln(s, 'LaneEstimate/1', 'Stanley + PID (aBaja 2025)/1');
    ln(s, 'Speed/1', 'Stanley + PID (aBaja 2025)/2');
    ln(s, 'Active/1', 'Stanley + PID (aBaja 2025)/3');
    ln(s, 'Stanley State Memory/1', 'Stanley + PID (aBaja 2025)/4');
    ln(s, 'Stanley + PID (aBaja 2025)/2', 'Stanley State Memory/1');
    ln(s, 'Stanley + PID (aBaja 2025)/1', 'Steering To Curvature/1');
    ln(s, 'Speed/1', 'Steering To Curvature/2');
    ln(s, 'Steering To Curvature/1', 'CurvatureStanley/1');
    nameOut([s '/Stanley + PID (aBaja 2025)'], 1, 'steer_stanley');
    tidy(s);
end

function buildMpc(s)
    % Curvature MPC with road-curvature preview and the a_y / jerk envelope as QP constraints
    nA = 4 * evalin('base', 'LKA.Mpc.Horizon');
    steerToCurv(s, 'Applied Path Curvature');
    mlfb(s, 'Vehicle Path Curvature', strjoin({
        'function kappaVeh = fcn(yawRate, v, kappaApplied)'
        '% Measured path curvature of the rear axle, r/v (falls back to the steering command at low speed)'
        'if v > 1'
        '    kappaVeh = yawRate / v;'
        'else'
        '    kappaVeh = kappaApplied;'
        'end'}, newline), [180 200 340 280]);
    mlfb(s, 'Curvature MPC (active-set QP)', strjoin({
        'function [kappaCmd, iA, info] = fcn(est, v, accel, kappaPrev, kappaVeh, iA0, LKA)'
        '% LTV lane-error MPC: model e_y, e_psi, kappa_v along the predicted speed profile;'
        '% preview kappa_road; |kappa| and |dkappa| constrained per step'
        '[kappaCmd, iA, info] = lka_mpc_curvature(est, v, accel, kappaPrev, kappaVeh, iA0, ...'
        '    LKA.Mpc, LKA.Veh, LKA.Lim, LKA.Ts);'}, newline), ...
        [300 40 500 200], 'params', {'LKA'}, 'busIn', {'est', 'LaneEstimateBus'}, ...
        'outSpec', {'kappaCmd', 'double', '1', 'iA', 'boolean', sprintf('[%d 1]', nA), 'info', 'double', '[2 1]'});
    add_block('simulink/Discrete/Unit Delay', [s '/Active Set Memory'], ...
        'InitialCondition', sprintf('false(%d,1)', nA), 'SampleTime', '-1');
    add_block('simulink/Sinks/Terminator', [s '/Solver Info']);
    % Longitudinal acceleration for the predicted speed profile: (v(k) - v(k-1)) / Ts, clipped
    add_block('simulink/Discrete/Unit Delay', [s '/Speed Memory'], 'InitialCondition', '0', 'SampleTime', '-1');
    add_block('simulink/Math Operations/Sum', [s '/Speed Change'], 'Inputs', '+-');
    add_block('simulink/Math Operations/Gain', [s '/Per Second'], 'Gain', '1/LKA.Ts');
    add_block('simulink/Discontinuities/Saturation', [s '/Acceleration Clip'], ...
        'UpperLimit', 'LKA.Mpc.AccelMax', 'LowerLimit', '-LKA.Mpc.AccelMax');
    ln(s, 'Speed/1', 'Speed Memory/1');
    ln(s, 'Speed/1', 'Speed Change/1');
    ln(s, 'Speed Memory/1', 'Speed Change/2');
    ln(s, 'Speed Change/1', 'Per Second/1');
    ln(s, 'Per Second/1', 'Acceleration Clip/1');

    ln(s, 'SteerApplied/1', 'Applied Path Curvature/1');
    ln(s, 'Speed/1', 'Applied Path Curvature/2');
    ln(s, 'LaneEstimate/1', 'Curvature MPC (active-set QP)/1');
    ln(s, 'Speed/1', 'Curvature MPC (active-set QP)/2');
    ln(s, 'Acceleration Clip/1', 'Curvature MPC (active-set QP)/3');
    add_block('simulink/Discrete/Unit Delay', [s '/Previous MPC Command'], 'InitialCondition', '0', 'SampleTime', '-1');
    ln(s, 'Previous MPC Command/1', 'Curvature MPC (active-set QP)/4');
    ln(s, 'YawRate/1', 'Vehicle Path Curvature/1');
    ln(s, 'Speed/1', 'Vehicle Path Curvature/2');
    ln(s, 'Applied Path Curvature/1', 'Vehicle Path Curvature/3');
    ln(s, 'Vehicle Path Curvature/1', 'Curvature MPC (active-set QP)/5');
    ln(s, 'Active Set Memory/1', 'Curvature MPC (active-set QP)/6');
    ln(s, 'Curvature MPC (active-set QP)/1', 'Previous MPC Command/1');
    ln(s, 'Curvature MPC (active-set QP)/2', 'Active Set Memory/1');
    ln(s, 'Curvature MPC (active-set QP)/3', 'Solver Info/1');
    ln(s, 'Curvature MPC (active-set QP)/1', 'CurvatureMPC/1');
    nameOut([s '/Curvature MPC (active-set QP)'], 3, 'mpc_solver_info');
    tidy(s);
end

function steerToCurv(s, name)
    mlfb(s, name, strjoin({
        'function kappa = fcn(delta, v, LKA)'
        '% kappa = tan(delta) / (L + K_us*v^2)'
        'kappa = lka_steer_to_curvature(delta, v, LKA.Veh);'}, newline), [480 60 620 140], 'params', {'LKA'});
end

function buildLimiter(s)
    mlfb(s, 'Limit Computation', strjoin({
        'function [kappaMax, kappaRateMax] = fcn(v, LKA)'
        '% |kappa| <= a_y,max/v^2 and steering-angle limit; |dkappa/dt| <= jerk_max/v^2 and steering-rate limit'
        '[kappaMax, kappaRateMax] = lka_curvature_limits(v, LKA.Veh, LKA.Lim);'}, newline), ...
        [120 160 300 260], 'params', {'LKA'});
    add_block('simulink/Math Operations/Gain', [s '/Negate Curvature Limit'], 'Gain', '-1');
    add_block('simulink/Math Operations/Gain', [s '/Negate Rate Limit'], 'Gain', '-1');
    add_block('simulink/Discontinuities/Saturation Dynamic', [s '/Lateral Acceleration Limit']);
    add_block('simulink/Discontinuities/Rate Limiter Dynamic', [s '/Lateral Jerk Limit']);
    ln(s, 'Speed/1', 'Limit Computation/1');
    ln(s, 'Limit Computation/1', 'Lateral Acceleration Limit/1');
    ln(s, 'CurvatureReq/1', 'Lateral Acceleration Limit/2');
    ln(s, 'Limit Computation/1', 'Negate Curvature Limit/1');
    ln(s, 'Negate Curvature Limit/1', 'Lateral Acceleration Limit/3');
    ln(s, 'Limit Computation/2', 'Lateral Jerk Limit/1');
    ln(s, 'Lateral Acceleration Limit/1', 'Lateral Jerk Limit/2');
    ln(s, 'Limit Computation/2', 'Negate Rate Limit/1');
    ln(s, 'Negate Rate Limit/1', 'Lateral Jerk Limit/3');
    ln(s, 'Lateral Jerk Limit/1', 'CurvatureCmd/1');
    ln(s, 'Limit Computation/1', 'CurvatureLimit/1');
    tidy(s);
end

function buildSteeringCommand(s)
    mlfb(s, 'Curvature To Road-Wheel Angle', strjoin({
        'function delta = fcn(kappa, v, LKA)'
        '% Single-track steady-state steering: delta = atan(kappa*(L + K_us*v^2))'
        'delta = lka_curvature_to_steer(kappa, v, LKA.Veh);'}, newline), [200 40 380 120], 'params', {'LKA'});
    ln(s, 'CurvatureCmd/1', 'Curvature To Road-Wheel Angle/1');
    ln(s, 'Speed/1', 'Curvature To Road-Wheel Angle/2');
    ln(s, 'Curvature To Road-Wheel Angle/1', 'SteerReq/1');
    tidy(s);
end

function buildActuatorManager(s)
    add_block('simulink/Signal Attributes/Data Type Conversion', [s '/Active To Double'], 'OutDataTypeStr', 'double');
    add_block('simulink/Discontinuities/Rate Limiter', [s '/Authority Ramp'], ...
        'RisingSlewLimit', '1/LKA.Act.RampTime', 'FallingSlewLimit', '-1/LKA.Act.RampTime', 'InitialCondition', '0');
    add_block('simulink/Math Operations/Product', [s '/Apply Authority'], 'Inputs', '2');
    add_block('simulink/Discontinuities/Saturation', [s '/Steering Angle Limit'], ...
        'UpperLimit', 'LKA.Veh.SteerMax', 'LowerLimit', '-LKA.Veh.SteerMax');
    add_block('simulink/Discontinuities/Rate Limiter', [s '/Steering Rate Limit'], ...
        'RisingSlewLimit', 'LKA.Veh.SteerRateMax', 'FallingSlewLimit', '-LKA.Veh.SteerRateMax', 'InitialCondition', '0');
    add_block('simulink/Discrete/Discrete Transfer Fcn', [s '/Output Low-Pass'], ...
        'Numerator', '[1-exp(-2*pi*LKA.Act.LpfCutoff*LKA.Ts)]', ...
        'Denominator', '[1 -exp(-2*pi*LKA.Act.LpfCutoff*LKA.Ts)]', 'SampleTime', '-1');
    add_block('simulink/Math Operations/Abs', [s '/Abs Request']);
    add_block('simulink/Logic and Bit Operations/Compare To Constant', [s '/At Angle Limit'], ...
        'relop', '>=', 'const', 'LKA.Veh.SteerMax - LKA.Act.SatTol');
    ln(s, 'Active/1', 'Active To Double/1');
    ln(s, 'Active To Double/1', 'Authority Ramp/1');
    ln(s, 'SteerReq/1', 'Apply Authority/1');
    ln(s, 'Authority Ramp/1', 'Apply Authority/2');
    ln(s, 'Apply Authority/1', 'Steering Angle Limit/1');
    ln(s, 'Steering Angle Limit/1', 'Steering Rate Limit/1');
    ln(s, 'Steering Rate Limit/1', 'Output Low-Pass/1');
    ln(s, 'Apply Authority/1', 'Abs Request/1');
    ln(s, 'Abs Request/1', 'At Angle Limit/1');
    ln(s, 'Output Low-Pass/1', 'SteerCmd/1');
    ln(s, 'Authority Ramp/1', 'Authority/1');
    ln(s, 'At Angle Limit/1', 'Saturated/1');
    nameOut([s '/Authority Ramp'], 1, 'authority');
    tidy(s);
end

function buildTurningRadius(s)
    mlfb(s, 'Turning Radius From Steering', strjoin({
        'function [R, kappaOut, ay] = fcn(delta, v, LKA)'
        '% Path curvature and turning radius produced by the commanded road-wheel angle'
        'kappaOut = lka_steer_to_curvature(delta, v, LKA.Veh);'
        'Rmax = LKA.Out.RadiusMax;'
        'if abs(kappaOut) > 1/Rmax'
        '    R = 1/kappaOut;'
        'elseif kappaOut >= 0'
        '    R = Rmax;'
        'else'
        '    R = -Rmax;'
        'end'
        'ay = v^2*kappaOut;'}, newline), [200 40 380 160], 'params', {'LKA'});
    ln(s, 'SteerCmd/1', 'Turning Radius From Steering/1');
    ln(s, 'Speed/1', 'Turning Radius From Steering/2');
    ln(s, 'Turning Radius From Steering/1', 'TurnRadius/1');
    ln(s, 'Turning Radius From Steering/2', 'CurvatureOut/1');
    ln(s, 'Turning Radius From Steering/3', 'LatAccel/1');
    tidy(s);
end

function code = packInfoCode()
    code = strjoin({
        'function info = fcn(lkaState, active, authority, kReq, kPP, kSt, kMpc, kFF, kCmd, kMax, Ld, yT, ...'
        '    dReq, dCmd, kOut, R, ay, sat, ldwL, ldwR, dlcL, dlcR, tlcL, tlcR, LKA)'
        'info = struct(''State'', lkaState, ''Active'', active, ''Authority'', authority, ...'
        '    ''ControllerSelect'', double(LKA.ControllerSelect), ''CurvatureReq'', kReq, ...'
        '    ''CurvaturePP'', kPP, ''CurvatureStanley'', kSt, ''CurvatureMPC'', kMpc, ''CurvatureFF'', kFF, ...'
        '    ''CurvatureCmd'', kCmd, ''CurvatureLimit'', kMax, ''Lookahead'', Ld, ''TargetOffset'', yT, ...'
        '    ''SteerReq'', dReq, ''SteerCmd'', dCmd, ''CurvatureOut'', kOut, ''TurnRadius'', R, ...'
        '    ''LatAccelCmd'', ay, ''Saturated'', sat, ''LdwLeft'', ldwL, ''LdwRight'', ldwR, ...'
        '    ''DlcLeft'', dlcL, ''DlcRight'', dlcR, ''TlcLeft'', tlcL, ''TlcRight'', tlcR);'}, newline);
end

% =============================================================================
%                              VirtualLaneSensor.slx
% =============================================================================
function buildSensor(modelDir)
    mdl = 'VirtualLaneSensor';
    new_system(mdl, 'Subsystem');
    names = {'EgoPose', 'CenterPose', 'CenterCurvature', 'LaneWidth', 'Dropout', 'NoiseScale'};
    for k = 1:numel(names)
        add_block('simulink/Sources/In1', [mdl '/' names{k}], 'Port', num2str(k), ...
            'Position', [30 40+80*k 60 54+80*k]);
        add_block('simulink/Discrete/Zero-Order Hold', [mdl '/Frame Sampling ' num2str(k)], ...
            'SampleTime', 'LKA.Sen.Ts', 'Position', [110 35+80*k 140 59+80*k]);
        ln(mdl, [names{k} '/1'], ['Frame Sampling ' num2str(k) '/1']);
    end

    mlfb(mdl, 'Lane Boundary Projection', strjoin({
        'function [bL, bR] = fcn(egoPose, centerPose, curv, laneWidth)'
        '% Ideal detector: lane boundaries as clothoids in the vehicle frame (see lka_lane_geometry.m)'
        '[bL, bR] = lka_lane_geometry(egoPose(:).'', centerPose(:).'', curv(1), curv(2), laneWidth);'}, newline), ...
        [220 120 400 300]);
    for k = 1:4
        ln(mdl, sprintf('Frame Sampling %d/1', k), sprintf('Lane Boundary Projection/%d', k));
    end

    % Gaussian measurement noise, sigma * scenario scale, independent per side
    sides = {'Left', 'Right'};
    add_block('simulink/Sources/Constant', [mdl '/Noise Sigma'], 'Value', 'LKA.Sen.Sigma', 'SampleTime', 'inf');
    for k = 1:2
        rn = sprintf('%s Noise', sides{k});
        add_block('simulink/Sources/Random Number', [mdl '/' rn], 'Mean', 'zeros(4,1)', ...
            'Variance', 'ones(4,1)', 'Seed', sprintf('LKA.Sen.Seed + %d + (0:3)''', 10*k), ...
            'SampleTime', 'LKA.Sen.Ts');
        add_block('simulink/Math Operations/Product', [mdl '/Scale ' rn], 'Inputs', '***', ...
            'Multiplication', 'Element-wise(.*)');
        add_block('simulink/Math Operations/Sum', [mdl '/Add ' rn], 'Inputs', '++');
        ln(mdl, [rn '/1'], ['Scale ' rn '/1']);
        ln(mdl, 'Noise Sigma/1', ['Scale ' rn '/2']);
        ln(mdl, 'Frame Sampling 6/1', ['Scale ' rn '/3']);
        ln(mdl, sprintf('Lane Boundary Projection/%d', k), ['Add ' rn '/1']);
        ln(mdl, ['Scale ' rn '/1'], ['Add ' rn '/2']);
    end

    % Random per-frame misses plus scenario-forced outages
    add_block('simulink/Sources/Uniform Random Number', [mdl '/Miss Draw'], 'Minimum', '[0;0]', ...
        'Maximum', '[1;1]', 'Seed', 'LKA.Sen.Seed + [40;41]', 'SampleTime', 'LKA.Sen.Ts');
    add_block('simulink/Logic and Bit Operations/Compare To Constant', [mdl '/Random Miss'], ...
        'relop', '<', 'const', 'LKA.Sen.DropoutProb');
    add_block('simulink/Logic and Bit Operations/Compare To Constant', [mdl '/Forced Outage'], ...
        'relop', '>', 'const', '0.5');
    add_block('simulink/Logic and Bit Operations/Logical Operator', [mdl '/Boundary Missed'], 'Operator', 'OR', 'Inputs', '2');
    ln(mdl, 'Miss Draw/1', 'Random Miss/1');
    ln(mdl, 'Frame Sampling 5/1', 'Forced Outage/1');
    ln(mdl, 'Random Miss/1', 'Boundary Missed/1');
    ln(mdl, 'Forced Outage/1', 'Boundary Missed/2');

    mlfb(mdl, 'Assemble Detector Output', strjoin({
        'function lanes = fcn(bL, bR, missed, LKA)'
        '% Detector message: a missed boundary is reported with strength 0'
        'lanes.Left  = boundary(bL, ~missed(1), LKA);'
        'lanes.Right = boundary(bR, ~missed(2), LKA);'
        'lanes.FrameValid = true;'
        ''
        'function b = boundary(v, seen, LKA)'
        'if seen'
        '    b = struct(''LateralOffset'', v(1), ''HeadingAngle'', v(2), ''Curvature'', v(3), ...'
        '        ''CurvatureDerivative'', v(4), ''Strength'', LKA.Sen.Strength, ''ViewRange'', LKA.Sen.ViewRange);'
        'else'
        '    b = struct(''LateralOffset'', 0, ''HeadingAngle'', 0, ''Curvature'', 0, ...'
        '        ''CurvatureDerivative'', 0, ''Strength'', 0, ''ViewRange'', 0);'
        'end'}, newline), [560 180 740 320], 'params', {'LKA'}, 'busOut', {'lanes', 'LaneSensorBus'});
    ln(mdl, 'Add Left Noise/1', 'Assemble Detector Output/1');
    ln(mdl, 'Add Right Noise/1', 'Assemble Detector Output/2');
    ln(mdl, 'Boundary Missed/1', 'Assemble Detector Output/3');

    add_block('simulink/Discrete/Delay', [mdl '/Processing Latency'], 'DelayLength', 'LKA.Sen.LatencySteps', ...
        'InitialCondition', '0', 'SampleTime', '-1');
    add_block('simulink/Sinks/Out1', [mdl '/LaneSensor'], 'Port', '1');
    ln(mdl, 'Assemble Detector Output/1', 'Processing Latency/1');
    ln(mdl, 'Processing Latency/1', 'LaneSensor/1');
    tidy(mdl);
    save_system(mdl, fullfile(modelDir, [mdl '.slx']));
end

% =============================================================================
%                              LKA_MIL_TestBench.slx
% =============================================================================
function buildBench(modelDir)
    mdl = 'LKA_MIL_TestBench';
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', '0.005', ...
        'StopTime', 'SCN.StopTime', 'SaveFormat', 'Dataset', 'SignalLogging', 'on', ...
        'SignalLoggingName', 'logsout', 'ReturnWorkspaceOutputs', 'on', ...
        'AlgebraicLoopMsg', 'error');
    set_param(mdl, 'Description', ['Closed-loop MIL test bench: scenario (speed profile, detector outages), ' ...
        'Baja steering actuator and nonlinear single-track dynamics, true lane geometry, emulated lane ' ...
        'detector, LKA_Controller (model reference) and ground-truth wheel-in-lane metrics.']);

    % ---------------- Scenario
    sceOuts = {'SpeedRef', 'Dropout', 'NoiseScale', 'Enable'};
    sce = sub(mdl, 'Scenario', {}, sceOuts, [40 40 200 200]);
    add_block('simulink/Sources/Clock', [sce '/Clock']);
    mlfb(sce, 'Speed Profile and Events', strjoin({
        'function [vRef, dropout, noiseScale, enable] = fcn(t, SCN)'
        '% Driver speed profile v = min(V0 + a*t, VSet) and forced detector outages'
        'vRef = min(SCN.V0 + SCN.Accel*t, SCN.VSet);'
        'dropout = [0; 0];'
        'for k = 1:size(SCN.Dropouts, 1)'
        '    if t >= SCN.Dropouts(k,1) && t < SCN.Dropouts(k,2)'
        '        side = SCN.Dropouts(k,3);'
        '        if side == 1 || side == 3, dropout(1) = 1; end'
        '        if side == 2 || side == 3, dropout(2) = 1; end'
        '    end'
        'end'
        'noiseScale = SCN.NoiseScale;'
        'enable = true;'}, newline), [120 40 300 180], 'params', {'SCN'});
    ln(sce, 'Clock/1', 'Speed Profile and Events/1');
    for k = 1:4
        ln(sce, sprintf('Speed Profile and Events/%d', k), [sceOuts{k} '/1']);
    end
    tidy(sce);

    % ---------------- Driver longitudinal + steering actuator + vehicle
    add_block('simulink/Continuous/Transfer Fcn', [mdl '/Driver Speed Tracking'], ...
        'Numerator', '1', 'Denominator', '[0.3 1]', 'Position', [280 40 360 80]);
    act = sub(mdl, 'Steering Actuator', {'SteerCmd'}, {'SteerActual'}, [700 300 860 360]);
    add_block('simulink/Discontinuities/Rate Limiter', [act '/Steering Rate Limit'], ...
        'RisingSlewLimit', 'vehicle.maxSteerRate', 'FallingSlewLimit', '-vehicle.maxSteerRate');
    add_block('simulink/Discontinuities/Saturation', [act '/Steering Angle Limit'], ...
        'UpperLimit', 'vehicle.maxSteerAngle', 'LowerLimit', '-vehicle.maxSteerAngle');
    add_block('simulink/Continuous/Transfer Fcn', [act '/Actuator Lag'], ...
        'Numerator', '1', 'Denominator', '[vehicle.steerLagTau 1]');
    ln(act, 'SteerCmd/1', 'Steering Rate Limit/1');
    ln(act, 'Steering Rate Limit/1', 'Steering Angle Limit/1');
    ln(act, 'Steering Angle Limit/1', 'Actuator Lag/1');
    ln(act, 'Actuator Lag/1', 'SteerActual/1');
    tidy(act);

    veh = sub(mdl, 'Vehicle Dynamics (Baja single-track)', {'SteerActual', 'Speed'}, ...
        {'CgPose', 'RearAxlePose', 'YawRate', 'LatAccel'}, [950 280 1150 420]);
    mlfb(veh, 'Single-Track Equations', strjoin({
        'function [dx, ay, rearPose, cgPose, yawRate] = fcn(x, delta, vx, vehicle)'
        '% Nonlinear bicycle model with saturating tyres (see lka_single_track_dynamics.m)'
        '[dx, ay, rearPose] = lka_single_track_dynamics(x, delta, vx, vehicle);'
        'cgPose = [x(1) x(2) x(3)];'
        'yawRate = x(5);'}, newline), [200 40 400 220], 'params', {'vehicle'});
    add_block('simulink/Continuous/Integrator', [veh '/States [X Y psi vy r]'], 'InitialCondition', 'SCN.X0');
    ln(veh, 'States [X Y psi vy r]/1', 'Single-Track Equations/1');
    ln(veh, 'SteerActual/1', 'Single-Track Equations/2');
    ln(veh, 'Speed/1', 'Single-Track Equations/3');
    ln(veh, 'Single-Track Equations/1', 'States [X Y psi vy r]/1');
    ln(veh, 'Single-Track Equations/4', 'CgPose/1');
    ln(veh, 'Single-Track Equations/3', 'RearAxlePose/1');
    ln(veh, 'Single-Track Equations/5', 'YawRate/1');
    ln(veh, 'Single-Track Equations/2', 'LatAccel/1');
    tidy(veh);

    % ---------------- True lane geometry at the rear axle (detector input)
    geo = sub(mdl, 'Lane Geometry (ground truth)', {'RearAxlePose'}, {'CenterPose', 'CenterCurvature', 'LaneWidth'}, [1250 120 1430 220]);
    add_block('simulink/Discrete/Zero-Order Hold', [geo '/Detector Rate'], 'SampleTime', 'LKA.Sen.Ts');
    roadLookup(geo, 'Closest Lane Point');
    ln(geo, 'RearAxlePose/1', 'Detector Rate/1');
    ln(geo, 'Detector Rate/1', 'Closest Lane Point/1');
    ln(geo, 'Closest Lane Point/1', 'CenterPose/1');
    ln(geo, 'Closest Lane Point/2', 'CenterCurvature/1');
    ln(geo, 'Closest Lane Point/3', 'LaneWidth/1');
    add_block('simulink/Sinks/Terminator', [geo '/Unused Station']);
    ln(geo, 'Closest Lane Point/4', 'Unused Station/1');
    tidy(geo);

    % ---------------- Emulated lane detector (subsystem reference)
    add_block('simulink/Ports & Subsystems/Subsystem Reference', [mdl '/Virtual Lane Sensor'], ...
        'ReferencedSubsystem', 'VirtualLaneSensor', 'Position', [1520 100 1700 260]);

    % ---------------- LKA controller (model reference)
    rtNames = {'Speed To Controller Rate', 'Yaw Rate To Controller Rate', 'Enable To Controller Rate'};
    for k = 1:3
        add_block('simulink/Signal Attributes/Rate Transition', [mdl '/' rtNames{k}], ...
            'OutPortSampleTime', 'LKA.Ts', 'Position', [1560 300+60*k 1600 330+60*k]);
    end
    add_block('simulink/Ports & Subsystems/Model', [mdl '/LKA Controller'], 'ModelName', 'LKA_Controller', ...
        'SimulationMode', 'Normal', 'Position', [1780 100 1980 400]);

    % ---------------- Ground-truth metrics at the CG
    gt = sub(mdl, 'Ground Truth Metrics', {'CgPose'}, {'LatOffset', 'HeadingErr', 'WheelMargins', 'InLane', 'Station', 'RoadCurvature'}, ...
        [1250 480 1430 640]);
    add_block('simulink/Discrete/Zero-Order Hold', [gt '/Metrics Rate'], 'SampleTime', '0.01');
    roadLookup(gt, 'Closest Lane Point');
    mlfb(gt, 'Wheel-In-Lane Assessment', strjoin({
        'function [latOffset, headingErr, margins, inLane] = fcn(cgPose, centerPose, curv, width, ROAD, vehicle)'
        '% True CG offset, heading error and tyre-edge-to-line margins (rulebook K.2.4)'
        '[latOffset, headingErr, margins, inLane] = lka_ground_truth_metrics(cgPose, centerPose, curv, width, ROAD.lineWidth, vehicle);'}, newline), ...
        [420 40 600 200], 'params', {'ROAD', 'vehicle'});
    add_block('simulink/Signal Routing/Demux', [gt '/Split Curvature'], 'Outputs', '2');
    ln(gt, 'CgPose/1', 'Metrics Rate/1');
    ln(gt, 'Metrics Rate/1', 'Closest Lane Point/1');
    ln(gt, 'Metrics Rate/1', 'Wheel-In-Lane Assessment/1');
    ln(gt, 'Closest Lane Point/1', 'Wheel-In-Lane Assessment/2');
    ln(gt, 'Closest Lane Point/2', 'Wheel-In-Lane Assessment/3');
    ln(gt, 'Closest Lane Point/3', 'Wheel-In-Lane Assessment/4');
    ln(gt, 'Wheel-In-Lane Assessment/1', 'LatOffset/1');
    ln(gt, 'Wheel-In-Lane Assessment/2', 'HeadingErr/1');
    ln(gt, 'Wheel-In-Lane Assessment/3', 'WheelMargins/1');
    ln(gt, 'Wheel-In-Lane Assessment/4', 'InLane/1');
    ln(gt, 'Closest Lane Point/4', 'Station/1');
    ln(gt, 'Closest Lane Point/2', 'Split Curvature/1');
    ln(gt, 'Split Curvature/1', 'RoadCurvature/1');
    add_block('simulink/Sinks/Terminator', [gt '/Unused dkappa']);
    ln(gt, 'Split Curvature/2', 'Unused dkappa/1');
    tidy(gt);

    % ---------------- top-level wiring
    ln(mdl, 'Scenario/1', 'Driver Speed Tracking/1');
    ln(mdl, 'Steering Actuator/1', 'Vehicle Dynamics (Baja single-track)/1');
    ln(mdl, 'Driver Speed Tracking/1', 'Vehicle Dynamics (Baja single-track)/2');
    ln(mdl, 'Vehicle Dynamics (Baja single-track)/2', 'Lane Geometry (ground truth)/1');
    ln(mdl, 'Vehicle Dynamics (Baja single-track)/2', 'Virtual Lane Sensor/1');
    ln(mdl, 'Lane Geometry (ground truth)/1', 'Virtual Lane Sensor/2');
    ln(mdl, 'Lane Geometry (ground truth)/2', 'Virtual Lane Sensor/3');
    ln(mdl, 'Lane Geometry (ground truth)/3', 'Virtual Lane Sensor/4');
    ln(mdl, 'Scenario/2', 'Virtual Lane Sensor/5');
    ln(mdl, 'Scenario/3', 'Virtual Lane Sensor/6');
    ln(mdl, 'Virtual Lane Sensor/1', 'LKA Controller/1');
    ln(mdl, 'Driver Speed Tracking/1', 'Speed To Controller Rate/1');
    ln(mdl, 'Vehicle Dynamics (Baja single-track)/3', 'Yaw Rate To Controller Rate/1');
    ln(mdl, 'Scenario/4', 'Enable To Controller Rate/1');
    ln(mdl, 'Speed To Controller Rate/1', 'LKA Controller/2');
    ln(mdl, 'Yaw Rate To Controller Rate/1', 'LKA Controller/3');
    ln(mdl, 'Enable To Controller Rate/1', 'LKA Controller/4');
    ln(mdl, 'LKA Controller/1', 'Steering Actuator/1');
    ln(mdl, 'Vehicle Dynamics (Baja single-track)/1', 'Ground Truth Metrics/1');

    % ---------------- sinks for logged-only outputs (lines must exist before logging is set)
    for k = 1:6
        add_block('simulink/Sinks/Terminator', sprintf('%s/Metric Sink %d', mdl, k));
        ln(mdl, sprintf('Ground Truth Metrics/%d', k), sprintf('Metric Sink %d/1', k));
    end
    for k = 2:4
        add_block('simulink/Sinks/Terminator', sprintf('%s/LKA Output Sink %d', mdl, k));
        ln(mdl, sprintf('LKA Controller/%d', k), sprintf('LKA Output Sink %d/1', k));
    end
    add_block('simulink/Sinks/Terminator', [mdl '/Lat Accel Sink']);
    ln(mdl, 'Vehicle Dynamics (Baja single-track)/4', 'Lat Accel Sink/1');

    % ---------------- logging
    logOut(mdl, 'Driver Speed Tracking', 1, 'speed');
    logOut(mdl, 'Steering Actuator', 1, 'steer_actual');
    logOut(mdl, 'Vehicle Dynamics (Baja single-track)', 1, 'cg_pose');
    logOut(mdl, 'Vehicle Dynamics (Baja single-track)', 3, 'yaw_rate');
    logOut(mdl, 'Vehicle Dynamics (Baja single-track)', 4, 'lat_accel');
    logOut(mdl, 'Virtual Lane Sensor', 1, 'lane_sensor');
    logOut(mdl, 'Scenario', 2, 'dropout');
    logOut(mdl, 'LKA Controller', 1, 'steer_cmd');
    logOut(mdl, 'LKA Controller', 2, 'turn_radius');
    logOut(mdl, 'LKA Controller', 3, 'lka_info');
    logOut(mdl, 'LKA Controller', 4, 'lane_estimate');
    logOut(mdl, 'Ground Truth Metrics', 1, 'gt_lat_offset');
    logOut(mdl, 'Ground Truth Metrics', 2, 'gt_heading_err');
    logOut(mdl, 'Ground Truth Metrics', 3, 'gt_wheel_margins');
    logOut(mdl, 'Ground Truth Metrics', 4, 'gt_in_lane');
    logOut(mdl, 'Ground Truth Metrics', 5, 'gt_station');
    logOut(mdl, 'Ground Truth Metrics', 6, 'gt_road_curvature');

    tidy(mdl);
    save_system(mdl, fullfile(modelDir, [mdl '.slx']));
end

function roadLookup(s, name)
    mlfb(s, name, strjoin({
        'function [centerPose, curv, width, station] = fcn(pose, ROAD)'
        '% Foot point of the pose on the true lane centreline (windowed search, see lka_road_closest_point.m)'
        'persistent idx'
        'if isempty(idx), idx = 0; end'
        '[centerPose, curv, width, station, idx] = lka_road_closest_point(pose(1), pose(2), idx, ...'
        '    ROAD.s, ROAD.x, ROAD.y, ROAD.psi, ROAD.kappa, ROAD.dkappa, ROAD.width);'}, newline), ...
        [200 40 380 180], 'params', {'ROAD'});
end

% =============================================================================
%                               helpers
% =============================================================================
function path = sub(parent, name, inNames, outNames, pos)
    path = [parent '/' name];
    add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', pos);
    delete_line(path, 'In1/1', 'Out1/1');
    delete_block([path '/In1']);
    delete_block([path '/Out1']);
    for k = 1:numel(inNames)
        add_block('simulink/Sources/In1', [path '/' inNames{k}], 'Port', num2str(k), ...
            'Position', [20 30+70*k 50 44+70*k]);
    end
    for k = 1:numel(outNames)
        add_block('simulink/Sinks/Out1', [path '/' outNames{k}], 'Port', num2str(k), ...
            'Position', [900 30+70*k 930 44+70*k]);
    end
end

function inp(mdl, name, port, pos, dtype)
    p = [mdl '/' name];
    add_block('simulink/Sources/In1', p, 'Port', num2str(port), 'Position', pos, ...
        'OutDataTypeStr', dtype, 'SampleTime', 'LKA.Ts');
    if startsWith(dtype, 'Bus:')
        set_param(p, 'BusOutputAsStruct', 'on');
    end
end

function outp(mdl, name, port, pos, dtype)
    p = [mdl '/' name];
    add_block('simulink/Sinks/Out1', p, 'Port', num2str(port), 'Position', pos, 'OutDataTypeStr', dtype);
    if startsWith(dtype, 'Bus:')
        set_param(p, 'BusOutputAsStruct', 'on');
    end
end

function path = mlfb(sys, name, code, pos, varargin)
    opts = struct('params', {{}}, 'busIn', {{}}, 'busOut', {{}}, 'outSpec', {{}});
    for k = 1:2:numel(varargin)
        opts.(varargin{k}) = varargin{k+1};
    end
    path = [sys '/' name];
    add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', pos);
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', path);
    ch.Script = code;
    for k = 1:numel(opts.params)
        d = ch.find('-isa', 'Stateflow.Data', 'Name', opts.params{k});
        d.Scope = 'Parameter';
        d.Tunable = false;
    end
    for k = 1:2:numel(opts.busIn)
        d = ch.find('-isa', 'Stateflow.Data', 'Name', opts.busIn{k});
        d.DataType = ['Bus: ' opts.busIn{k+1}];
    end
    for k = 1:2:numel(opts.busOut)
        d = ch.find('-isa', 'Stateflow.Data', 'Name', opts.busOut{k});
        d.DataType = ['Bus: ' opts.busOut{k+1}];
    end
    % Explicit output types/sizes, needed where outputs feed back to inputs via delays
    for k = 1:3:numel(opts.outSpec)
        d = ch.find('-isa', 'Stateflow.Data', 'Name', opts.outSpec{k});
        d.DataType = opts.outSpec{k+1};
        d.Props.Array.Size = opts.outSpec{k+2};
    end
end

function ln(sys, src, dst)
    add_line(sys, src, dst, 'autorouting', 'smart');
end

function nameOut(blk, port, name)
    ph = get_param(blk, 'PortHandles');
    lh = get_param(ph.Outport(port), 'Line');
    if lh > 0
        set_param(lh, 'Name', name);
    end
end

function logOut(mdl, blk, port, name)
    nameOut([mdl '/' blk], port, name);
    ph = get_param([mdl '/' blk], 'PortHandles');
    set_param(ph.Outport(port), 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName');
end

function tidy(sys)
    try
        Simulink.BlockDiagram.arrangeSystem(sys);
    catch
    end
end
