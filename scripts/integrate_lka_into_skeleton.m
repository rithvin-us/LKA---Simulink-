function integrate_lka_into_skeleton(options)
% INTEGRATE_LKA_INTO_SKELETON Wires the LKA into the competition model aBaja2026.slx.
%
% Syntax:
%   integrate_lka_into_skeleton()
%   integrate_lka_into_skeleton(Save=false)
%
% What it does (idempotent: an existing LKA integration is removed first):
%   1. Loads SoftwareVirtualWorldSimulation-master/Models/aBaja2026.slx with the
%      LKA calibration for the skeleton plant (lkaProfile = 'skeleton').
%   2. Inside Subsystem2 exposes three signals as new outports:
%        LKA_VehicleInfo  previous-step vehicle state bus (CurrPose, CurrVelocity, CurrYawRate)
%        LKA_RefPose      closest route point [x y yaw_deg]   (emulated sensor / metrics only)
%        LKA_RefCurvature route curvature [1/m]               (emulated sensor / metrics only)
%   3. Adds the root subsystem "LKA (Lane Keeping Assist)":
%        Lane Source (variant subsystem, control variable LKA_LANE_SOURCE):
%          1 Camera Lane Detector - Simulation 3D Camera image -> YOLOP / team CNN / classic
%          2 Emulated Lane Sensor - route ground truth, development only (rule I.2.3)
%        Ego Sensors (wheel speed + IMU) -> LKA_Controller -> rad to deg
%      and drives Subsystem2/Controller_SteerCmd (road-wheel angle, deg, + left).
%   4. Logs steer command, turning radius, LKA info, lane estimate, lane sensor, the
%      measured yaw rate and the pose-derived yaw rate (YawRateSign check, not an input).
%
% With LKA_LANE_SOURCE = 1 the controller receives only camera lane boundaries, the
% wheel-speed speed and the gyro yaw rate. Saving keeps the model in R2026a format.

    arguments
        options.Save (1,1) logical = true
    end

    root = fileparts(fileparts(mfilename('fullpath')));
    skel = fullfile(root, 'SoftwareVirtualWorldSimulation-master');
    run(fullfile(root, 'config', 'startup.m'));
    addpath(fullfile(root, 'models'), fullfile(skel, 'Models'), fullfile(skel, 'SupportingFunctions'));
    evalin('base', 'lkaProfile = ''skeleton''; lka_controller_params; create_lka_buses;');

    mdl = 'aBaja2026';
    load_system(fullfile(skel, 'Models', [mdl '.slx']));
    s2 = [mdl '/Subsystem2'];
    lkaSys = [mdl '/LKA (Lane Keeping Assist)'];
    removeExisting(mdl, s2, lkaSys);

    % ------------------------------------------------ expose signals from Subsystem2
    taps = {
        'LKA_VehicleInfo',  [s2 '/Delay1'], 1
        'LKA_RefPose',      [s2 '/Reference Pose on Path'], 1
        'LKA_RefCurvature', [s2 '/Reference Pose on Path'], 2};
    nOut = numel(find_system(s2, 'SearchDepth', 1, 'BlockType', 'Outport'));
    for k = 1:size(taps, 1)
        src = get_param(taps{k, 2}, 'PortHandles');
        pos = get_param(taps{k, 2}, 'Position');
        blk = add_block('simulink/Sinks/Out1', [s2 '/' taps{k, 1}], 'Port', num2str(nOut + k), ...
            'Position', [pos(3) + 120, pos(2) + 40 * k, pos(3) + 150, pos(2) + 40 * k + 14]);
        dst = get_param(blk, 'PortHandles');
        add_line(s2, src.Outport(taps{k, 3}), dst.Inport(1), 'autorouting', 'smart');
    end

    % ------------------------------------------------ LKA subsystem at root level
    p2 = get_param(s2, 'Position');
    add_block('simulink/Ports & Subsystems/Subsystem', lkaSys, ...
        'Position', [p2(1) - 420, p2(4) + 80, p2(1) - 180, p2(4) + 300]);
    delete_line(lkaSys, 'In1/1', 'Out1/1');
    delete_block([lkaSys '/In1']);
    delete_block([lkaSys '/Out1']);
    ins = {'VehicleInfo', 'RefPose', 'RefCurvature', 'Mode', 'CameraImage'};
    for k = 1:numel(ins)
        add_block('simulink/Sources/In1', [lkaSys '/' ins{k}], 'Port', num2str(k));
    end
    outs = {'SteerCmdDeg', 'TurnRadius', 'LkaInfo', 'LaneEstimate', 'LaneSensor'};
    for k = 1:numel(outs)
        add_block('simulink/Sinks/Out1', [lkaSys '/' outs{k}], 'Port', num2str(k));
    end

    adapter = [lkaSys '/Skeleton Pose Adapter'];
    add_block('simulink/User-Defined Functions/MATLAB Function', adapter);
    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', adapter);
    ch.Script = strjoin({
        'function [egoPose, centerPose, curv, speed, yawRate, enable, yawRatePose] = fcn(veh, refPose, refCurv, mode, LKA)'
        '% Skeleton signals -> LKA conventions (SI units, radians, rear-axle reference).'
        '% speed / yawRate stand for the wheel-speed sensor and the gyro (noise added by the'
        '% Ego Sensors block). egoPose, centerPose, curv feed only the emulated sensor'
        '% (development variant). yawRatePose is differentiated from the ground-truth pose'
        '% and only logged to verify LKA.Cosim.YawRateSign (rule I.2.3: not a controller input).'
        'persistent psiPrev'
        'psi = deg2rad(veh.CurrPose(3) + LKA.Cosim.YawOffsetDeg);'
        'if isempty(psiPrev), psiPrev = psi; end'
        'yawRatePose = lka_wrap_angle(psi - psiPrev) / LKA.Ts;'
        'psiPrev = psi;'
        'yawRate = LKA.Cosim.YawRateSign * deg2rad(veh.CurrYawRate);'
        'egoPose = [veh.CurrPose(1) + LKA.Cosim.PoseToRearAxle*cos(psi), ...'
        '           veh.CurrPose(2) + LKA.Cosim.PoseToRearAxle*sin(psi), psi];'
        'centerPose = [refPose(1), refPose(2), deg2rad(refPose(3) + LKA.Cosim.YawOffsetDeg)];'
        'curv = [refCurv(1), 0];'
        'speed = abs(veh.CurrVelocity);'
        'enable = any(mode == LKA.Cosim.LkaModes);'}, newline);
    d = ch.find('-isa', 'Stateflow.Data', 'Name', 'LKA'); d.Scope = 'Parameter'; d.Tunable = false;
    d = ch.find('-isa', 'Stateflow.Data', 'Name', 'veh'); d.DataType = 'Bus: BusVehicleInfo';

    add_block('simulink/Sources/Constant', [lkaSys '/Lane Width'], 'Value', 'LKA.Lane.WidthNominal', 'SampleTime', 'LKA.Ts');
    add_block('simulink/Sources/Constant', [lkaSys '/No Forced Outage'], 'Value', '[0;0]', 'SampleTime', 'LKA.Ts');
    add_block('simulink/Sources/Constant', [lkaSys '/Noise Scale'], 'Value', '1', 'SampleTime', 'LKA.Ts');
    buildLaneSource(lkaSys);
    lka_add_ego_sensors(lkaSys, 'Ego Sensors (wheel speed + IMU)', [500 300 600 380]);
    add_block('simulink/Ports & Subsystems/Model', [lkaSys '/LKA Controller'], ...
        'ModelName', 'LKA_Controller', 'SimulationMode', 'Normal');
    add_block('simulink/Math Operations/Gain', [lkaSys '/Rad To Deg (+ left)'], 'Gain', '180/pi');
    add_block('simulink/Sinks/Terminator', [lkaSys '/Yaw Rate Check (dev log)']);

    a = 'Skeleton Pose Adapter';
    es = 'Ego Sensors (wheel speed + IMU)';
    ln(lkaSys, 'VehicleInfo/1', [a '/1']);
    ln(lkaSys, 'RefPose/1', [a '/2']);
    ln(lkaSys, 'RefCurvature/1', [a '/3']);
    ln(lkaSys, 'Mode/1', [a '/4']);
    ln(lkaSys, 'CameraImage/1', 'Lane Source/1');
    ln(lkaSys, [a '/1'], 'Lane Source/2');
    ln(lkaSys, [a '/2'], 'Lane Source/3');
    ln(lkaSys, [a '/3'], 'Lane Source/4');
    ln(lkaSys, 'Lane Width/1', 'Lane Source/5');
    ln(lkaSys, 'No Forced Outage/1', 'Lane Source/6');
    ln(lkaSys, 'Noise Scale/1', 'Lane Source/7');
    ln(lkaSys, 'Lane Source/1', 'LKA Controller/1');
    ln(lkaSys, 'Lane Source/1', 'LaneSensor/1');
    ln(lkaSys, [a '/4'], [es '/1']);
    ln(lkaSys, [a '/5'], [es '/2']);
    ln(lkaSys, [es '/1'], 'LKA Controller/2');
    ln(lkaSys, [es '/2'], 'LKA Controller/3');
    ln(lkaSys, [a '/6'], 'LKA Controller/4');
    ln(lkaSys, [a '/7'], 'Yaw Rate Check (dev log)/1');
    ln(lkaSys, 'LKA Controller/1', 'Rad To Deg (+ left)/1');
    ln(lkaSys, 'Rad To Deg (+ left)/1', 'SteerCmdDeg/1');
    ln(lkaSys, 'LKA Controller/2', 'TurnRadius/1');
    ln(lkaSys, 'LKA Controller/3', 'LkaInfo/1');
    ln(lkaSys, 'LKA Controller/4', 'LaneEstimate/1');
    logPort([lkaSys '/' es], 2, 'lka_yaw_rate_meas');
    logPort([lkaSys '/' a], 7, 'lka_yaw_rate_pose');
    try Simulink.BlockDiagram.arrangeSystem(lkaSys); catch, end

    % ------------------------------------------------ root wiring
    ln(mdl, sprintf('Subsystem2/%d', nOut + 1), 'LKA (Lane Keeping Assist)/1');
    ln(mdl, sprintf('Subsystem2/%d', nOut + 2), 'LKA (Lane Keeping Assist)/2');
    ln(mdl, sprintf('Subsystem2/%d', nOut + 3), 'LKA (Lane Keeping Assist)/3');
    ln(mdl, 'Mode/1', 'LKA (Lane Keeping Assist)/4');
    ln(mdl, 'Simulation 3D Camera/1', 'LKA (Lane Keeping Assist)/5');
    ln(mdl, 'LKA (Lane Keeping Assist)/1', 'Subsystem2/1');

    names = {'lka_steer_cmd_deg', 'lka_turn_radius', 'lka_info', 'lane_estimate', 'lka_lane_sensor'};
    ph = get_param(lkaSys, 'PortHandles');
    for k = 1:numel(names)
        if k > 1
            t = add_block('simulink/Sinks/Terminator', sprintf('%s/LKA Log Sink %d', mdl, k), ...
                'Position', get_param(lkaSys, 'Position') + [300 40*k 270 40*k-140]);
            tp = get_param(t, 'PortHandles');
            add_line(mdl, ph.Outport(k), tp.Inport(1), 'autorouting', 'smart');
        end
        set_param(get_param(ph.Outport(k), 'Line'), 'Name', names{k});
        set_param(ph.Outport(k), 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName');
    end
    set_param(mdl, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
    set_param(lkaSys, 'Description', ['aBaja 2026 LKA: lane source (camera detector YOLOP / team CNN / ' ...
        'classic, or emulated for development), wheel-speed and gyro models, LKA_Controller (lane tracker, ' ...
        'supervisor, pure pursuit / Stanley 2025 / MPC, a_y-jerk limiter, actuator manager). Output: ' ...
        'road-wheel angle [deg, + left] to Subsystem2/Controller_SteerCmd.']);

    if options.Save
        save_system(mdl);
        fprintf('[LKA] Integrated and saved %s\n', get_param(mdl, 'FileName'));
    end
end

function buildLaneSource(lkaSys)
    vs = [lkaSys '/Lane Source'];
    add_block('simulink/Ports & Subsystems/Variant Subsystem', vs, 'Position', [300 40 480 260]);
    template = find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', ...
        'MatchFilter', @Simulink.match.allVariants);
    for k = numel(template):-1:2                     % library template: In1, Out1, 2 choices
        delete_block(template{k});
    end
    ins = {'CameraImage', 'EgoPose', 'CenterPose', 'Curvature', 'LaneWidth', 'Outage', 'NoiseScale'};
    for k = 1:numel(ins)
        add_block('built-in/Inport', [vs '/' ins{k}], 'Port', num2str(k), 'Position', [20 30 + 40 * k 50 44 + 40 * k]);
    end
    add_block('built-in/Outport', [vs '/LaneSensor'], 'Port', '1', 'Position', [600 100 630 114]);

    c1 = [vs '/Camera Lane Detector'];
    add_block('built-in/SubSystem', c1, 'VariantControl', 'LKA_LANE_SOURCE == 1', 'Position', [200 40 380 140]);
    choicePorts(c1, ins);
    lka_add_vision_detector(c1, 'Vision Lane Detector', [200 40 380 160]);
    add_line(c1, 'CameraImage/1', 'Vision Lane Detector/1');
    add_line(c1, 'Vision Lane Detector/1', 'LaneSensor/1');
    for k = 2:3
        add_block('built-in/Terminator', sprintf('%s/Vision Out %d', c1, k));
        add_line(c1, sprintf('Vision Lane Detector/%d', k), sprintf('Vision Out %d/1', k));
    end
    terminateInputs(c1, ins(2:end));

    c2 = [vs '/Emulated Lane Sensor (development only)'];
    add_block('built-in/SubSystem', c2, 'VariantControl', 'LKA_LANE_SOURCE == 2', 'Position', [200 180 380 280]);
    choicePorts(c2, ins);
    add_block('simulink/Ports & Subsystems/Subsystem Reference', [c2 '/Virtual Lane Sensor'], ...
        'ReferencedSubsystem', 'VirtualLaneSensor');
    for k = 2:numel(ins)
        add_line(c2, [ins{k} '/1'], sprintf('Virtual Lane Sensor/%d', k - 1));
    end
    add_line(c2, 'Virtual Lane Sensor/1', 'LaneSensor/1');
    terminateInputs(c2, ins(1));
end

function choicePorts(c, ins)
    for k = 1:numel(ins)
        add_block('built-in/Inport', [c '/' ins{k}], 'Port', num2str(k), 'Position', [20 30 + 40 * k 50 44 + 40 * k]);
    end
    add_block('built-in/Outport', [c '/LaneSensor'], 'Port', '1', 'Position', [600 100 630 114]);
end

function terminateInputs(c, names)
    for k = 1:numel(names)
        add_block('built-in/Terminator', [c '/Unused ' names{k}]);
        add_line(c, [names{k} '/1'], ['Unused ' names{k} '/1']);
    end
end

function logPort(blk, port, name)
    ph = get_param(blk, 'PortHandles');
    lh = get_param(ph.Outport(port), 'Line');
    if lh > 0
        set_param(lh, 'Name', name);
    end
    set_param(ph.Outport(port), 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName');
end

function removeExisting(mdl, s2, lkaSys)
    if getSimulinkBlockHandle(lkaSys) > 0
        lh = get_param(lkaSys, 'LineHandles');
        delete_line([lh.Inport(lh.Inport > 0), lh.Outport(lh.Outport > 0)]);
        delete_block(lkaSys);
    end
    sinks = find_system(mdl, 'SearchDepth', 1, 'Regexp', 'on', 'Name', '^LKA Log Sink');
    for k = 1:numel(sinks)
        delete_block(sinks{k});
    end
    for nm = {'LKA_VehicleInfo', 'LKA_RefPose', 'LKA_RefCurvature'}
        b = [s2 '/' nm{1}];
        if getSimulinkBlockHandle(b) > 0
            lh = get_param(b, 'LineHandles');
            if lh.Inport > 0, delete_line(lh.Inport); end
            delete_block(b);
        end
    end
    % Dangling lines left at the root after removing ports
    delete_line(find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line', 'Connected', 'off'));
end

function ln(sys, src, dst)
    add_line(sys, src, dst, 'autorouting', 'smart');
end
