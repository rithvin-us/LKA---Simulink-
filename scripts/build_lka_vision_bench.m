function build_lka_vision_bench()
% BUILD_LKA_VISION_BENCH Camera-in-the-loop LKA test bench on the official LKA track.
%
% Syntax:
%   build_lka_vision_bench()
%
% Starts from LKA_MIL_TestBench (same scenarios, Baja plant, ground-truth metrics) and
% replaces the emulated lane sensor with the camera chain:
%   rear-axle pose -> Simulation 3D Vehicle with Ground Following (Unreal, official LKA
%   track exported from RoadRunner as USD: data/scenes/LKA/LKA.usd)
%   -> Simulation 3D Camera (RGB + semantic labels) -> Vision Lane Detector
%   (LKA.Vision.Detector: YOLOP | team CNN | classic) -> processing latency -> LKA_Controller.
% Speed and yaw rate reach the controller through the wheel-speed / gyro sensor model.
% Road geometry and semantic labels are used only for metrics and datasets (rule I.2.3).
% Output: models/LKA_Vision_TestBench.slx

    root = fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params; create_lka_buses;');
    modelDir = fullfile(root, 'models');
    if ~isfile(fullfile(modelDir, 'LKA_MIL_TestBench.slx'))
        build_lka_models();
    end
    mdl = 'LKA_Vision_TestBench';
    for m = {mdl, 'LKA_MIL_TestBench'}
        if bdIsLoaded(m{1}), close_system(m{1}, 0); end
    end
    load_system(fullfile(modelDir, 'LKA_MIL_TestBench.slx'));
    save_system('LKA_MIL_TestBench', fullfile(modelDir, [mdl '.slx']));   % save as
    set_param(mdl, 'Description', ['Camera-in-the-loop LKA bench: Baja single-track plant, Unreal ' ...
        'official LKA track, Simulation 3D Camera, switchable lane detector (YOLOP / team CNN / ' ...
        'classic), wheel-speed and gyro models, LKA_Controller, ground-truth metrics.']);

    % ---------------- remove the emulated lane sensor
    removeBlock([mdl '/Virtual Lane Sensor']);
    terminate(mdl, 'Lane Geometry (ground truth)', 1:3);
    terminate(mdl, 'Scenario', 2:3);

    % ---------------- Unreal scene, ego actor, camera
    scene = [mdl '/Simulation 3D Scene Configuration'];
    add_block('drivingsim3d/Simulation 3D Scene Configuration', scene, 'Position', [40 760 260 840]);
    set_param(scene, 'ProjectFormat', 'RoadRunner', ...
        'RoadRunnerProjectPath', evalin('base', 'LKA.Vision.Bench.ScenePath'), 'Ts', 'LKA.Ts');

    add_block('simulink/Discrete/Zero-Order Hold', [mdl '/Unreal Rate'], 'SampleTime', 'LKA.Ts', ...
        'Position', [1250 700 1290 730]);
    addMlfb(mdl, 'Pose To Unreal', strjoin({
        'function [X, Y, yawDeg] = fcn(rearPose, LKA)'
        '% Track pose (OpenDRIVE / RoadRunner frame) -> Simulation 3D world inputs'
        '[X, Y, yawDeg] = lka_pose_to_sim3d(rearPose, LKA.Vision.Bench.FrameMap);'}, newline), ...
        [1330 690 1480 770]);
    ego = [mdl '/Ego Vehicle (Unreal)'];
    add_block('drivingsim3d/Simulation 3D Vehicle with Ground Following', ego, 'Position', [1540 690 1700 790]);
    set_param(ego, 'PassVehMesh', 'Muscle car', 'ActorName', 'LkaEgo', 'SampleTime', 'LKA.Ts');

    cam = [mdl '/LKA Camera'];
    add_block('drivingsim3d/Simulation 3D Camera', cam, 'Position', [1540 840 1700 920]);
    set_param(cam, 'mountLoc', 'Origin', 'offsetFlag', 'on', ...
        'tmountOffset', '[LKA.Vision.Camera.X, LKA.Vision.Camera.Y, LKA.Vision.Camera.Height + LKA.Vision.Bench.MountZOffset]', ...
        'rmountOffset', '[0, LKA.Vision.Bench.PitchSign * LKA.Vision.Camera.PitchDeg, 0]', ...
        'FocalLength', 'LKA.Vision.Camera.FocalLength', 'OpticalCenter', 'LKA.Vision.Camera.PrincipalPoint', ...
        'ImageSize', 'LKA.Vision.Camera.ImageSize', 'SemanticOutportEnabled', 'on', 'SampleTime', 'LKA.Ts');
    try
        set_param(cam, 'vehTag', 'LkaEgo');
    catch ME
        warning('[LKA vision] Select parent "LkaEgo" in the LKA Camera dialog and save (%s).', ME.message);
    end

    lka_add_vision_detector(mdl, 'Vision Lane Detector', [1780 840 1960 960]);
    add_block('simulink/Sinks/Terminator', [mdl '/Semantic Sink'], 'Position', [1780 990 1800 1010]);

    ln(mdl, 'Vehicle Dynamics (Baja single-track)/2', 'Unreal Rate/1');
    ln(mdl, 'Unreal Rate/1', 'Pose To Unreal/1');
    for k = 1:3
        ln(mdl, sprintf('Pose To Unreal/%d', k), sprintf('Ego Vehicle (Unreal)/%d', k));
    end
    ln(mdl, 'LKA Camera/1', 'Vision Lane Detector/1');
    ln(mdl, 'LKA Camera/2', 'Semantic Sink/1');
    ln(mdl, 'Vision Lane Detector/1', 'LKA Controller/1');

    % ---------------- wheel-speed and gyro models in front of the controller
    delete_line(mdl, 'Speed To Controller Rate/1', 'LKA Controller/2');
    delete_line(mdl, 'Yaw Rate To Controller Rate/1', 'LKA Controller/3');
    es = 'Ego Sensors (wheel speed + IMU)';
    lka_add_ego_sensors(mdl, es, [1640 340 1740 440]);
    ln(mdl, 'Speed To Controller Rate/1', [es '/1']);
    ln(mdl, 'Yaw Rate To Controller Rate/1', [es '/2']);
    ln(mdl, [es '/1'], 'LKA Controller/2');
    ln(mdl, [es '/2'], 'LKA Controller/3');

    % ---------------- annotated video
    rec = [mdl '/Vision Recorder'];
    add_block('simulink/User-Defined Functions/MATLAB System', rec, 'System', 'LkaVisionRecorder', ...
        'Position', [2060 840 2220 960]);
    set_param(rec, 'Vision', 'LKA.Vision', 'Ts', 'LKA.Ts', 'FileName', 'LKA.Vision.Bench.VideoFile');
    add_block('simulink/Math Operations/Gain', [mdl '/Steer Deg'], 'Gain', '180/pi', 'Position', [1990 1000 2030 1030]);
    add_block('simulink/Signal Routing/Bus Selector', [mdl '/Select State'], 'OutputSignals', 'State', ...
        'Position', [1990 1060 2000 1090]);
    ln(mdl, 'LKA Camera/1', 'Vision Recorder/1');
    ln(mdl, 'Vision Lane Detector/2', 'Vision Recorder/2');
    ln(mdl, 'Vision Lane Detector/1', 'Vision Recorder/3');
    ln(mdl, 'LKA Controller/1', 'Steer Deg/1');
    ln(mdl, 'Steer Deg/1', 'Vision Recorder/4');
    ln(mdl, 'LKA Controller/3', 'Select State/1');
    ln(mdl, 'Select State/1', 'Vision Recorder/5');
    terminate(mdl, 'Vision Lane Detector', 3);

    % ---------------- logging
    logOut(mdl, 'Vision Lane Detector', 1, 'lane_sensor');
    logOut(mdl, 'Vision Lane Detector', 3, 'vision_info');
    logOut(mdl, es, 1, 'speed_meas');
    logOut(mdl, es, 2, 'yaw_rate_meas');

    save_system(mdl);
    fprintf('[LKA vision] Built %s\n', fullfile(modelDir, [mdl '.slx']));
end

function removeBlock(blk)
    lh = get_param(blk, 'LineHandles');
    h = [lh.Inport, lh.Outport];
    for k = find(h > 0)
        delete_line(h(k));
    end
    delete_block(blk);
end

function terminate(mdl, blk, ports)
    for p = ports
        t = add_block('simulink/Sinks/Terminator', sprintf('%s/%s Term %d', mdl, blk, p), 'MakeNameUnique', 'on');
        add_line(mdl, sprintf('%s/%d', blk, p), [get_param(t, 'Name') '/1'], 'autorouting', 'smart');
    end
end

function addMlfb(sys, name, code, pos)
    path = [sys '/' name];
    add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', pos);
    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', path);
    ch.Script = code;
    d = ch.find('-isa', 'Stateflow.Data', 'Name', 'LKA');
    d.Scope = 'Parameter';
    d.Tunable = false;
end

function ln(sys, src, dst)
    add_line(sys, src, dst, 'autorouting', 'smart');
end

function logOut(mdl, blk, port, name)
    ph = get_param([mdl '/' blk], 'PortHandles');
    lh = get_param(ph.Outport(port), 'Line');
    if lh > 0
        set_param(lh, 'Name', name);
    end
    set_param(ph.Outport(port), 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName');
end
