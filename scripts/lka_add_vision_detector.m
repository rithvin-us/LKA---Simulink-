function path = lka_add_vision_detector(sys, name, pos)
% LKA_ADD_VISION_DETECTOR Adds the camera lane-detector subsystem (Image -> LaneSensorBus).
%
% Syntax:
%   path = lka_add_vision_detector(sys, name, pos)
%
% Contents: MATLAB System LkaVisionLaneDetector (Vision = LKA.Vision, Ts = LKA.Ts) and a
% Delay of LKA.Vision.LatencySteps frames (processing latency, compensated in the lane
% tracker by LKA.KF.LatencyComp).
% Outputs: 1 LaneSensor (delayed), 2 Preview (bird's-eye probability), 3 VisionInfo.
    path = [sys '/' name];
    add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', pos);
    delete_line(path, 'In1/1', 'Out1/1');
    delete_block([path '/In1']);
    delete_block([path '/Out1']);
    add_block('simulink/Sources/In1', [path '/Image'], 'Port', '1', 'Position', [20 100 50 114]);
    det = [path '/Lane Detector'];
    add_block('simulink/User-Defined Functions/MATLAB System', det, 'System', 'LkaVisionLaneDetector', ...
        'Position', [120 70 320 190]);
    set_param(det, 'Vision', 'LKA.Vision', 'Ts', 'LKA.Ts');
    add_block('simulink/Discrete/Delay', [path '/Processing Latency'], 'DelayLength', 'LKA.Vision.LatencySteps', ...
        'InitialCondition', '0', 'SampleTime', 'LKA.Ts', 'Position', [400 80 440 110]);
    outs = {'LaneSensor', 'Preview', 'VisionInfo'};
    for k = 1:3
        add_block('simulink/Sinks/Out1', [path '/' outs{k}], 'Port', num2str(k), ...
            'Position', [520 60 + 50 * k 550 74 + 50 * k]);
    end
    set_param([path '/LaneSensor'], 'OutDataTypeStr', 'Bus: LaneSensorBus');
    add_line(path, 'Image/1', 'Lane Detector/1');
    add_line(path, 'Lane Detector/1', 'Processing Latency/1');
    add_line(path, 'Processing Latency/1', 'LaneSensor/1');
    add_line(path, 'Lane Detector/2', 'Preview/1');
    add_line(path, 'Lane Detector/3', 'VisionInfo/1');
end
