function results = run_lka_vision(options)
% RUN_LKA_VISION Runs the camera-in-the-loop LKA bench (Unreal) for detectors and scenarios.
%
% Syntax:
%   results = run_lka_vision()
%   results = run_lka_vision(Scenarios={'S1_Nominal','S2_SpeedLimit'}, Detectors={'yolop','classic'})
%
% Name-value options:
%   Scenarios  - scenario names from generate_lka_scenarios (default {'S1_Nominal'})
%   Detectors  - {'yolop' | 'cnn' | 'classic'} (default {'yolop'})
%   Controller - 1 pure pursuit, 2 Stanley 2025, 3 MPC (default 1)
%   Video      - write results/vision/<scenario>_<detector>.mp4 (default true)
%   Dataset    - record frames for the detector benchmark (default false)
%   OutDir     - default results/vision
%   Rebuild    - rebuild LKA_Vision_TestBench first (default false)
% Output: struct array (res, kpi); CSV lka_vision_kpis.csv and figures in OutDir.

    arguments
        options.Scenarios cell = {'S1_Nominal'}
        options.Detectors cell = {'yolop'}
        options.Controller (1,1) double = 1
        options.Video (1,1) logical = true
        options.Dataset (1,1) logical = false
        options.OutDir char = ''
        options.Rebuild (1,1) logical = false
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    if isempty(options.OutDir)
        options.OutDir = fullfile(root, 'results', 'vision');
    end
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params; create_lka_buses;');
    vehicle = evalin('base', 'vehicle');
    LKA0 = evalin('base', 'LKA');
    mdl = 'LKA_Vision_TestBench';
    if options.Rebuild || ~isfile(fullfile(root, 'models', [mdl '.slx']))
        build_lka_vision_bench();
    end
    load_system(mdl);
    scenarios = generate_lka_scenarios(vehicle);
    scenarios = scenarios(ismember({scenarios.Name}, options.Scenarios));
    results = struct('res', {}, 'kpi', {});
    for s = 1:numel(scenarios)
        sc = scenarios(s);
        r = sc.Road;
        assignin('base', 'SCN', sc.P);
        assignin('base', 'ROAD', struct('s', r.s, 'x', r.x, 'y', r.y, 'psi', r.psi, 'kappa', r.kappa, ...
            'dkappa', r.dkappa, 'width', r.width, 'lineWidth', r.lineWidth));
        for d = 1:numel(options.Detectors)
            LKA = LKA0;
            LKA.ControllerSelect = options.Controller;
            LKA.Vision.Detector = options.Detectors{d};
            tag = sprintf('%s_%s', sc.Name, LKA.Vision.Detector);
            LKA.Vision.Bench.VideoFile = '';
            if options.Video
                LKA.Vision.Bench.VideoFile = fullfile(options.OutDir, [tag '.mp4']);
            end
            LKA.Vision.Bench.DatasetDir = '';
            if options.Dataset
                LKA.Vision.Bench.DatasetDir = fullfile(root, 'data', 'vision_datasets', tag);
            end
            assignin('base', 'LKA', LKA);
            fprintf('[LKA vision] %-16s camera %-8s ... ', sc.Name, LKA.Vision.Detector);
            t0 = tic;
            simOut = sim(mdl);
            res = lka_extract_results(simOut, sc, LKA, vehicle);
            res.ControllerName = sprintf('%s, camera %s', res.ControllerName, LKA.Vision.Detector);
            kpi = lka_compute_kpis(res);
            vi = squeeze(simOut.logsout.getElement('vision_info').Values.Data);
            if size(vi, 1) == 7, vi = vi'; end
            kpi.Detector = string(LKA.Vision.Detector);
            kpi.VisionMsMean = mean(vi(2:end, 7));
            kpi.BothLinesRatio = mean(vi(:, 3) & vi(:, 4));
            kpi.WallClock_s = toc(t0);
            fprintf('%5.0f s | max|lat| %.3f m | in lane %5.1f %% | %.1f ms/frame\n', kpi.WallClock_s, ...
                kpi.MaxLatOffset_m, 100 * kpi.InLaneRatio, kpi.VisionMsMean);
            plot_lka_results(res, kpi, options.OutDir);
            close all
            results(end + 1) = struct('res', res, 'kpi', kpi); %#ok<AGROW>
        end
    end
    assignin('base', 'LKA', LKA0);
    if ~isfolder(options.OutDir), mkdir(options.OutDir); end
    % merge with earlier runs: one row per (scenario, controller, detector), newest wins
    csv = fullfile(options.OutDir, 'lka_vision_kpis.csv');
    T = struct2table(arrayfun(@(x) x.kpi, results));
    T.Scenario = string(T.Scenario); T.Controller = string(T.Controller); T.Detector = string(T.Detector);
    if isfile(csv)
        old = readtable(csv, 'TextType', 'string');
        if isequal(sort(old.Properties.VariableNames), sort(T.Properties.VariableNames))
            keep = ~ismember(old.Scenario + "|" + old.Controller, T.Scenario + "|" + T.Controller);
            T = [old(keep, T.Properties.VariableNames); T];
        end
    end
    writetable(T, csv);
end
