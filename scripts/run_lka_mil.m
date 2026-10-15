function results = run_lka_mil(options)
% RUN_LKA_MIL Runs the LKA MIL test bench over scenarios and controllers.
%
% Syntax:
%   results = run_lka_mil()
%   results = run_lka_mil(Scenarios={'S1_Nominal'}, Controllers=1, Plot=true)
%
% Name-value options:
%   Scenarios    - cellstr of scenario names (default: all from generate_lka_scenarios)
%   Controllers  - controller ids, 1 pure pursuit, 2 Stanley 2025, 3 MPC (default 1:3)
%   Plot         - write result figures (default true)
%   OutDir       - output folder (default results/mil)
%   Rebuild      - regenerate the Simulink models first (default false; automatic if missing)
%   CloseFigures - close figures after saving them (default true)
%
% Output:
%   results - struct array (one per run) with fields res (signals) and kpi.
% Also written to OutDir: PNG figures, lka_mil_kpis.csv and lka_mil_results.mat.

    arguments
        options.Scenarios cell = {}
        options.Controllers double = 1:3
        options.Plot (1,1) logical = true
        options.OutDir char = ''
        options.Rebuild (1,1) logical = false
        options.CloseFigures (1,1) logical = true
    end

    root = fileparts(fileparts(mfilename('fullpath')));
    if isempty(options.OutDir)
        options.OutDir = fullfile(root, 'results', 'mil');
    end
    run(fullfile(root, 'config', 'startup.m'));
    addpath(fullfile(root, 'models'), fullfile(root, 'scripts'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params;');
    vehicle = evalin('base', 'vehicle');
    LKA0 = evalin('base', 'LKA');

    if options.Rebuild || ~isfile(fullfile(root, 'models', 'LKA_MIL_TestBench.slx'))
        build_lka_models();
    end
    load_system('LKA_MIL_TestBench');

    scenarios = generate_lka_scenarios(vehicle);
    if ~isempty(options.Scenarios)
        scenarios = scenarios(ismember({scenarios.Name}, options.Scenarios));
    end

    results = struct('res', {}, 'kpi', {});
    for s = 1:numel(scenarios)
        sc = scenarios(s);
        r = sc.Road;
        ROAD = struct('s', r.s, 'x', r.x, 'y', r.y, 'psi', r.psi, 'kappa', r.kappa, ...
            'dkappa', r.dkappa, 'width', r.width, 'lineWidth', r.lineWidth);
        assignin('base', 'SCN', sc.P);
        assignin('base', 'ROAD', ROAD);
        perScenario = struct('res', {}, 'kpi', {});
        for c = options.Controllers
            LKA = LKA0;
            LKA.ControllerSelect = c;
            if sc.P.LatencySteps >= 0
                LKA.Sen.LatencySteps = sc.P.LatencySteps;
                LKA.KF.LatencyComp = sc.P.LatencySteps * LKA.Sen.Ts;
            end
            assignin('base', 'LKA', LKA);
            fprintf('[LKA MIL] %-18s controller %d ... ', sc.Name, c);
            t0 = tic;
            simOut = sim('LKA_MIL_TestBench');
            res = lka_extract_results(simOut, sc, LKA, vehicle);
            kpi = lka_compute_kpis(res);
            fprintf('%5.1f s | max|lat| %.3f m | in lane %5.1f %% | min margin %+.3f m\n', ...
                toc(t0), kpi.MaxLatOffset_m, 100 * kpi.InLaneRatio, kpi.MinWheelMargin_m);
            if options.Plot
                plot_lka_results(res, kpi, options.OutDir);
                if options.CloseFigures, close all; end
            end
            perScenario(end+1) = struct('res', res, 'kpi', kpi); %#ok<AGROW>
        end
        if options.Plot && numel(perScenario) > 1
            plot_lka_comparison(perScenario, options.OutDir);
            if options.CloseFigures, close all; end
        end
        results = [results, perScenario]; %#ok<AGROW>
    end
    assignin('base', 'LKA', LKA0);

    if ~isfolder(options.OutDir), mkdir(options.OutDir); end
    T = struct2table(arrayfun(@(r) r.kpi, results));
    writetable(T, fullfile(options.OutDir, 'lka_mil_kpis.csv'));
    kpis = T; %#ok<NASGU>
    save(fullfile(options.OutDir, 'lka_mil_results.mat'), 'kpis');
    disp(T(:, {'Scenario', 'Controller', 'MaxLatOffset_m', 'RmsLatOffset_m', 'MinWheelMargin_m', ...
        'InLaneRatio', 'MaxLatAccel_mps2', 'MaxLatJerk05_mps3', 'MinTurnRadius_m', 'TakeOverRequests', 'RulebookPass'}));
end
